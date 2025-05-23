#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>

#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <syslog.h>

#include "rttask.h"


using namespace std;
using namespace hgz;

const string INDI_PORT { "-p 7624" };
const string _cmd_driftscan = "rt_transitscan %f %f %s %f %d";
const string _cmd_tracking = "rt_tracking %f %f %s %f %d";
const string _cmd_horscan = "rt_scan_hor %f %f %f %f %s %f %f %f %d";
const string _cmd_equscan = "rt_scan_equ %f %f %f %f %s %f %f %f %d";
const string INDI_DEVICE { "Pi Radiotelescope" };
const string INDI_PROP_HOR_COORD { INDI_DEVICE+".HORIZONTAL_EOD_COORD" };
const string INDI_PROP_EQU_COORD { INDI_DEVICE+".EQUATORIAL_EOD_COORD" };
const string INDI_WAIT_IDLE { "indi_eval "+INDI_PORT+" -w -t 100 '\""+INDI_DEVICE+".SCOPE_STATUS.SCOPE_IDLE\"==1'" };
const string INDI_PROP_PARK { INDI_DEVICE+".TELESCOPE_PARK.PARK" };
const string INDI_PROP_UNPARK { INDI_DEVICE+".TELESCOPE_PARK.UNPARK" };
const string INDI_WAIT_PARKED { "indi_eval "+INDI_PORT+" -w -t 100 '\""+INDI_DEVICE+".SCOPE_STATUS.SCOPE_PARKED\"==1'" };

template <class T>
std::string to_string(T t, std::ios_base & (*f)(std::ios_base&))
{
  std::ostringstream oss;
  oss << f << t;
  return oss.str();
}


//
// RTTask
//

int RTTask::fNumTasks=0;
bool RTTask::fAnyActive=false;
std::string RTTask::fDataPath="";
std::string RTTask::fExecutablePath="";


RTTask::~RTTask()
{
	if ( fState == ACTIVE ) Stop();
	fNumTasks--;
}


int RTTask::RunShellCommand(const char *strCommand)
{
	int iForkId, iStatus;
	iForkId = vfork();
	if (iForkId == 0)       // This is the child
	{
		int setsidstatus=setsid();
		//int pgidstatus=setpgid(getpid(),0);
		syslog (LOG_DEBUG, "setsid() status= %d", setsidstatus);
		//syslog (LOG_DEBUG, "changed PGID to %d (status=%d)", getpid(), pgidstatus);
		syslog (LOG_DEBUG, "running shell command: %s", strCommand);
		iStatus = execl("/bin/sh","sh","-c", strCommand, (char*) NULL);
		// We must exit here, or we will have multiple mainlines running...
		exit(iStatus);
	}
	else if (iForkId > 0)   // Parent, no error
	{
		// return pid of child process
		iStatus = iForkId;
		// wait 10ms to be sure that the parent exited
		usleep(10000);
	}
	else    // Parent, with error (iForkId == -1)
	{
		iStatus = -1;
	}
	return(iStatus);
}


int RTTask::Start()
{
	if (fState==FINISHED) return (int)FINISHED;
	if (fState==ACTIVE) return (int)ACTIVE;
	if (fState==CANCELLED) return (int)CANCELLED;
	if (fState==ERROR) return (int)ERROR;
	if (fState==STOPPED) return (int)STOPPED;
	if (fAnyActive) { 
		fState = WAITING;
		return fState; 
	}
	fState=ACTIVE;
	fAnyActive=true;
	fStartTime=Time::Now();
	return 0;
}

int RTTask::Stop()
{
	if (fState==FINISHED) return (int)FINISHED;
	if (fState==ACTIVE) {
		for (int i=0; i<fPIDList.size(); i++) {
			int iDeadId=0;
			while (iDeadId==0) {
//				int iStatus=kill(fPIDList[i], SIGKILL);
				// kill all child processes in process group ID = PID of child
				int iStatus=kill(-fPIDList[i], SIGKILL);
				usleep(10000);
				int iChildiStatus;
				iDeadId = waitpid(fPIDList[i], &iChildiStatus, WNOHANG);
				if (iDeadId>0) {
					syslog (LOG_INFO, "stopping child processes with PGID %d, kill=%d", fPIDList[i], iStatus);
					//result=0;
				}
				else if (iDeadId<0) {
					syslog (LOG_ERR, "failed to stop child processes with PGID %d; kill=%d, waitpid=%d", fPIDList[i], iStatus, iDeadId);
					//result=-1;
				}
			}
		}
		fPIDList.clear();
		fState=STOPPED;
		fAnyActive=false;
	} else if ( fState==CANCELLED || fState==STOPPED || fState==ERROR ) {
		return fState;
	} else fState=STOPPED;
	return 0;
}

int RTTask::Cancel()
{
	int result=Stop();
	fState=CANCELLED;
	return result;
}

double RTTask::Eta() const
{ 
	if (fState==CANCELLED || fState==STOPPED || fState==ERROR || fState == FINISHED) return 0.;
	return fMaxRunTime-fElapsedTime;
}

void RTTask::Print() const
{
   std::cout<<"RT Task:\n";
   std::cout<<setfill('0');
   std::cout<<" id    : "<<setw(8)<<hex<<fId<<dec<<std::endl;
}

auto RTTask::WriteHeader( const std::string& datafile ) -> bool {
	std::ofstream file( datafile, ios_base::out | ios_base::trunc );
	if ( file.fail() || !file.good() ) {
		syslog (LOG_DEBUG, "RTTask::WriteHeader: error opening data file %s", datafile.c_str());
		return false;
	}
	file << "# " << tasktype_string.at(fType) << "\n";
	file << "# Task ID: " << fId << "\n";
	file << "# Submit time: " << fSubmitTime << "\n";
	file << "# Schedule time: " << fScheduleTime << "\n";
	file << "# Start time: " << fStartTime << "\n";
	file << "# Max run time: " << fMaxRunTime << "h\n";
	file << "# User: " << fUser << "\n";
	file << "# Priority: " << fPriority << "\n";
	file << "# Comment: " << fComment << "\n";
	file << flush;
	return true;
}

void RTTask::Process()
{
	if ( fState == FINISHED || fState == STOPPED || fState == CANCELLED || fState == ERROR ) return;
	if (fVerbose>4) cout<<"RTTask::Process()"<<endl;
	// handle an active task here
	if (fState==ACTIVE) {
		// Wait till the commands complete
		int iChildiStatus = 0;
		int iDeadId = waitpid(-1, &iChildiStatus, WNOHANG);
		if (iDeadId < 0)
		{
			// Wait id error
			// something really bad happened, so terminate the task and set state to error
			syslog (LOG_ERR, "waitpid error: %u", iDeadId);
			Stop();
			fState = ERROR;
		}
		else if (iDeadId > 0)
		{
			// child process finished, so just mark the task as finished
			syslog (LOG_DEBUG, "child proc finished: pid = %u", iDeadId);
			fPIDList.clear();
			fAnyActive=false;
			//RTTask::Stop();	// need to call the base-class method only
									// since the measurement process finished alone
			fState=FINISHED;
			return;
		} else {
			// iDeadId == 0, no processes died
			// the task remains active, so do nothing
		}

		if ((fElapsedTime=(Time::Now().timestamp()-fStartTime.timestamp())/3600.)>fMaxRunTime) {
			// max. runtime constraint fulfilled; stop the measurement by force
			Stop();
			if (fVerbose>3) cout<<"RTTask::Process(): forcefully stopped task - maximum runtime exceeded"<<endl;
			fState=FINISHED;
			//fElapsedTime=fMaxRunTime;
		}
		return;
	}

	// handle the task, if it is idle or waiting
	if ( fScheduleTime.timestamp()-Time::Now().timestamp()<0. ) {
		// the schedule time of the task is up, check if it can be executed
		if ( !fAnyActive ) {
			if (fVerbose>3) cout<<"RTTask::Process(): started task"<<endl;
			Start();
		} else {
			fState = WAITING;
			if ( ( fScheduleTime.timestamp() + fMaxRunTime * 3600. - Time::Now().timestamp() ) < 0. ) {
				// the latest scheduled execution time is surpassed
				// check if the task can be executed at a later or any time
				if ( fAltPeriod < -1e-4 ) {
					// settings indicate that the task must have executed at the specified time
					// since this was not possible, cancel the task for good
					fState=CANCELLED;
				} else if ( fAltPeriod > 1e-4  ) {
					// task can be repeated at a later time with the indicated repetition interval
					// so reschedule the task to the next possible time slot
					fScheduleTime += fAltPeriod * 3600.;
				}
			}
		}
	}
}


//
// DriftScanTask
//
int DriftScanTask::Start()
{
	if (fVerbose>3) cout<<"DriftScanTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// here code to do the measurement
		string cmdstring;
		char tmpstr[256];
		char datafilestr[256];
		sprintf(datafilestr,"task_drift%04d%02d%02d_%05d",fStartTime.year(),fStartTime.month(),fStartTime.day(),(long)fStartTime.timestamp()%86400L);
		fDataFile=string(datafilestr);
//		fDataFile="task_drift"+to_string<long>((long)fStartTime.timestamp(), std::dec);
		bool file_success = WriteHeader( ((fDataPath.empty()) ? "" : fDataPath+"/" ) + fDataFile );
		if ( !file_success) {
			fState = ERROR;
			syslog (LOG_ERR, "failed to start driftscan task with id=%d: data file i/o error", this->ID());
			return (int)ERROR;
		}

		double intTime { 1. };
		#if __cplusplus > 199711L
		if (isnormal(fIntTime))
		#else
		if (isnormal<double>(fIntTime))
		#endif		
		{
			intTime = fIntTime;
		}
		
		cmdstring="";
		if ( !fExecutablePath.empty() ) {
			cmdstring="cd "+fExecutablePath+" && ";
		}
		sprintf(tmpstr, string(_cmd_driftscan).c_str(), (float)fStartCoords.Phi(), 
				(float)fStartCoords.Theta(), 
				string( ( (fDataPath.empty()) ? "" : fDataPath+"/" ) + fDataFile).c_str(),
				intTime,
                fRefInterval
   			);
		cmdstring+=tmpstr;

		syslog (LOG_DEBUG, "executing command: %s", cmdstring.c_str());
		int iStatus = RunShellCommand(cmdstring.c_str());
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting driftscan task with id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to start driftscan task with id=%d", this->ID());
			fState = ERROR;
			result=-1;
		}
	}
	return result;
}

int DriftScanTask::Stop()
{
	if (fVerbose>3) cout<<"DriftScanTask::Stop()"<<endl;
	int result=RTTask::Stop();
	syslog (LOG_NOTICE, "stopping driftscan task with id=%d", this->ID());
	return result;
}

auto DriftScanTask::WriteHeader( const std::string& datafile ) -> bool {
        bool success = RTTask::WriteHeader( datafile );
	if ( !success ) return false;
	std::ofstream file( datafile, ios_base::out | ios_base::app );
	if ( file.fail() || !file.good() ) {
		return false;
	}
	file << "#------------------------------------------\n";
	file << "# Coordinates: Az=" << fStartCoords.Phi() << " Alt=" << fStartCoords.Theta() << "\n";
	file << "# Integration time: " << fIntTime << "s\n";
	file << flush;
	return true;
}

//
// TrackingTask
//

int TrackingTask::Start()
{
	if (fVerbose>3) cout<<"TrackingTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// here code to do the measurement
		string cmdstring;
		char tmpstr[256];
		char datafilestr[256];
		sprintf(datafilestr,"task_track%04d%02d%02d_%05d",fStartTime.year(),fStartTime.month(),fStartTime.day(),(long)fStartTime.timestamp()%86400L);
		fDataFile=string(datafilestr);
//		fDataFile="task"+to_string<long>((long)fStartTime.timestamp(), std::dec);
		bool file_success = WriteHeader( ((fDataPath.empty()) ? "" : fDataPath+"/" ) + fDataFile );
		if ( !file_success) {
			fState = ERROR;
			syslog (LOG_ERR, "failed to start tracking scan task with id=%d: data file i/o error", this->ID());
			return (int)ERROR;
		}

		double intTime { 1. };
		#if __cplusplus > 199711L
		if (isnormal(fIntTime))
		#else
		if (isnormal<double>(fIntTime))
		#endif		
		{
			intTime = fIntTime;
		}
		
		cmdstring="";
		if ( !fExecutablePath.empty() ) {
			cmdstring="cd "+fExecutablePath+" && ";
		}
		sprintf(tmpstr, string(_cmd_tracking).c_str(), (float)fTrackCoords.Phi(), 
				(float)fTrackCoords.Theta(),
				string( ( (fDataPath.empty()) ? "" : fDataPath+"/" ) + fDataFile).c_str(),
				intTime,
                fRefInterval
   			);
		cmdstring+=tmpstr;

		syslog (LOG_DEBUG, "executing command: %s", cmdstring.c_str());
		int iStatus = RunShellCommand(cmdstring.c_str());
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting tracking task with id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to start tracking task with id=%d", this->ID());
			fState = ERROR;
			result=-1;
		}
	}
	return result;
}

int TrackingTask::Stop()
{
	if (fVerbose>3) cout<<"TrackingTask::Stop()"<<endl;
	return RTTask::Stop();
}

auto TrackingTask::WriteHeader( const std::string& datafile ) -> bool {
        bool success = RTTask::WriteHeader( datafile );
	if ( !success ) return false;
	std::ofstream file( datafile, ios_base::out | ios_base::app );
	if ( file.fail() || !file.good() ) {
		return false;
	}
	file << "#------------------------------------------\n";
	file << "# Coordinates: RA=" << fTrackCoords.Phi() << " Dec=" << fTrackCoords.Theta() << "\n";
	file << "# Integration time: " << fIntTime << "s\n";
	file << flush;
	return true;
}


//
// HorScanTask
//

int HorScanTask::Start()
{
	if (fVerbose>3) cout<<"HorScanTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// here code to start the measurement
		string cmdstring;
		char tmpstr[512];
		char datafilestr[256];
		sprintf(datafilestr,"task_horscan%04d%02d%02d_%05d",fStartTime.year(),fStartTime.month(),fStartTime.day(),(long)fStartTime.timestamp()%86400L);
		fDataFile=string(datafilestr);
//		fDataFile="task"+to_string<long>((long)fStartTime.timestamp(), std::dec);
		bool file_success = WriteHeader( ((fDataPath.empty()) ? "" : fDataPath+"/" ) + fDataFile );
		if ( !file_success) {
			fState = ERROR;
			syslog (LOG_ERR, "failed to start horscan task with id=%d: data file i/o error", this->ID());
			return (int)ERROR;
		}

		double stepAz { 1. };
		double stepAlt { 1. };
		double intTime { 1. };
		#if __cplusplus > 199711L
		if (isnormal(fStepAz) && isnormal(fStepAlt)) 
		#else
		if (isnormal<double>(fStepAz) && isnormal<double>(fStepAlt)) 
		#endif		
		{
			stepAz = fStepAz;
			stepAlt = fStepAlt;
		}
		#if __cplusplus > 199711L
		if (isnormal(fIntTime))
		#else
		if (isnormal<double>(fIntTime))
		#endif		
		{
			intTime = fIntTime;
		}
		
		cmdstring="";
		if ( !fExecutablePath.empty() ) {
			cmdstring="cd "+fExecutablePath+" && ";
		}
		sprintf(tmpstr, string(_cmd_horscan).c_str(),(float)fStartCoords.Phi(), (float)fEndCoords.Phi(),
		 fStartCoords.Theta(), fEndCoords.Theta(),
		 string( ( (fDataPath.empty()) ? "" : fDataPath+"/" ) + fDataFile).c_str(),
		 stepAz, stepAlt, intTime, fRefInterval
		);
		cmdstring+=tmpstr;

		syslog (LOG_DEBUG, "executing command: %s", cmdstring.c_str());
		int iStatus = RunShellCommand(cmdstring.c_str());
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting HorScan task with id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to start HorScan task with id=%d", this->ID());
			fState = ERROR;
			result=-1;
		}
	}
	return result;
}

int HorScanTask::Stop()
{
	if (fVerbose>3) cout<<"HorScanTask::Stop()"<<endl;
	return RTTask::Stop();
}

auto HorScanTask::WriteHeader( const std::string& datafile ) -> bool {
        bool success = RTTask::WriteHeader( datafile );
	if ( !success ) return false;
	std::ofstream file( datafile, ios_base::out | ios_base::app );
	if ( file.fail() || !file.good() ) {
		return false;
	}
	file << "#------------------------------------------\n";
	file << "# Start coordinates: Az=" << fStartCoords.Phi() << "deg Alt=" << fStartCoords.Theta() << "deg\n";
	file << "# End coordinates: Az=" << fEndCoords.Phi() << "deg Alt=" << fEndCoords.Theta() << "deg\n";
	file << "# Step size: Az=" << fStepAz << "deg Alt=" << fStepAlt << "deg\n";
	file << "# Integration time: " << fIntTime << "s\n";
	file << flush;
	return true;
}


//
// EquScanTask
//

int EquScanTask::Start()
{
	if (fVerbose>3) cout<<"EquScanTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// here code to do the measurement
		string cmdstring;
		char tmpstr[256];
		char datafilestr[256];
		sprintf(datafilestr,"task_equscan%04d%02d%02d_%05d",fStartTime.year(),fStartTime.month(),fStartTime.day(),(long)fStartTime.timestamp()%86400L);
		fDataFile=string(datafilestr);
//		fDataFile="task"+to_string<long>((long)fStartTime.timestamp(), std::dec);
		bool file_success = WriteHeader( ((fDataPath.empty()) ? "" : fDataPath+"/" ) + fDataFile );
		if ( !file_success) {
			fState = ERROR;
			syslog (LOG_ERR, "failed to start equscan task with id=%d: data file i/o error", this->ID());
			return (int)ERROR;
		}

		double stepRa { 0.067 };
		double stepDec { 1. };
		double intTime { 1. };
		#if __cplusplus > 199711L
		if (isnormal(fStepRa) && isnormal(fStepDec)) 
		#else
		if (isnormal<double>(fStepRa) && isnormal<double>(fStepDec)) 
		#endif		
		{
			stepRa = fStepRa;
			stepDec = fStepDec;
		}
		#if __cplusplus > 199711L
		if (isnormal(fIntTime))
		#else
		if (isnormal<double>(fIntTime))
		#endif		
		{
			intTime = fIntTime;
		}
		
		cmdstring="";
		if ( !fExecutablePath.empty() ) {
			cmdstring="cd "+fExecutablePath+" && ";
		}

		sprintf(tmpstr, string(_cmd_equscan).c_str(),(float)fStartCoords.Phi(), (float)fEndCoords.Phi(),
			(float)fStartCoords.Theta(), (float)fEndCoords.Theta(),
			string( ( (fDataPath.empty()) ? "" : fDataPath+"/" ) + fDataFile).c_str(),
			stepRa, stepDec, intTime, fRefInterval
			);
		cmdstring+=tmpstr;

		syslog (LOG_DEBUG, "executing command: %s", cmdstring.c_str());
		int iStatus = RunShellCommand(cmdstring.c_str());
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting EquScan task with id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to start EquScan task with id=%d", this->ID());
			fState = ERROR;
			result=-1;
		}
	}
	return result;
}

int EquScanTask::Stop()
{
	if (fVerbose>3) cout<<"EquScanTask::Stop()"<<endl;
	return RTTask::Stop();
}

auto EquScanTask::WriteHeader( const std::string& datafile ) -> bool {
        bool success = RTTask::WriteHeader( datafile );
	if ( !success ) return false;
	std::ofstream file( datafile, ios_base::out | ios_base::app );
	if ( file.fail() || !file.good() ) {
		return false;
	}
	file << "#------------------------------------------\n";
	file << "# Start coordinates: RA=" << fStartCoords.Phi() << "h Dec=" << fStartCoords.Theta() << "deg\n";
	file << "# End coordinates: RA=" << fEndCoords.Phi() << "h Dec=" << fEndCoords.Theta() << "deg\n";
	file << "# Step size: RA=" << fStepRa << "h = " << 360.*fStepRa/24. <<"deg  Dec=" << fStepDec << "deg\n";
	file << "# Integration time: " << fIntTime << "s\n";
	file << flush;
	return true;
}

//
// GotoHorTask
//

int GotoHorTask::Start()
{
	if (fVerbose>3) cout<<"GotoHorTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		char cmdstr[256];
		// first, send goto command to indi
		sprintf(cmdstr,"echo -n $(indi_setprop %s \"%s.AZ;ALT=%f;%f\")", INDI_PORT.c_str(), INDI_PROP_HOR_COORD.c_str(),fGotoCoords.Phi(),fGotoCoords.Theta());
		int returnvalue = system (cmdstr);
		syslog (LOG_DEBUG, "executing command: %s , code %d", cmdstr, returnvalue);
		
		// wait a bit to let the goto command commence and RT state change from idle to slew
		usleep( 400000UL );
		
		// now, wait until pos reached
		sprintf( cmdstr, INDI_WAIT_IDLE.c_str() );
/*
		sprintf(	cmdstr,
					"indi_eval -p 7273 -t 2 -w 'abs(\"%s.AZ\"-%f)<.085 && abs(\"%s.ALT\"-%f)<.085'",
					INDI_HOR_COORD_PROPERTY.c_str(), (float)fGotoCoords.Phi(), 
					INDI_HOR_COORD_PROPERTY.c_str(), (float)fGotoCoords.Theta());
*/
		syslog (LOG_DEBUG, "executing command: %s", cmdstr);
		int iStatus = RunShellCommand(cmdstr);
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting slew to horizontal coordinate, task id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to slew to horizontal coordinate, task id=%d", this->ID());
			fState = ERROR;
			result=-1;
		}
	}
	return result;
}

int GotoHorTask::Stop()
{
	if (fVerbose>3) cout<<"GotoHorTask::Stop()"<<endl;
	int result=RTTask::Stop();
	syslog (LOG_NOTICE, "stopping goto hor task with id=%d", this->ID());
	return result;
}


//
// GotoEquTask
//

int GotoEquTask::Start()
{
	if (fVerbose>3) cout<<"GotoEquTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		// here code to do the measurement
		char cmdstr[256];
		// first, send goto command to indi
		sprintf(cmdstr,"echo -n $(indi_setprop %s \"%s.RA;DEC=%f;%f\")", INDI_PORT.c_str(), INDI_PROP_EQU_COORD.c_str(),fGotoCoords.Phi(),fGotoCoords.Theta());
		int returnvalue = system (cmdstr);

		// wait a bit to let the goto command commence and RT state change from idle to slew
		usleep( 400000UL );

		// now, wait until pos reached
		sprintf( cmdstr, INDI_WAIT_IDLE.c_str() );
/*
		sprintf(	cmdstr,
					"indi_eval -p 7273 -t 2 -w 'abs(\"LX200 RT.EQUATORIAL_EOD_COORD.RA\"-%f)<.007 && abs(\"LX200 RT.EQUATORIAL_EOD_COORD.DEC\"-%f)<.09'",
					(float)fGotoCoords.Phi(), (float)fGotoCoords.Theta());
*/
		syslog (LOG_DEBUG, "executing command: %s", cmdstr);
		int iStatus = RunShellCommand(cmdstr);
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting slew to equatorial coordinate, task id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to slew to equatorial coordinate, task id=%d", this->ID());
			fState = ERROR;
			result=-1;
		}
	}
	return result;
}

int GotoEquTask::Stop()
{
	if (fVerbose>3) cout<<"GotoEquTask::Stop()"<<endl;
	int result=RTTask::Stop();
	syslog (LOG_NOTICE, "stopping goto equ task with id=%d", this->ID());
	return result;
}


//
// MaintenanceTask
//

int MaintenanceTask::Start()
{
   if (fVerbose>3) cout<<"MaintenanceTask::Start()"<<endl;
   int result=RTTask::Start();
   if (result==0) {
      char cmdstr[256];
      // simply issue a forked-off sleep command with the duration of the task's maximum runtime 
      double duration { fMaxRunTime * 3600. - 0.25 };
	  if ( duration < 1e-3 ) duration = 1e-3;
	  sprintf(cmdstr,"sleep %f",duration);
      syslog (LOG_DEBUG, "executing command: %s", cmdstr);
      int iStatus = RunShellCommand(cmdstr);
      if (iStatus>0) {
         syslog (LOG_NOTICE, "starting maintenance task, task id=%d (pid %d)", this->ID(), iStatus);
         fPIDList.push_back(iStatus);
         result=0;
      }
      else {
         syslog (LOG_ERR, "failed to start maintenance task, task id=%d", this->ID());
         result=-1;
      }
   }
   return result;
}

int MaintenanceTask::Stop()
{
   if (fVerbose>3) cout<<"MaintenanceTask::Stop()"<<endl;
   int result=RTTask::Stop();
   syslog (LOG_NOTICE, "stopping maintenance task with id=%d", this->ID());
   return result;
}

//
// ParkTask
//

int ParkTask::Start()
{
	if (fVerbose>3) cout<<"ParkTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		char cmdstr[256];
		// first, send park command to indi
		sprintf(cmdstr,"echo -n $(indi_setprop %s \"%s=On\")", INDI_PORT.c_str(), INDI_PROP_PARK.c_str());
		int returnvalue = system (cmdstr);
		syslog (LOG_DEBUG, "executing command: %s , code %d", cmdstr, returnvalue);
		
		// now, wait until scope confirmes that it reached park position
		sprintf( cmdstr, INDI_WAIT_PARKED.c_str() );
		syslog (LOG_DEBUG, "executing command: %s", cmdstr);
		int iStatus = RunShellCommand(cmdstr);
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting park task, task id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		}
		else {
			syslog (LOG_ERR, "failed to start park task, task id=%d", this->ID());
			result=-1;
		}
	}
	return result;
}

int ParkTask::Stop()
{
   if (fVerbose>3) cout<<"ParkTask::Stop()"<<endl;
   int result=RTTask::Stop();
   syslog (LOG_NOTICE, "stopping park task with id=%d", this->ID());
   return result;
}

//
// UnparkTask
//

int UnparkTask::Start()
{
	if (fVerbose>3) cout<<"UnparkTask::Start()"<<endl;
	int result=RTTask::Start();
	if (result==0) {
		char cmdstr[256];
		// first, send unpark command to indi
		sprintf(cmdstr,"echo -n $(indi_setprop %s \"%s=On\")", INDI_PORT.c_str(), INDI_PROP_UNPARK.c_str());
		int returnvalue = system (cmdstr);
		syslog (LOG_DEBUG, "executing command: %s , code %d", cmdstr, returnvalue);

		// now, wait until scope becomes idle
		sprintf( cmdstr, INDI_WAIT_IDLE.c_str() );
		syslog (LOG_DEBUG, "executing command: %s", cmdstr);
		int iStatus = RunShellCommand(cmdstr);
		if (iStatus>0) {
			syslog (LOG_NOTICE, "starting unpark task, task id=%d (pid %d)", this->ID(), iStatus);
			fPIDList.push_back(iStatus);
			result=0;
		} else {
			syslog (LOG_ERR, "failed to start unpark task, task id=%d", this->ID());
			result=-1;
		}
	}
	return result;
}

int UnparkTask::Stop()
{
   if (fVerbose>3) cout<<"UnparkTask::Stop()"<<endl;
   int result=RTTask::Stop();
   syslog (LOG_NOTICE, "stopping unpark task with id=%d", this->ID());
   return result;
}
