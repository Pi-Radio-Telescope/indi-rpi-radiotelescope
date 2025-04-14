#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <syslog.h>
#include <unistd.h>		// for getopt()
#include <ctype.h>
#include <fcntl.h>
#include <signal.h>

#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>

#include "ratsche_message.h"
#include "rttask.h"
#include "time.h"

using namespace std;
using namespace hgz;

constexpr int MSQ_ID { 42 };
constexpr unsigned long server_loop_delay_us { 20000UL };
constexpr size_t MAX_MSG_LEN { 4*8192UL };

const string defaultTaskFile = "/var/ratsche/ratsche_tasks";

void Usage(const char* progname)
{
	cout<<"RaTSche - The Radiotelescope Task Scheduler"<<endl;
	cout<<"v1.1 - HG Zaunick 2010-2011,2021-22"<<endl;
	cout<<endl;
	cout<<" Usage : "<<string(progname)<<"  [-vlrEdph?] -k <keyID> -e|c|s <taskID> -a <taskfile> -x|o <path>"<<endl;
	cout<<"  command line options are:   "<<endl;
	cout<<"	 -l            list all tasks"<<endl;
	cout<<"	 -r            reverse sort of data output (with -l)"<<endl;
	cout<<"	 -p            export tasklist (for storage in file) to stdout"<<endl;
	cout<<"	 -k <keyID>    use message queue with key keyID"<<endl;
	cout<<"	 -a <taskfile> add task(s) supplied in file taskfile"<<endl;
	cout<<"	 -a -          add single task supplied through stdin"<<endl;
	cout<<"	 -c <taskID>   cancel task with ID taskId"<<endl;
	cout<<"	 -s <taskID>   stop task with ID taskId"<<endl;
	cout<<"	 -e <taskID>   erase task with ID taskId"<<endl;
	cout<<"	 -E            erase all tasks"<<endl;
	cout<<"	 -d            run as daemon (scheduling server) and fork to background"<<endl;
	cout<<"	 -x <path>     path to the executable macros"<<endl;
	cout<<"	 -o <path>     path to data output"<<endl;
	cout<<"	 -v            increase verbosity level for stderr and syslog"<<endl;
	cout<<"	 -h -?         show this help and exit"<<endl;
	cout<<endl;
}


void error(const string& progname, const string& message) {
//	cerr<<string(asctime(localtime(time(NULL))))<<" "<<<<endl;
	char str[100];
	time_t now=time(NULL);
	strftime(str, 100, "%Y/%m/%d %H:%M:%S", localtime(&now));
	cerr<<string(str)<<" error in "<<progname<<" : "<<message<<endl;
}


int send_message(int msqid, int fromID, int toID, int action, int subaction, task_t* task, int seriesID=1, int seriesCount=1) {
	message_t smsg;
	int result;
//	int pid=getpid();

	smsg.mtype = toID;
	smsg.maction = action;
	smsg.msubaction = subaction;
	smsg.msenderID=fromID;
	smsg.mseriesID=seriesID;
	smsg.mseriesCount=seriesCount;

	if (task!=NULL) {
		smsg.mtask=*task;
		(void) strcpy(smsg.mtask.user, task->user);
		(void) strcpy(smsg.mtask.comment, task->comment);
	}

	size_t buf_length = sizeof(smsg);

	unsigned long int cnt=0;
	while ((result=msgsnd(msqid, &smsg, buf_length, IPC_NOWAIT)) < 0 && cnt++<100) {
		perror("msgsnd");
        usleep(10000UL);
	}
	if (result<0) {
		syslog (LOG_CRIT, "error in msgsnd: unable to access message queue");
		return -1;
	}

//	syslog (LOG_DEBUG, "Sent %d bytes\n", buf_length);
//   printf("Message: Sent %d bytes\n", buf_length);
   return result;
}

int receive_message(int msqid, int* fromID, int toID, int* action, int* subaction, task_t* task, int* seriesID=NULL, int* seriesCount=NULL) {
	message_t rmsg;
//	int pid=getpid();

//	size_t buf_length = sizeof(rmsg);
	size_t buf_length = MAX_MSG_LEN;
	int result;

	unsigned long int cnt=0;
	while ((result=msgrcv(msqid, &rmsg, buf_length, toID, IPC_NOWAIT)) < 0 && cnt++<100) {
		if (errno==ENOMSG){
			// no message, return simply
			return -1;
		}
		// wait 10ms and hope that access to the queue will work in a new iteration
		usleep(10000);
	}
	if (result<0) {
		syslog (LOG_CRIT, "error in msgrcv: unable to access message queue");
		return -1;
	}

	if (fromID!=NULL) *fromID=rmsg.msenderID;
	if (action!=NULL) *action=rmsg.maction;
	if (subaction!=NULL) *subaction=rmsg.msubaction;
	if (seriesID!=NULL) *seriesID=rmsg.mseriesID;
	if (seriesCount!=NULL) *seriesCount=rmsg.mseriesCount;
	if (task!=NULL) {
		*task=rmsg.mtask;
		(void) strcpy(task->user, rmsg.mtask.user);
		(void) strcpy(task->comment, rmsg.mtask.comment);
	}
	return result;
}

bool save_tasks(const std::string& filename, const std::vector<task_t>& tasklist)
{
	uint32_t num_tasks = tasklist.size();
	std::ofstream file( filename, ios_base::out | ios_base::trunc | ios::binary );
	if ( file.fail() || !file.good() ) {
		return false;
	}
	file.write( reinterpret_cast<char*>(&num_tasks), sizeof(uint32_t) );
	for ( task_t task : tasklist ) {
		file.write( reinterpret_cast<char*>(&task), sizeof(task_t) );
	}
	return true;
}

bool load_tasks(const std::string& filename, std::vector<task_t>& tasklist)
{
	tasklist.clear();
	uint32_t num_tasks { 0 };
	std::ifstream file( filename, ios_base::in | ios::binary );
	if ( file.fail() || !file.good() ) {
		return false;
	}
	file.read( reinterpret_cast<char*>(&num_tasks), sizeof(uint32_t) );
	if ( !num_tasks ) return true;
	for ( int i = 0; i < num_tasks; i++ ) {
		task_t task { };
		file.read( reinterpret_cast<char*>(&task), sizeof(task_t) );
		tasklist.emplace_back( std::move(task) );
	}
	return true;
}

void export_tasks(std::ostream& ostr, const vector<task_t>& tasklist) {
	ostr<<"# RT TASK"<<endl;
	ostr<<"# v0.2"<<endl;
	ostr<<"# task file for definition of measurement(s) with the PiRaTe radio telescope"<<endl;
	ostr<<"# fields are ignored or assumed with defaults when marked with '*'"<<endl;
	ostr<<"# priority = {0=ignore|1=immediate|2=immediate when free|3=asap when optimal|4=anytime when optimal|5=low priority}"<<endl;
	ostr<<"# mode = { drift | track | equscan | horscan | gotohor | gotoequ | park | unpark | maintenance }"<<endl;
	ostr<<"# alt_period : time period after which the conditions are expected identical and the measurement can equally commence (hours)"<<endl;
	ostr<<"#              (e.g. 24h for scans of stellar objects),"<<endl;
	ostr<<"#              or  0 = task may be started any time,"<<endl;
	ostr<<"#                 -1 = task shall only be started in the given time window defined by start time and max-duration"<<endl;
	ostr<<"#                  * = don't care, start the task when convenient"<<endl;
	ostr<<"# x1,y1: coordinates of the lower left corner of the scanwindow for 2d-scans (Az/Alt for Hor; RA/Dec for Equ)"<<endl;
	ostr<<"#     or coordinates of measurement position for drift tasks (Az/Alt)"<<endl;
	ostr<<"#     or coordinates of measurement position for track tasks (RA/Dec)"<<endl;
	ostr<<"# x2,y2: coordinates of the upper right corner of the scanwindow for 2d-scans (Az/Alt for Hor; RA/Dec for Equ)"<<endl;
	ostr<<"# stepx,stepy : step sizes for 2d scans (deg/deg for Hor; hours/deg for Equ)"<<endl;
	ostr<<"# int_time : detector adc integration time constant in seconds"<<endl;
	ostr<<"# ref_cycle : N/A"<<endl;
	ostr<<"# max duration : maximum allowed run time of task (hours)"<<endl;
	ostr<<"# meaning of columns:"<<endl;
	ostr<<"# start_time mode priority alt_period user x1 y1 x2 y2 stepx stepy int_time ref_cycle max_duration comment"<<endl;

	for ( task_t task : tasklist ){
		char str[100];
		strftime(str, 100, "%Y/%m/%d %H:%M:%S", localtime(&task.start_time));
		ostr << string(str) << " " << static_cast<int>(task.type) << " " << static_cast<int>(task.priority) << " "
			<< task.alt_period << " " << string(task.user) << " "
			<< task.coords1.x << " " <<task.coords1.y << " "
			<< task.coords2.x << " " <<task.coords2.y << " "
			<< task.step1 << " " << task.step2 <<" "
			<< task.int_time << " " << task.ref_cycle << " " << task.duration
			<< " \"" << string(task.comment) << "\""<<endl;
   }
   return;
}

int parseStringForTask( const std::string& str, task_t& task ) {
		string line(str);
		if (line.empty()) return -1;
		// remove comment lines
		size_t found=line.find_first_of('#');
		if (found!=string::npos) {
			line=line.erase(found, string::npos);
		}
		// remove leading spaces
		while (!line.empty() && (isspace(line[0]) || isdigit(line[0])==0)) line.erase(0,1);
		if (line.empty()) return -1;
		// parse line
		istringstream isline(line);
		string _date,_time,_mode,_user,_prio;
		string _alt_period, _x1, _x2, _y1, _y2, _step1, _step2, _int_time, _ref_cycle, _duration;
		isline>>_date>>_time>>_mode>>_prio>>_alt_period>>_user
				>>_x1>>_y1>>_x2>>_y2>>_step1>>_step2>>_int_time>>_ref_cycle>>_duration;
				
		if ( isline.tellg() != -1 && !isline.fail() ) {
			// allocate memory:
			char buffer[256];

			// read data as a block:
			int length = isline.readsome (buffer,256);
			
			string comment ( buffer, buffer+length );
			
			while ( isspace(comment.front()) || comment.front() == '\"' ) comment.erase(0,1);
			while ( !comment.empty() && ( comment.back() == '\"' || !isgraph(comment.back()) ) ) comment.pop_back();
			
			if ( !comment.empty() ) {
				strcpy(task.comment, comment.c_str());
			} else {
				strcpy(task.comment, "");
			}
		} else {
			strcpy(task.comment, "");
		}
		
		strcpy(task.user, _user.c_str());
		
		if (_mode=="drift" || _mode=="DRIFT") {
			task.type=RTTask::DRIFT;
		} else if (_mode=="track" || _mode=="TRACK") {
			task.type=RTTask::TRACK;
		} else if (_mode=="equscan" || _mode=="EQUSCAN") {
			task.type=RTTask::EQUSCAN;
		} else if (_mode=="horscan" || _mode=="HORSCAN") {
			task.type=RTTask::HORSCAN;
		} else if (_mode=="gotohor" || _mode=="GOTOHOR") {
			task.type=RTTask::GOTOHOR;
		} else if (_mode=="gotoequ" || _mode=="GOTOEQU") {
			task.type=RTTask::GOTOEQU;
		} else if (_mode=="maintenance" || _mode=="MAINTENANCE") {
			task.type=RTTask::MAINTENANCE;
		} else if (_mode=="park" || _mode=="PARK") {
			task.type=RTTask::PARK;
		} else if (_mode=="unpark" || _mode=="UNPARK") {
			task.type=RTTask::UNPARK;
		} else {
			errno=0;
			task.type=strtol(_mode.c_str(),NULL,10);
			if (errno) {
				error("ratsche", "could not determine the task type");
				return -1;
			}
		}
		errno=0;
		task.priority=strtol(_prio.c_str(),NULL,10);
		if (errno || !isdigit(_prio[0])) {
			error("","could not determine priority");
		}
		errno=0;
		task.alt_period=0.;
		task.alt_period=strtod(_alt_period.c_str(),NULL);
		if (errno || _alt_period[0]=='*') {
			error("","could not determine alt_period, setting to -1 (=singular)");
		}

		errno=0;
		task.coords1.x=strtod(_x1.c_str(),NULL);
		if (errno || _x1[0]=='*') {
			error("","could not determine x1, setting to NAN");
			task.coords1.x=NAN;
		}
		errno=0;
		task.coords1.y=strtod(_y1.c_str(),NULL);
		if (errno || _y1[0]=='*') {
			error("","could not determine y1, setting to NAN");
			task.coords1.y=NAN;
		}
		errno=0;
		task.coords2.x=strtod(_x2.c_str(),NULL);
		if (errno || _x2[0]=='*') {
			error("","could not determine x2, setting to NAN");
			task.coords2.x=NAN;
		}
		errno=0;
		task.coords2.y=strtod(_y2.c_str(),NULL);
		if (errno || _y2[0]=='*') {
			error("","could not determine y2, setting to NAN");
			task.coords2.y=NAN;
		}
		errno=0;
		task.step1=strtod(_step1.c_str(),NULL);
		if (errno || _step1[0]=='*') {
			error("","could not determine step1, setting to NAN");
			task.step1=NAN;
		}
		errno=0;
		task.step2=strtod(_step2.c_str(),NULL);
		if (errno || _step2[0]=='*') {
			error("","could not determine step2, setting to NAN");
			task.step2=NAN;
		}
		errno=0;
		task.int_time=strtod(_int_time.c_str(),NULL);
		if (errno || _int_time[0]=='*') {
			error("","could not determine int-time, setting to NAN");
			task.int_time=NAN;
		}
		errno=0;
		task.ref_cycle=strtod(_ref_cycle.c_str(),NULL);
		if (errno || _ref_cycle[0]=='*') {
			error("","could not determine ref_cycle, setting to 0");
			task.ref_cycle=0;
		}
		errno=0;
		task.duration=strtod(_duration.c_str(),NULL);
		if (errno || _duration[0]=='*') {
			error("","could not determine duration, setting to 1h");
			task.duration=1.;
		}

		task.submit_time=time(NULL);
		task.status=0;
		task.eta=-1.;
		task.elapsed=0.;

		struct tm starttm;
		int y,m;
		sscanf(_date.c_str(),"%4d%*c%2d%*c%2d",&y,&m,&starttm.tm_mday);

		starttm.tm_year=y-1900;
		starttm.tm_mon=m-1;
		float s;
		sscanf(_time.c_str(),"%2d%*c%2d%*c%02f",&starttm.tm_hour,&starttm.tm_min,&s);
		starttm.tm_sec=(int)s;
		starttm.tm_isdst=-1;
		task.start_time=mktime(&starttm);
		return 0;
}

int importTasklistFromFile(const string& filename, vector<task_t>& tasklist) {
	// parse file for following format:
	// start-time mode priority alt-period user x1 y1 x2 y2 step1 step2 int-time ref-cycle max_duration comment
	ifstream infile(filename.c_str());
	if (!infile.good() || infile.eof() || !infile.is_open()) return -1;
	while (!infile.eof()) {
		task_t task;
		string line;
		getline(infile,line);
		if ( parseStringForTask(line, task) == 0 ) {
			tasklist.push_back(task);
		}
	}
	return 0;
}

void print_tasklist(const vector<task_t>& tasklist) {
	cout<<"# id time mode priority alt-period user x1 y1 x2 y2 step1 step2 int-time ref-cycle max-duration elapsed eta status comment"<<endl;
	for ( task_t task : tasklist ){
		char str[100];
		strftime(str, 100, "%Y/%m/%d %H:%M:%S", localtime(&task.start_time));
		cout << task.id << " " << string(str) << " " << static_cast<int>(task.type) << " " << static_cast<int>(task.priority) << " "
			 << task.alt_period << " " << string(task.user) << " "
			 << task.coords1.x << " " << task.coords1.y << " "
			 << task.coords2.x << " " << task.coords2.y << " "
			 << task.step1 << " " << task.step2 << " "
			 << task.int_time << " " << task.ref_cycle << " "
			 << task.duration << " " << task.elapsed << " " << task.eta << " " << task.status
			 << " \"" << string(task.comment) << "\""<<endl;
	}
	return;
}

void print_task(const task_t& task) {
	cout<<"*** Task "<<task.id<<" ***"<<endl;
	cout<<" type       : "<<(int)task.type<<endl;
	cout<<" priority   : "<<(int)task.priority<<endl;
	printf(" start time : %s",asctime(localtime(&task.start_time)));
	printf(" submit time: %s",asctime(localtime(&task.submit_time)));
	cout<<" user       : "<<string(task.user)<<endl;
	cout<<" alt. period: "<<task.alt_period<<endl;
	cout<<" 1st point  : ("<<task.coords1.x<<" , "<<task.coords1.y<<")"<<endl;
	cout<<" 2nd point  : ("<<task.coords2.x<<" , "<<task.coords2.y<<")"<<endl;
	cout<<" step(x)    : "<<task.step1<<endl;
	cout<<" step(y)    : "<<task.step2<<endl;
	cout<<" int time   : "<<task.int_time<<endl;
	cout<<" ref cycle  : "<<task.ref_cycle<<endl;
	cout<<" max. duration : "<<task.duration<<endl;
	cout<<" elapsed time  : "<<task.elapsed<<endl;
	cout<<" eta        : "<<task.eta<<endl;
	cout<<" status     : "<<task.status<<endl;
	cout<<" comment    : "<<string(task.comment)<<endl;
}

task_t toMsgTask(RTTask* task)
{
	task_t msgtask;
	msgtask.id=task->ID();
	msgtask.priority=task->Priority();
	msgtask.type=(int)task->type();
	msgtask.start_time=task->scheduleTime().timestamp();
	msgtask.submit_time=task->submitTime().timestamp();
	msgtask.int_time=task->IntTime();
	msgtask.ref_cycle=task->RefInterval();
	msgtask.alt_period=task->AltPeriod();
	msgtask.coords1.x=0.;
	msgtask.coords1.y=0.;
	msgtask.coords2.x=0.;
	msgtask.coords2.y=0.;
	msgtask.step1=0.;
	msgtask.step2=0.;
	msgtask.duration=task->MaxRunTime();
	msgtask.elapsed=task->ElapsedTime();
	msgtask.eta=task->Eta();
	msgtask.status=task->State();
	strcpy(msgtask.user, task->User().c_str());
	strcpy(msgtask.comment, task->Comment().c_str());
	switch (task->type()) {
		case RTTask::DRIFT:
			msgtask.coords1.x=dynamic_cast<DriftScanTask*>(task)->StartCoords().Phi();
			msgtask.coords1.y=dynamic_cast<DriftScanTask*>(task)->StartCoords().Theta();
			break;
		case RTTask::TRACK:
			msgtask.coords1.x=dynamic_cast<TrackingTask*>(task)->TrackCoords().Phi();
			msgtask.coords1.y=dynamic_cast<TrackingTask*>(task)->TrackCoords().Theta();
			break;
		case RTTask::HORSCAN:
			msgtask.coords1.x=dynamic_cast<HorScanTask*>(task)->StartCoords().Phi();
			msgtask.coords1.y=dynamic_cast<HorScanTask*>(task)->StartCoords().Theta();
			msgtask.coords2.x=dynamic_cast<HorScanTask*>(task)->EndCoords().Phi();
			msgtask.coords2.y=dynamic_cast<HorScanTask*>(task)->EndCoords().Theta();
			msgtask.step1=dynamic_cast<HorScanTask*>(task)->StepAz();
			msgtask.step2=dynamic_cast<HorScanTask*>(task)->StepAlt();
			break;
		case RTTask::EQUSCAN:
			msgtask.coords1.x=dynamic_cast<EquScanTask*>(task)->StartCoords().Phi();
			msgtask.coords1.y=dynamic_cast<EquScanTask*>(task)->StartCoords().Theta();
			msgtask.coords2.x=dynamic_cast<EquScanTask*>(task)->EndCoords().Phi();
			msgtask.coords2.y=dynamic_cast<EquScanTask*>(task)->EndCoords().Theta();
			msgtask.step1=dynamic_cast<EquScanTask*>(task)->StepRa();
			msgtask.step2=dynamic_cast<EquScanTask*>(task)->StepDec();
			break;
		case RTTask::GOTOHOR:
			msgtask.coords1.x=dynamic_cast<GotoHorTask*>(task)->GotoCoords().Phi();
			msgtask.coords1.y=dynamic_cast<GotoHorTask*>(task)->GotoCoords().Theta();
			break;
		case RTTask::GOTOEQU:
			msgtask.coords1.x=dynamic_cast<GotoEquTask*>(task)->GotoCoords().Phi();
			msgtask.coords1.y=dynamic_cast<GotoEquTask*>(task)->GotoCoords().Theta();
			break;
		default:
			break;
	}
	return msgtask;
}


RTTask* fromMsgTask(const task_t& msgtask)
{
	RTTask* task { nullptr };

	switch ((RTTask::TASKTYPE)msgtask.type) {
		case RTTask::DRIFT:
			task=new DriftScanTask(msgtask.id, msgtask.priority,
				Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
				msgtask.int_time, msgtask.ref_cycle, msgtask.alt_period,
				hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y));
			break;
		case RTTask::TRACK:
			task=new TrackingTask(msgtask.id, msgtask.priority,
				Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
				msgtask.int_time, msgtask.ref_cycle, msgtask.alt_period,
				hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y));
			break;
		case RTTask::HORSCAN:
			task=new HorScanTask(msgtask.id, msgtask.priority,
				Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
				msgtask.int_time, msgtask.ref_cycle, msgtask.alt_period,
				hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y),
				hgz::SphereCoords(msgtask.coords2.x, msgtask.coords2.y),
				msgtask.step1, msgtask.step2);
			break;
		case RTTask::EQUSCAN:
			task=new EquScanTask(msgtask.id, msgtask.priority,
				Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
				msgtask.int_time, msgtask.ref_cycle, msgtask.alt_period,
				hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y),
				hgz::SphereCoords(msgtask.coords2.x, msgtask.coords2.y),
				msgtask.step1, msgtask.step2);
			break;
		case RTTask::GOTOHOR:
			task=new GotoHorTask(msgtask.id, msgtask.priority,
				Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
				msgtask.alt_period,
				hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y));
			break;
		case RTTask::GOTOEQU:
			task=new GotoEquTask(msgtask.id, msgtask.priority,
				Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
				msgtask.alt_period,
				hgz::SphereCoords(msgtask.coords1.x, msgtask.coords1.y));
			break;
		case RTTask::MAINTENANCE:
			task=new MaintenanceTask(msgtask.id, msgtask.priority,
				Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
				msgtask.alt_period);
			break;
		case RTTask::PARK:
			task=new ParkTask(msgtask.id, msgtask.priority,
				Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
				msgtask.alt_period);
			break;
		case RTTask::UNPARK:
			task=new UnparkTask(msgtask.id, msgtask.priority,
				Time((long double)msgtask.start_time), Time((long double)msgtask.submit_time),
				msgtask.alt_period);
			break;
		default:
			return nullptr;
			break;
	}
	task->SetMaxRunTime(msgtask.duration);
	task->SetElapsedTime(msgtask.elapsed);
	task->SetComment(msgtask.comment);
	task->SetUser(msgtask.user);
	task->SetState( (RTTask::TASKSTATE)msgtask.status );
	if ( task->State() == RTTask::TASKSTATE::ACTIVE ) task->SetState( RTTask::TASKSTATE::STOPPED );
	return task;
}


// process tasklist
void processTaskList(vector<RTTask*>& tasklist) {
	// remove identical tasks
	for (int first=0; first<(int)tasklist.size()-1; first++) {
		for (int second=first+1; second<tasklist.size(); second++) {
			// do we have the same type?
			if (tasklist[first]->type()!=tasklist[second]->type()) continue;
			// do the start times differ less than 30s?
			if (fabs(tasklist[first]->scheduleTime().timestamp()-tasklist[second]->scheduleTime().timestamp())>30.) continue;
			// do the int times differ by more than 1e-3?
			if (fabs(tasklist[first]->IntTime()-tasklist[second]->IntTime())>1e-3) continue;
			// do the ref intervals differ by more than 5?
			if (abs(tasklist[first]->RefInterval()-tasklist[second]->RefInterval())>5) continue;
			// when we reach here, the tasks are considered identical, so remove the second one
			syslog (LOG_WARNING, "task id %d is identical to id %d. removing the latter", (int)tasklist[first]->ID(), (int)tasklist[second]->ID());
			delete tasklist[second];
			tasklist.erase(tasklist.begin()+second);
		}
	}
	// sort tasklist for start-time in ascending order
	for (int first=0; first<(int)tasklist.size()-1; first++) {
		for (int second=first+1; second<tasklist.size(); second++) {
			if (tasklist[first]->scheduleTime().timestamp()>tasklist[second]->scheduleTime().timestamp()) {
				swap(tasklist[first], tasklist[second]);
			}
		}
	}
	// now process each task by calling the tasks' Process() method
	for (vector<RTTask*>::iterator it=tasklist.begin(); it!=tasklist.end(); ++it) {
		(*it)->Process();
	}
}


void daemonize()
{
        int i;

        if(getppid()==1) return; /* already a daemon */
        i=fork();
        if (i<0) exit(1); /* fork error */
        if (i>0) exit(0); /* parent exits */

        /* child (daemon) continues */
        setsid(); /* obtain a new process group */
        for (i=getdtablesize();i>=0;--i) close(i); /* close all descriptors */
        i=open("/dev/null",O_RDWR); dup(i); dup(i); /* handle standard I/O */

        /* first instance continues */
        signal(SIGCHLD,SIG_IGN); /* ignore child */
        signal(SIGTSTP,SIG_IGN); /* ignore tty signals */
        signal(SIGTTOU,SIG_IGN);
        signal(SIGTTIN,SIG_IGN);
}


// main program
int main(int argc, char *argv[])
{
	int msqid;
	int msgflg = IPC_CREAT | 0666;
	key_t key;
	message_t smsg;
	size_t buf_length;
	bool server=false;
	int fromid=0;
	int verbose=0;
	vector<pair<int,int> > cmdLineActions;
	bool exportTaskList=false;
	long lastTaskID=-1;

	key = MSQ_ID;

/*	
int main() {
    char buf[BUFSIZ];
    fgets(buf, sizeof buf, stdin);
    if (buf[strlen(buf)-1] == '\n') {
        // read full line
    } else {
        // line was truncated
    }
    return 0;
}
*/
	
	
	// command line parsing
	int ch;
	string infile = "";
	string execpath = "";
	string datapath = "/tmp/ratsche";
    char buf[BUFSIZ];
    bool list_tasks { false };
    bool reverse_sort { false };

	while ((ch = getopt(argc, argv, "vlrpdEe:a:s:c:k:x:o:h?")) != EOF) {
		switch ((char)ch) {
			case 'v':
				// increase verbosity level
				verbose++;
				break;
			case 'a':
				// add request only once to the action list
				//printf("optarg: %s", optarg);
				if ( !strcmp(optarg ,"-") ) {
					//printf("reading from stdin\n");
					fgets(buf, sizeof buf, stdin);
					printf("reading: %s", buf);
					cmdLineActions.push_back(make_pair((int)AC_ADD,1));
				} else if (optarg != "" && infile == "") {
					infile=string(optarg);
					cmdLineActions.push_back(make_pair((int)AC_ADD,0));
				}
				break;
			case 'k':
				if (optarg!="") {
					key=atoi(optarg);
				}
				break;
			case 'p':
				list_tasks = true;
				exportTaskList=true;
				break;
			case 'd':
				server=true;
				break;
			case 'l':
                list_tasks = true;
				break;
			case 'r':
				reverse_sort = true;
				break;
			case 'E':
				cmdLineActions.push_back(make_pair((int)AC_CLEAR,0));
				break;
			case 'e':
				cmdLineActions.push_back(make_pair((int)AC_DELETE,atoi(optarg)));
				break;
			case 's':
				cmdLineActions.push_back(make_pair((int)AC_STOP,atoi(optarg)));
				break;
			case 'c':
				cmdLineActions.push_back(make_pair((int)AC_CANCEL,atoi(optarg)));
				break;
			case 'x':
				execpath=optarg;
				break;
			case 'o':
				datapath=optarg;
				break;
			case 'h':
			case '?':  Usage(argv[0]); return 0;
			default: break;
		}
	}

	if (verbose>4) verbose=4;

	if (verbose>3)	{
		cout<<"pid="<<getpid()<<endl;
		printf("Calling msgget with key %#lx and flag %#o\n",key,msgflg);
	}
    
    if (list_tasks) {
        cmdLineActions.push_back(make_pair((int)AC_LIST, static_cast<int>(reverse_sort)));
    }

	
	if ((msqid = msgget(key, msgflg )) < 0) {
		perror("error accessing message queue: msgget failed");
		exit(1);
	}
	else if (verbose>2)	printf("msgget: msgget succeeded: msqid = %d\n", msqid);

	struct msqid_ds msqinfo;
	// see first if the message queue is full
	if (msgctl(msqid, IPC_STAT, &msqinfo)<0) {
		perror("error accessing message queue: msgctl failed");
		exit(1);
	} else {
		// message queue is full and we started as server process, so throw away all messages
		// since they are useless
		if (verbose>3) cout<<" nr. of messages in queue: "<<msqinfo.msg_qnum<<endl;
		if (msqinfo.msg_qnum>200 && server) {
			int fromid, action, subaction;
			while (receive_message(msqid, &fromid, 0, &action, &subaction, NULL) >= 0);
		}
	}

	if (send_message(msqid, getpid(), 1, AC_PING, 0, NULL) < 0) {
		perror("error accessing message queue: send_message failed");
		exit(1);
	}
	else if (verbose>3) printf("sent ping\n");

	// wait 100ms
	usleep(100000);

	int action,subaction;
	if (receive_message(msqid, &fromid, getpid(), &action, &subaction, NULL) < 0) {
		if (errno==ENOMSG){
			if (!server) {
				cerr<<string(argv[0])<<": no connection to server"<<endl;
				exit(1);
			} else {
				if (verbose>3)
					cout<<"no message found: i'm a server"<<endl;
			}
		} else { 
			perror("error accessing message queue: receive_message failed");
			exit(1);
		}
	}
	else {
		if (server) {
			cerr<<string(argv[0])<<": server already running"<<endl;
			exit(2);
		} else {
			if (verbose>3) printf("received pong: i'm client nr. %d\n", getpid());
		}
	}


	// execute the program in server mode from here on
	if (server)
	{
		//daemon(NULL, NULL);
		daemon(0, 0);
		//daemonize();
		vector<RTTask*> tasklist;
		try
		{
			int facility_priority = LOG_NOTICE; // default log priority is LOG_NOTICE
			if (verbose==1) facility_priority = LOG_INFO; // show also infos in syslog if verbose setting enabled
			if (verbose>1) facility_priority = LOG_DEBUG; // show all in syslog if verbosity increased

			setlogmask (LOG_UPTO (facility_priority));

			//openlog (NULL, LOG_CONS | LOG_PERROR | LOG_PID, LOG_DAEMON);
			openlog (NULL, LOG_PID, LOG_DAEMON);
			syslog (LOG_NOTICE, "starting RT task scheduler server");
			syslog (LOG_NOTICE, "using message queue 0x%04x",key);

			if (!execpath.empty()) {
				RTTask::SetExecutablePath(execpath);
				syslog (LOG_NOTICE, "using executable path %s",execpath.c_str());
			}
			if (!datapath.empty()) {
				RTTask::SetDataPath(datapath);
				syslog (LOG_NOTICE, "using data path %s",datapath.c_str());
			}
			// clear queue
			int nrOldMsg=0;
			while (receive_message(msqid, &fromid, 0, &action, &subaction, NULL) >= 0) {nrOldMsg++;}
			if (nrOldMsg) syslog (LOG_WARNING, "found %d zombie message(s) in queue...deleting",nrOldMsg);

			// try to load tasklist from previous session
			{
				// add task(s)
				vector<task_t> msgTaskVector;
//				if (importTasklistFromFile(defaultTaskFile, msgTaskVector)!=0) {
				if ( !load_tasks(defaultTaskFile, msgTaskVector) ) {
					error(argv[0], "reading task file");
				} else {
					// submit tasklist
					// loop over tasks
					syslog (LOG_NOTICE, "loading tasklist from previous session, adding %d tasks", msgTaskVector.size());
					for ( task_t task : msgTaskVector ) {
						task.id=++lastTaskID;
						syslog (LOG_INFO, "received ADD request, adding new task (id=%d) to list", task.id);
						RTTask* taskptr { fromMsgTask( task ) };
						if ( taskptr != nullptr ) tasklist.push_back( taskptr );
					}
				}
			}
			// stay in endless loop
			while (true) {
				// see if there is a message in the queue
				task_t task { };
				if (receive_message(msqid, &fromid, 1, &action, &subaction, &task) >= 0) {
					RTTask* taskptr { nullptr };
					switch (action) {
						case AC_PING:
							// received ping, send back echo
							//syslog (LOG_DEBUG, "received PING message, sending back echo");
							if (send_message(msqid, 1, fromid, AC_PING, 0, NULL) < 0) {
								syslog (LOG_CRIT, "unable to send message to message queue");
								perror("send_message");
							}
							else {
								//printf("pingpong\n");
							}
							break;
						case AC_LIST:
							// List all tasks
							//syslog (LOG_DEBUG, "received LIST request, sending back list of %d task(s)", tasklist.size());
							if (!tasklist.size()) {
								// send empty list
								if (send_message(msqid, 1, fromid, AC_LIST, subaction, NULL, 1, 0) < 0) {
									syslog (LOG_CRIT, "unable to send message to message queue");
									perror("send_message");
								}
							} else
                            for (size_t i=0; i<tasklist.size(); i++) {
								const size_t curr_task { (subaction==0)?i:tasklist.size()-i-1 };
                                task_t _task=toMsgTask(tasklist[curr_task]);
								if (send_message(msqid, 1, fromid, AC_LIST, subaction, &_task, i+1, tasklist.size()) < 0) {
									syslog (LOG_CRIT, "unable to send message to message queue");
									perror("send_message");
								}
							}

							break;
						case AC_ADD:
							// add task
							task.id=++lastTaskID;
							syslog (LOG_DEBUG, "received ADD request, adding new task (id=%d) to list", task.id);
							taskptr=fromMsgTask(task);
							if (taskptr!=NULL) tasklist.push_back(taskptr);
							break;
						case AC_DELETE:
							// delete task
							syslog (LOG_DEBUG, "received DELETE request, deleting task (id=%d) from list", subaction);
							for (vector<RTTask*>::iterator it=tasklist.begin(); it!=tasklist.end(); ++it) {
								if ( (*it != NULL) && ((*it)->ID() == subaction) ) {
									if ( (*it)->State() == RTTask::ACTIVE ) {
										(*it)->Cancel();
									}
									delete *it;
									tasklist.erase(it);
									syslog (LOG_DEBUG," deleted task id=%d, new size=%d", subaction, tasklist.size());
									break;
								}
							}
							//syslog (LOG_WARNING, "trying to delete task id=%d, which does not exist", subaction);
							break;
						case AC_STOP:
							// stop task
							syslog (LOG_DEBUG, "received STOP request, stopping task (id=%d)", subaction);
							for (vector<RTTask*>::iterator it=tasklist.begin(); it!=tasklist.end(); ++it) {
								if ( (*it != NULL) && ((*it)->ID() == subaction) ) {
									(*it)->Stop();
									syslog (LOG_DEBUG," stopped task id=%d", subaction);
									break;
								}
							}
							break;
						case AC_CANCEL:
							// cancel task
							syslog (LOG_DEBUG, "received CANCEL request, cancelling task (id=%d)", subaction);
							for (vector<RTTask*>::iterator it=tasklist.begin(); it!=tasklist.end(); ++it) {
								if ( (*it != NULL) && ((*it)->ID() == subaction) ) {
									(*it)->Cancel();
									syslog (LOG_DEBUG," cancelled task id=%d", subaction);
									break;
								}
							}
							break;
						case AC_CLEAR:
							// delete all tasks
							syslog (LOG_INFO, "received CLEAR request, deleting all tasks");
							while ( !tasklist.empty() ) {
								auto it = tasklist.begin();
								long int id = (*it)->ID();
								if ( (*it)->State() == RTTask::ACTIVE ) {
									(*it)->Cancel();
								}
								delete *it;
								tasklist.erase(it);
								syslog (LOG_DEBUG," deleted task id=%d, new size=%d", id, tasklist.size());
							}
							break;
						default: break;
					}
					// process all tasks
					processTaskList(tasklist);
					// the tasklist has been modified, so back it up to file
					vector<task_t> msgTaskList;
					for (auto task : tasklist) {
						msgTaskList.push_back( toMsgTask(task) );
					}
					//ofstream ostr(defaultTaskFile.c_str());
					//export_tasks(ostr, _tasklist);
					save_tasks( defaultTaskFile, msgTaskList );
				} else {
					// process all tasks
					processTaskList(tasklist);
					// sleep for 10ms
					usleep( server_loop_delay_us );
				}
			}
		} // if (server)
		catch (...) {
			// if there's any uncaught exception, clear the task list cleanly,
			// write to the system log and exit
			syslog (LOG_CRIT, "caught unhandled exception");
			syslog (LOG_CRIT, "stopping server.");
			// unqueue task list
			while (tasklist.size()) {
				if (tasklist[0]!=NULL) delete tasklist[0];
				tasklist.erase(tasklist.begin());
			}
			exit(3);
		}
		// Todo: evaluate system signals (SIGTERM, SIGKILL etc.)
		// and unmount the task list cleanly
	}


//*******************************************************//

	// run as client

	task_t task;
	task.id=1234;
	task.type=1;
	task.priority=5;
	task.start_time=time(NULL);
	task.submit_time=time(NULL);
	task.type=1;
	(void) strcpy(task.user, "ratsche");
	(void) strcpy(task.comment, "dummy task");

	//print_task(task);

	// evaluate command line options
	for ( const auto& [ act, subact ] : cmdLineActions )
	{
		// list tasks
		if ( act == AC_LIST ) {
			if (send_message(msqid, getpid(), 1, AC_LIST, subact, NULL) < 0) {
				perror("send_message in requesting task list failed");
				exit(1);
			}
			else if (verbose>2) printf("sent LIST\n");

			usleep(20000);

			unsigned long int ctr=0;
			int msgcount=-1;
			vector<task_t> tasklist;
			while ( ctr < 100 && msgcount != 0) {
				int serid,sercnt,fromid;
				if (receive_message(msqid, &fromid, getpid(), &action, &subaction, &task, &serid, &sercnt) >= 0) {
					if (sercnt==0) { msgcount=0; break; }
					msgcount=sercnt-serid;
					tasklist.push_back(task);
					if (verbose>3) cout<<"rx task entry: serid="<<serid<<" , sercnt="<<sercnt<<endl;
				} else {
					// sleep 25ms
					usleep(25000);
					ctr++;
				}
			}
			if (msgcount<0) error(argv[0],"timeout receiving LIST");
			else {
				if (verbose>2) cout<<"received "<<tasklist.size()<<" entries. (rx-dur="<<ctr*10<<"ms)"<<endl;
				if (exportTaskList) {
					export_tasks(cout, tasklist);
					exportTaskList=false;
				} else {
					print_tasklist(tasklist);
				}
			}
		} else if ( act == AC_DELETE ) {
			// delete task
			if (send_message(msqid, getpid(), 1, AC_DELETE, subact, NULL) < 0) {
				perror("send_message in deleting a task failed");
				exit(1);
			}
			else if (verbose>2) printf("sent DELETE\n");
		} else if ( act == AC_CLEAR ) {
				// delete all tasks
				if (send_message(msqid, getpid(), 1, AC_CLEAR, 0, NULL) < 0) {
					perror("send_message in clearing task list failed");
					exit(1);
				}
				else if (verbose>2) printf("sent CLEAR\n");
		} else if ( act == AC_STOP ) {
			// stop task
			if (send_message(msqid, getpid(), 1, AC_STOP, subact, NULL) < 0) {
				perror("send_message in stopping a task failed");
				exit(1);
			}
			else if (verbose>2) printf("sent STOP\n");
		} else if ( act == AC_CANCEL ) {
			// cancel task
			if (send_message(msqid, getpid(), 1, AC_CANCEL, subact, NULL) < 0) {
				perror("send_message in cancelling a task failed");
				exit(1);
			}
			else if (verbose>2) printf("sent CANCEL\n");
		} else if ( act == AC_ADD ) {
			// add task(s)
			task_t task;
			vector<task_t> tasklist;
			switch ( subact ) {
				case 0:
					if (importTasklistFromFile(infile, tasklist) !=0 ) {
						error(argv[0], "reading task file");
					}
					break;
				case 1:
					if ( parseStringForTask(string(buf), task) == 0 ) {
						tasklist.push_back(task);
					}
					break;
				default:
					break;
			};
			// submit tasklist
			// loop over tasks
			for (int i=0; i<tasklist.size(); i++) {
				if (send_message(msqid, getpid(), 1, AC_ADD, 0, &tasklist[i]) < 0) {
					perror("send_message in adding a task failed");
					exit(1);
				}
				else if (verbose>2) printf("added Task\n");
			}
		}
	}

	return 0;
}

