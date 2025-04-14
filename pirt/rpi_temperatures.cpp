#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <string>
#include <unistd.h>

#include "rpi_temperatures.h"

#define DEFAULT_VERBOSITY 1

namespace PiRaTe {

constexpr std::chrono::milliseconds loop_delay { 5000 };
const std::vector<std::string> tempFileNameCandidates { "temp", "temperature" };
constexpr std::size_t MAX_DEVICES { 64 };

RpiTemperatureMonitor::RpiTemperatureMonitor(const std::string device_path)
    : fDevPath { std::move(device_path) }
{
    //rescanSources();
    fActiveLoop = true;
    // since C++14 using std::make_unique
    // fThread = std::make_unique<std::thread>( [this]() { this->readLoop(); } );
    // C++11 is unfortunately more unconvenient with move from a locally generated pointer
    std::unique_ptr<std::thread> thread(new std::thread([this]() { this->threadLoop(); }));
    fThread = std::move(thread);
}

RpiTemperatureMonitor::~RpiTemperatureMonitor()
{
    if (!fActiveLoop)
        return;
    fActiveLoop = false;
    if (fThread != nullptr)
        fThread->join();
}

void RpiTemperatureMonitor::rescanSources()
{
    // assume, that all available thermal sensors under /sys/class/thermal/thermal_zone0/hwmon0/subsystem/
    // are actually named hwmon0, hwmon1...hwmonN, with N being the last available sensor
    const std::string path { fDevPath + "/hwmon" };
    fItemList.clear();
    for (unsigned int hwMonIndex = 0; hwMonIndex < MAX_DEVICES; hwMonIndex++) {
        std::ifstream namefile(path + std::to_string(hwMonIndex) + "/name");
        if (!namefile.is_open())
            break;
        std::string name {};
        namefile >> name;
        bool isTemp { false };
        double temperature {};
        std::string tempFileName {};
        for (auto tempFileNameCandidate : tempFileNameCandidates) {
            tempFileName = path + std::to_string(hwMonIndex) + "/device/" + tempFileNameCandidate;
            std::ifstream tempfile(tempFileName);
            if (!tempfile.is_open())
                continue;
            tempfile >> temperature;
            isTemp = true;
            break;
        }
        if (!isTemp)
            continue;
        std::ifstream idfile(path + std::to_string(hwMonIndex) + "/device/name");
        TemperatureItem item {};
        item.name = name;
        item.temperature = temperature / 1000.;
        item.valid = true;
        item.hwMonIndex = hwMonIndex;
        item.value_device_path = tempFileName;
        if (!idfile.is_open()) {
            item.id = std::to_string(hwMonIndex);
        } else {
            idfile >> item.id;
        }
        fItemList.push_back(std::move(item));
    }
}

auto RpiTemperatureMonitor::getTemperatureItem(std::size_t source_index) -> TemperatureItem
{
    if (source_index >= fItemList.size())
        return TemperatureItem();
    return fItemList.at(source_index);
}

// this is the background thread loop
void RpiTemperatureMonitor::threadLoop()
{
    rescanSources();
    auto lastReadOutTime = std::chrono::system_clock::now();
    //bool errorFlag = true;
    while (fActiveLoop) {
        auto currentTime = std::chrono::system_clock::now();

        //		for ( auto &item: fItemList ) {
        for (unsigned int sourceIndex = 0; sourceIndex < fItemList.size(); sourceIndex++) {
            const std::lock_guard<std::mutex> lock(fMutex);
            auto item = fItemList.at(sourceIndex);
            std::ifstream tempfile(item.value_device_path);
            if (!tempfile.is_open()) {
                item.valid = false;
                continue;
            }
            double temperature {};
            tempfile >> temperature;
            item.temperature = temperature / 1000.;
            item.valid = true;
            item.sourceIndex = sourceIndex;
            if (fTempReadyFn)
                fTempReadyFn(item);
            //fUpdated = true;
        }

        std::this_thread::sleep_for(loop_delay);
    }
}

} // namespace PiRaTe
