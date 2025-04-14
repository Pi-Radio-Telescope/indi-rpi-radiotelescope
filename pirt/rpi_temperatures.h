#ifndef RPITEMP_H
#define RPITEMP_H

#include <cmath>
#include <functional>
#include <inttypes.h> // uint8_t, etc
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace PiRaTe {

class RpiTemperatureMonitor {
public:
    struct TemperatureItem {
        std::string name { "N/A" };
        std::string id { "0" };
        unsigned int hwMonIndex {};
        unsigned int sourceIndex {};
        std::string value_device_path { "" };
        double temperature {};
        bool valid { false };
    };

    RpiTemperatureMonitor(const std::string device_path = "/sys/class/thermal/thermal_zone0/hwmon0/subsystem/");
    ~RpiTemperatureMonitor();

    void rescanSources();
    [[nodiscard]] auto nrSources() const -> std::size_t { return fItemList.size(); }
    [[nodiscard]] auto getTemperatureItem(std::size_t source_index) -> TemperatureItem;

    void registerTempReadyCallback(std::function<void(TemperatureItem)> fn) { fTempReadyFn = fn; }

private:
    void threadLoop();

    std::string fDevPath { "" };
    std::vector<TemperatureItem> fItemList {};
    std::unique_ptr<std::thread> fThread { nullptr };
    bool fActiveLoop { false };
    std::mutex fMutex;

    std::function<void(TemperatureItem)> fTempReadyFn {};
};

} // namespace PiRaTe

#endif // #define RPITEMP_H
