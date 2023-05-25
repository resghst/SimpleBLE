#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "../common/utils.hpp"

#include "simpleble/Adapter.h"
#include "simpleble/SimpleBLE.h"

// #define DEVICE_NAME "RT584"
#define DEVICE_NAME "CITIZEN BPM CH681"
#define SCAN_DELAY 2
#define CONN_DELAY 2

#define BLOOD_PRESSURE "00001810-0000-1000-8000-00805f9b34fb"
#define BLOOD_PRESSURE_MEASUREMENT "00002a35-0000-1000-8000-00805f9b34fb"

SimpleBLE::Safe::Adapter* adapter;

#define BPMUnitsFlag (0x1 << 0)
#define BPMTimeFlag  (0x1 << 1)
#define BPMPulseRateFlag (0x1 << 2)
#define BPMUIDFlag (0x1 << 3)
#define BPMMSFlag (0x1 << 4)

struct __attribute__((__packed__)) BPMFlag{
    uint8_t flag;
};

struct __attribute__((__packed__)) BPMValue{
    uint16_t Systolic; // Systolic
    uint16_t Diastolic; // Diastolic
    uint16_t MAP; // Mean Arterial Pressure
};

struct __attribute__((__packed__)) BPMTime{
    uint16_t year; // year
    uint8_t month; // month
    uint8_t day; // day
    uint8_t hour; // hour
    uint8_t min; // min
    uint8_t sec; // sec
    uint8_t weekday;
};

struct __attribute__((__packed__)) BPMPulseRate{
    uint16_t pulse_rate; // pulse_rate
};

struct __attribute__((__packed__)) BPMUID{
    uint8_t UID; // User ID
};

struct __attribute__((__packed__)) BPMState{
    bool BMD; // Body Movement Detection
    bool CFD; // Cuff Fit Detection
    bool IPD; // Irregular Pulse Detection
    bool PRRD_1; // Pulse Rate Range Detection
    bool PRRD_2; // Pulse Rate Range Detection
    bool MPD; // Measurement Position Detection
} ;

void pass_BP_data(SimpleBLE::ByteArray& bytes) {
    struct BPMFlag flag;
    struct BPMValue val;
    struct BPMTime time;
    struct BPMPulseRate pr;
    struct BPMUID uid;
    struct BPMState state;
    std::string unit;
    
    uint8_t *cstr = new uint8_t[bytes.length() + 1];
    memcpy(&cstr[0], bytes.c_str(), bytes.length());
    for(int i =0; i<bytes.length(); i++)
        std::cout << std::hex  << (uint32_t)((uint8_t)cstr[i]) ;
    
    std::cout << std::endl
                << "===== Parising Blood Pressure Data =====" << std::endl;

    memcpy(&flag, (uint8_t *)&cstr[0], sizeof(BPMFlag));
    int idx = sizeof(BPMFlag);
    std::cout << std::hex << (uint32_t)((uint8_t)flag.flag) << std::endl;
    if (flag.flag & BPMUnitsFlag)
        unit = "kPa";
    else
        unit = "mmHg";

    memcpy(&val, (uint8_t *)&cstr[idx], sizeof(BPMValue));
    idx += sizeof(BPMValue);
    printf("Systolic: %u %s\n", val.Systolic, unit.c_str());
    printf("Diastolic: %u %s\n", val.Diastolic, unit.c_str());
    printf("Mean Arterial Pressure: %u %s\n", val.MAP, unit.c_str());

    if (flag.flag & BPMTimeFlag) {
        // time = &bytes[idx];
        memcpy(&time, (uint8_t *)&cstr[idx], sizeof(BPMTime));
        idx += sizeof(BPMTime);
        printf("Time:  %u/%u/%u %u:%u:%u \n", time.year, time.month, time.day,
            time.hour, time.min, time.sec);
    }
    if (flag.flag & BPMPulseRateFlag) {
        // pr = &bytes[idx];
        memcpy(&pr, (uint8_t *)&cstr[idx], sizeof(BPMPulseRate));
        idx += sizeof(BPMPulseRate);
        printf("Pulse Rate: %u\n", pr.pulse_rate);
    }
    if (flag.flag & BPMUIDFlag) {
        // uid = &bytes[idx];
        memcpy(&uid, &cstr[idx], sizeof(BPMUID));
        idx += sizeof(BPMUID);
        printf("User ID: %u\n", uid.UID);
    }
    if (flag.flag & BPMMSFlag) {
        // state = &bytes[idx];
        memcpy(&state, (uint8_t *)&cstr[idx], sizeof(BPMState));
        std::cout << "Measurement State" << std::endl;
        std::cout << "Body Movement Detection: : " << state.BMD << std::endl;
        std::cout << "Cuff Fit Detection: " << state.CFD << std::endl;
        std::cout << "Irregular Pulse Detection: " << state.IPD << std::endl;
        std::cout << "Pulse Rate Range Detection: " << state.PRRD_1 << state.PRRD_2
                << std::endl;
        std::cout << "Measurement Position Detection: " << state.MPD << std::endl;
    }

    std::cout << "============= Parising End =============" << std::endl;
    // memcpy(student.name, bytes, strlen(BPM)+1);
    // for (auto b : bytes) {
    //     std::cout << std::hex << std::setfill('0') << std::setw(2) <<
    //     (uint32_t)((uint8_t)b) << " ";
    // }
    // std::cout << std::endl;
}

bool add_deviced=0;
bool no_conn_dev =true;
void add_device(){
    std::cout<< "add_device"<< std::endl;
    std::vector<SimpleBLE::Safe::Peripheral> peripherals;
    adapter->set_callback_on_scan_found(
        [&](SimpleBLE::Safe::Peripheral peripheral) {peripherals.push_back(peripheral);});
    // adapter->set_callback_on_scan_start([](){std::cout << "Scan started." << std::endl;});
    // adapter->set_callback_on_scan_stop([](){std::cout << "Scan stopped." << std::endl;});
    add_deviced=0;
    no_conn_dev =true;
    while (!add_deviced)
    {
        if(no_conn_dev){
            adapter->scan_for(5000); // Scan for 5 seconds and return.
            sleep(5);
            for (size_t i = 0; i < peripherals.size(); i++) {
                if (strcmp(&(peripherals[i].identifier().value_or("UNKNOWN"))[0], DEVICE_NAME) == 0){
                    std::cout << "Found: " << DEVICE_NAME << " [" << peripherals[i].address().value_or("UNKNOWN") << "]" << std::endl;
                    std::cout << "Connecting to " << peripherals[i].identifier().value_or("UNKNOWN") << std::endl;
                    peripherals[i].set_callback_on_connected([](){
                        no_conn_dev=false;
                        std::cout << "connected" << std::endl;});
                    peripherals[i].set_callback_on_disconnected([](){
                        add_deviced=true;
                        std::cout << "disconnected" << std::endl;});
                    bool connect_was_successful = peripherals[i].connect();
                    if (!connect_was_successful) {
                        std::cout << "Failed to connect to " << peripherals[i].identifier().value_or("UNKNOWN") << " ["
                                << peripherals[i].address().value_or("UNKNOWN") << "]" << std::endl;
                        std::cout << "Back to scan device " << std::endl;
                        break;
                    }
                    std::cout << "Successfully connected, listing services." << std::endl;
                    auto services = peripherals[i].services();
                    if (!services.has_value()) {
                        std::cout << "Failed to list services." << std::endl;
                        exit(1);
                    }

                    for (auto service : *services) {
                        std::cout << "Service: " << service.uuid() << std::endl;
                        for (auto characteristic : service.characteristics()) {
                            std::cout << "  Characteristic: " << characteristic.uuid() << std::endl;
                            for (auto& descriptor : characteristic.descriptors()) {
                                std::cout << "  Descriptor: " << descriptor.uuid() << std::endl;
                            }
                        }
                    }
                    auto paired = peripherals[i].is_paired();
                    std::cout << "Waiting device pairing. (waiting for more than 30 seconds, please clean bluez cache.)"<<std::endl;
                    int count = 0;
                    while (!paired.value())
                    {
                        if(count!=0 && count%5==0){std::cout << "current waiting time " << count << " s" << std::endl;}
                        sleep(1);
                        count++;
                        paired = peripherals[i].is_paired();
                    }
                    peripherals[i].disconnect();
                    peripherals.clear();
                    return ;
                }
            }
        }
        sleep(SCAN_DELAY);
    }
}


void fetch_BP(){
    std::cout<< "start fetch BP"<< std::endl;
    std::vector<SimpleBLE::Safe::Peripheral> peripherals;
    adapter->set_callback_on_scan_found([&](SimpleBLE::Safe::Peripheral peripheral) {peripherals.push_back(peripheral);});

    while (1)
    {
        std::cout << "Start scan BP device (Scan 5 seconds)" << std::endl;
        adapter->scan_for(5000); // Scan for 5 seconds and return.
        sleep(5);
        for (size_t i = 0; i < peripherals.size(); i++) {
            if (strcmp(&(peripherals[i].identifier().value_or("UNKNOWN"))[0], DEVICE_NAME) == 0){
                std::cout << "Found: " << DEVICE_NAME << " [" << peripherals[i].address().value_or("UNKNOWN") << "]" << std::endl;
                std::cout << "Connecting to " << peripherals[i].identifier().value_or("UNKNOWN") << std::endl;
                std::string conn_data, dis_conn_data;
                peripherals[i].set_callback_on_connected([](){
                    std::cout << "connected" << std::endl;});
                peripherals[i].set_callback_on_disconnected([](){
                    std::cout << "disconnected" << std::endl;});
                bool connect_was_successful = peripherals[i].connect();
                if (!connect_was_successful) {
                    std::cout << "Failed to connect to " << peripherals[i].identifier().value_or("UNKNOWN") << " ["
                            << peripherals[i].address().value_or("UNKNOWN") << "]" << std::endl;
                    std::cout << "Back to scan device " << std::endl;
                    break;
                }
                std::cout << "Successfully connected, listing services." << std::endl;
                bool indicated = true;
                auto services = peripherals[i].services();
                if (!services.has_value()) {
                    std::cout << "Failed to list services." << std::endl;
                    exit(1);
                }

                // Store all service and characteristic uuids in a vector.
                std::vector<std::pair<SimpleBLE::BluetoothUUID, SimpleBLE::BluetoothUUID>> uuids;
                for (auto service : *services) {
                    for (auto characteristic : service.characteristics()) {
                        uuids.push_back(std::make_pair(service.uuid(), characteristic.uuid()));
                    }
                }

                // std::cout << "The following services and characteristics were found:" << std::endl;
                for (size_t i = 0; i < uuids.size(); i++) {
                    // std::cout << "[" << i << "] " << uuids[i].first << " " << uuids[i].second << std::endl;
                    if ((strcmp(&uuids[i].first[0], BLOOD_PRESSURE) == 0) && 
                        (strcmp(&uuids[i].second[0], BLOOD_PRESSURE_MEASUREMENT) == 0)){
                        peripherals[i].notify(uuids[i].first, uuids[i].second,
                            [&, i](SimpleBLE::ByteArray bytes) {
                                std::cout << "Peripheral " << i << " received: ";
                                // Utils::print_byte_array(bytes);
                                pass_BP_data(bytes);
                                indicated = true;
                        });
                    }
                }
                auto connected = peripherals[i].is_connected();
                std::cout << "Waiting indication"<<std::endl;
                int count = 0;
                while (connected.value()) { 
                    sleep(1);
                    // std::cout << "connected " << connected.value() <<std::endl;
                    connected = peripherals[i].is_connected();
                }
                peripherals[i].disconnect();
                peripherals.clear();
                break;
            }
            peripherals.clear();
            std::cout << "Back to scan BP device" << std::endl;
        }
        sleep(SCAN_DELAY);
    }
    return ;
}

void trigger_bluetooth(int adapter_idx, bool add_device_flag){
    auto adapter_list = SimpleBLE::Safe::Adapter::get_adapters();
    // if (!adapter_list.has_value()) {
    //     std::cout << "Failed to list adapters" << std::endl;
    //     exit(1);
    // }
    // else if (adapter_list->empty()) {
    //     std::cout << "No adapter was found." << std::endl;
    //     exit(1);
    // }

    std::cout << "Available adapters: " << std::endl;
    int i = 0, tar = -1;
    for (auto& adapter : *adapter_list) {
        std::cout << "[" << i << "] " << adapter.identifier().value() << " [" << adapter.address().value() << "]"
                  << std::endl;
        if(adapter_idx==atoi((adapter.identifier().value()).c_str() )){ tar = i; }
        i++;        
    }
    std::cout << "Select hci" << adapter_idx << std::endl;

    // auto adapter_selection = Utils::getUserInputInt("Please select an adapter", adapter_list->size() - 1);
    // if (!adapter_selection.has_value()) { exit(1); }

    adapter = &adapter_list->at(tar);
    if(add_device_flag){
        auto paired_list = adapter->get_paired_peripherals();
        for(auto paired : *paired_list){
            if (strcmp(&(paired.identifier().value_or("UNKNOWN"))[0], DEVICE_NAME) == 0){
                std::cout << "Found unpair device: " << &(paired.identifier().value_or("UNKNOWN"))[0] << std::endl << std::endl;
                paired.unpair();
            }
        }
        add_device();
    }
    fetch_BP();
    return ;
}

int main(int argc, char *argv[]) {
    if(argc!=3){
        std::cout << "Invailed Arg\n";
        exit(1);
    }
    int adapter_idx = atoi(argv[1]);
    bool add_device_flag = atoi(argv[2]);
    trigger_bluetooth(adapter_idx, add_device_flag);
    
}
