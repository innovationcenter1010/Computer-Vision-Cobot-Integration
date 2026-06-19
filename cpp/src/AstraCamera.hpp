#pragma once

#include <astra/astra.hpp>
#include <astra_core/astra_core.hpp>
#include <astra/capi/astra.h>
#include <astra_core/capi/astra_core.h>


#include <atomic>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

struct astra_status

{

    bool connected = true;
    bool streaming = true;

    uint64_t framesCaptured;

    float fps;

    uint64_t lastFrameTimestamp;

};

//single frame data including rgb and depth maps
struct RGBDFrame
{
    uint64_t timestamp = 0;

    uint32_t colorFrameIndex = 0;
    uint32_t depthFrameIndex = 0;

    int colorWidth = 0;
    int colorHeight = 0;

    int depthWidth = 0;
    int depthHeight = 0;

    std::vector<uint8_t> rgb;
    std::vector<int16_t> depth;
};


struct astra_result{

    bool operation_status;
    bool astra_health; 
    //mstd::string message;
    //uint8_t error_code;  
    RGBDFrame snapshot; 


};

class AstraCamera
{
public:

    AstraCamera();
    ~AstraCamera();


    bool start();
    void stop();


    astra_result getFrame();


private:


    void run();



private:

    RGBDFrame frontBuffer_;
    RGBDFrame backBuffer_;


    std::mutex bufferMutex_;
    std::mutex statusMutex_;


    std::atomic<bool> running_{false};


    std::thread worker_;
    astra_status camera_status{};

    astra::StreamSet streamSet_;
    astra::StreamReader reader_;

};