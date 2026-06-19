#include "AstraCamera.hpp"

const int FRAME_TIMEOUT_MS = -1;//forever


AstraCamera::AstraCamera()
:
reader_(streamSet_.create_reader())
{
    camera_status.connected = false;
    camera_status.streaming = false;
    camera_status.framesCaptured = 0;
    camera_status.fps = 0;
    camera_status.lastFrameTimestamp = 0;
}

AstraCamera::~AstraCamera()
{
    stop();
}



bool AstraCamera::start()
{
    if (running_)
        return true;

    astra::initialize();

    astra::DepthStream depth =
        reader_.stream<astra::DepthStream>();

    astra::ColorStream color =
        reader_.stream<astra::ColorStream>();

    depth.start();
    color.start();

    running_ = true;

    worker_ = std::thread(&AstraCamera::run,this);

    return true;
}

void AstraCamera::stop()
{
    if(!running_)
        return;


    running_ = false;


    if(worker_.joinable())
        worker_.join();


    astra::terminate();

    {

        std::lock_guard lock(statusMutex_);
        camera_status.connected = false;
        camera_status.streaming = false;

    }
}

void AstraCamera::run()
{
    while(running_)
    {
        astra::Frame frame = reader_.get_latest_frame(FRAME_TIMEOUT_MS);

        astra::DepthFrame depth = frame.get<astra::DepthFrame>(); 
        astra::ColorFrame color = frame.get<astra::ColorFrame>();


        if(!depth.is_valid() || !color.is_valid())
        {
            camera_status.streaming = false;   
            continue;
        }

        {


            camera_status.streaming = true; 
            camera_status.connected = true; 
        
        }

        backBuffer_.timestamp =
        std::chrono::duration_cast<
        std::chrono::milliseconds>
        (
        std::chrono::steady_clock::now()
        .time_since_epoch()
        )
        .count();

        backBuffer_.colorFrameIndex = color.frame_index();
        backBuffer_.depthFrameIndex = depth.frame_index();

        backBuffer_.colorWidth = color.width();
        backBuffer_.colorHeight = color.height();

        backBuffer_.depthWidth = depth.width();
        backBuffer_.depthHeight = depth.height();

        // fill back buffer
        backBuffer_.depth.resize(depth.length());

        backBuffer_.rgb.resize(color.length()*3);


        depth.copy_to(
            backBuffer_.depth.data());


        color.copy_to(
            reinterpret_cast<astra::RgbPixel*>(
                backBuffer_.rgb.data()));



        //CRITICAL SECTION
        {
            std::lock_guard lock(bufferMutex_);

            std::swap(
                frontBuffer_,
                backBuffer_);
        }
    }
}

astra_result AstraCamera::getFrame()
{

    astra_result result{};
    result.astra_health = camera_status.connected && camera_status.streaming;

    uint64_t time_now =
    std::chrono::duration_cast<
        std::chrono::milliseconds>
    (
        std::chrono::steady_clock::now()
        .time_since_epoch()
    )
    .count();

    {
    std::lock_guard lock(bufferMutex_);
    result.snapshot =  frontBuffer_;
    }

    {
        std::lock_guard lock(bufferMutex_);
        result.operation_status &= (time_now - result.snapshot.timestamp) < 1000;

    }

    return result;
}
