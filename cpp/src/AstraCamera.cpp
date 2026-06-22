#include "AstraCamera.hpp"

const int FRAME_TIMEOUT_MS = -1;//forever


AstraCamera::AstraCamera()
{
    astra_status_t status = astra::initialize();

    if (status != ASTRA_STATUS_SUCCESS)

    {
    std::cerr << "ASTRA INIT FAILED: " << status << std::endl;
    return;
    }

    else std::cerr << "ASTRA INIT SUCCEDED: " << status << std::endl;




    streamSet_ = std::make_unique<astra::StreamSet>();

    std::cerr << "streamset created\n";


    reader_ = std::make_unique<astra::StreamReader>(
        streamSet_->create_reader()
    );

    std::cerr << "reader created\n";


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
    std::cout << "astra starting\n";

    if (running_)
        return true;

    try
    {
        astra::DepthStream depth =
            reader_->stream<astra::DepthStream>();

        astra::ColorStream color =
            reader_->stream<astra::ColorStream>();

        depth.start();
        color.start();

        bool camera_ok = false;

        for (int i = 0; i < 30; i++)
        {
            astra_update();

            astra::Frame frame =
                reader_->get_latest_frame(100);

            astra::DepthFrame d = frame.get<astra::DepthFrame>();
            astra::ColorFrame c = frame.get<astra::ColorFrame>();

            if (d.is_valid() && c.is_valid())
            {
                camera_ok = true;
                break;
            }
        }

        if (!camera_ok)
        {
            std::cerr << "ASTRA START FAILED: no valid frames received\n";
            return false;
        }

        std::cout << "ASTRA CAMERA READY\n";

        camera_status.connected = true;
        camera_status.streaming = true;

        running_ = true;

        worker_ = std::thread(&AstraCamera::run, this);

        std::cout << "astra started\n";
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "ASTRA START EXCEPTION: " << e.what() << std::endl;
        return false;
    }
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
    while (running_)
    {
        astra_update();   // 🔥 REQUIRED

        astra::Frame frame = reader_->get_latest_frame(FRAME_TIMEOUT_MS);

        astra::DepthFrame depth = frame.get<astra::DepthFrame>();
        astra::ColorFrame color = frame.get<astra::ColorFrame>();

        if (!depth.is_valid() || !color.is_valid())
        {
            camera_status.streaming = false;
            continue;
        }

        camera_status.streaming = true;
        camera_status.connected = true;


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

    std::cout << "-----getting camera frame--------------";

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
