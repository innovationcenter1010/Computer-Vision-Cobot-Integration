#pragma once

#include "YMConnect.h"
#include <thread>
#include <chrono>

enum status_code
{
    STATUS_OK = 0,
    STATUS_ERROR = 1
};

using namespace YMConnect;



inline status_code check_status(const StatusInfo& st);
