#pragma once

//#include "MotomanRobot.hpp"
#include <stdexcept>
#include "YMConnect.h"
#include "status.hpp"


inline status_code check_status(const StatusInfo& st)
{
    if (!st.IsOk())
        return handle_error(st);

    return status_code::STATUS_OK;
}


status_code handle_error(const StatusInfo& st)
{

    

    return status_code::STATUS_ERROR;
}


 
