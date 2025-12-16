#pragma once

#include <stdio.h>
#include "LSM6DSV16XSensor.h"

void logger_task(void *q);

struct LoggerMsg
{
    bool delayed;      // Whether the vTaskDelayUntil was delayed.
    int32_t read_time; // usec time at end of collection
    bool imu;          // Which IMU was collected.
    int sample_count;
    lsm6dsv16x_fifo_record_t records[32]; // Up to 16 samples per read.
};
