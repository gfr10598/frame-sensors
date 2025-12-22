#pragma once

#include <stdio.h>
#include "LSM6DSV16XSensor.h"

void logger_task(void *q);

struct LoggerMsg
{
    lsm6dsv16x_fifo_record_t records[32]; // Up to 32 samples per read.
    int64_t read_time{0};                 // usec time at end of collection
    uint16_t sample_count{0};
    bool delayed{false}; // Whether the vTaskDelayUntil was delayed.
    bool imu;            // Which IMU was collected.
};

void test_reproject();
void test_imu_tracker();
