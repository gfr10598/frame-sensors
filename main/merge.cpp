#include <stdio.h>
#include "Arduino.h"
#include <string>
#include "base64_encode.hpp"
#include "IMU.h"

struct LoggerMsg
{
    bool delayed;      // Whether the vTaskDelayUntil was delayed.
    int32_t read_time; // usec time at end of collection
    bool imu;          // Which IMU was collected.
    int sample_count;
    lsm6dsv16x_fifo_record_t records[32]; // Up to 16 samples per read.
};

void logger_task(void *q)
{
    QueueHandle_t queue = (QueueHandle_t)q;
    while (1)
    {
        LoggerMsg msg;
        if (xQueueReceive(queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            printf("Logger: IMU: %d Read %2d samples at %4ld usec (%d)\n", msg.imu, msg.sample_count, msg.read_time, msg.delayed);
        }
    }
}
