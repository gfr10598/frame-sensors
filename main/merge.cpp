
#include <stdio.h>
#include "Arduino.h"
#include <string>
#include "IMU.h"
#include "lsm6dsv16x_reg.h"

#include "merge.h"

struct MergeMessage
{
    int16_t data[6]; // This will translate into 16 bytes of base64.
    int8_t left_cnt;
    int8_t right_cnt;
};

class Merger
{
private:
    MergeMessage ping_pong[20]; // About 10 msec of data.
    int left_index = 0;
    int right_index = 0;
    bool last_imu = false;
    bool last_written = false; // false for last, true for first.

    int total_left = 0;
    int total_right = 0;

public:
    /// @brief Merges one data record from left IMU.
    /// @param data
    void merge_left(lsm6dsv16x_fifo_record_t &rec)
    {
        printf("L");
        int16_t *data = rec.data;
        ping_pong[left_index].left_cnt = rec.tag.tag_cnt;
        ping_pong[left_index].data[0] = data[0];
        ping_pong[left_index].data[1] = data[1];
        ping_pong[left_index].data[2] = data[2];
        left_index = (left_index + 1) % 20;
        if (left_index == 0 && right_index < 10)
        {
            // Write out the last block
            if (last_written == false)
            {
                printf("\nLeft  merge is writing last  block  %1d %1d %2d %2d  ******************   ", ping_pong[10].left_cnt, ping_pong[10].right_cnt, left_index, right_index);
            }
            else
            {
                printf("\nLeft  merge is writing last  block  %1d %1d %2d %2d   ", ping_pong[10].left_cnt, ping_pong[10].right_cnt, left_index, right_index);
            }
            last_written = false;
        }
        else if (left_index == 10 && right_index >= 10)
        {
            if (last_written == true)
            {
                printf("\nLeft  merge is writing first block  %1d %1d %2d %2d  ******************   ", ping_pong[0].left_cnt, ping_pong[0].right_cnt, left_index, right_index);
            }
            else
            {
                printf("\nLeft  merge is writing first block  %1d %1d %2d %2d   ", ping_pong[0].left_cnt, ping_pong[0].right_cnt, left_index, right_index);
                last_written = true;
            }
        }
    }

    void merge_right(lsm6dsv16x_fifo_record_t &rec)
    {
        printf("R");
        int16_t *data = rec.data;
        ping_pong[right_index].right_cnt = rec.tag.tag_cnt;
        ping_pong[right_index].data[3] = data[0];
        ping_pong[right_index].data[4] = data[1];
        ping_pong[right_index].data[5] = data[2];
        right_index = (right_index + 1) % 20;
        if (right_index == 0 && left_index < 10)
        {
            if (last_written == false)
            {
                printf("\nRight merge is writing last  block  %1d %1d %2d %2d  ******************   ", ping_pong[10].left_cnt, ping_pong[10].right_cnt, left_index, right_index);
            }
            else
            {
                printf("\nRight merge is writing last  block  %1d %1d %2d %2d   ", ping_pong[10].left_cnt, ping_pong[10].right_cnt, left_index, right_index);
            }
            last_written = false;
        }
        else if (right_index == 10 && left_index >= 10)
        {
            if (last_written == true)
            {
                printf("\nRight merge is writing first block  %1d %1d %2d %2d  ******************   ", ping_pong[0].left_cnt, ping_pong[0].right_cnt, left_index, right_index);
            }
            else
            {
                printf("\nRight merge is writing first block  %1d %1d %2d %2d   ", ping_pong[0].left_cnt, ping_pong[0].right_cnt, left_index, right_index);
            }
            last_written = true;
        }
    }

    void handle(LoggerMsg &msg)
    {
        if (msg.imu == last_imu)
        {
            printf("****************************************** Warning: duplicate IMU message %d\n", msg.imu);
            return;
        }
        last_imu = msg.imu;

        int count = 0;
        if (left_index == 0 && right_index == 0)
        {
            if (msg.imu)
                left_index = 4;
            else
                right_index = 4;
        }

        for (int i = 0; i < msg.sample_count; i++)
        {
            lsm6dsv16x_fifo_record_t &rec = msg.records[i];
            if (rec.tag.tag_sensor == 2) // XL
            {
                count++;
                if (msg.imu)
                    merge_right(rec);
                else
                    merge_left(rec);
            }
        }

        if (msg.imu)
        {
            // Right IMU should be ahead of left by roughly 4 samples.
            int offset = (right_index - left_index + 20) % 20;
            // printf("Right: %2d %2d %2d %2d\n", right_index, left_index, offset, count);
            // Bad hack for now
            if (count >= 8 && offset < 3)
            {
                // merge_right(msg.records[msg.sample_count - 1]);
            }
        }
        else
        {
            // Left IMU should be behind right by roughly 4 samples.
            int offset = (left_index - right_index + 20) % 20;
            // printf("Left: %2d %2d %2d %2d\n", right_index, left_index, offset, count);
            // Bad hack for now
            if (count >= 8 && offset < 3)
            {
                // merge_left(msg.records[msg.sample_count - 1]);
            }
        }
    }
};

Merger merger;

void logger_task(void *q)
{
    QueueHandle_t queue = (QueueHandle_t)q;

    while (1)
    {
        LoggerMsg msg;
        if (xQueueReceive(queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            merger.handle(msg);
            // printf("Logger: IMU: %d Read %2d samples at %4ld usec (%d)\n", msg.imu, msg.sample_count, msg.read_time, msg.delayed);
        }
    }
}
