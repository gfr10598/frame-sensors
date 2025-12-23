
#include <cassert>
#include <stdio.h>
#include "Arduino.h"
#include <string>
#include "esp_debug_helpers.h"

#include "lsm6dsv16x_reg.h"
#include "IMU.h"

#include "fitter.h"
#include "merge.h"

/// @brief Reproject samples using linear interpolation.
/// @param last  The last sample prior to the message.
/// @param msg The message to reproject.
/// @param start The fractional start position (<= 1.0).
/// @param increment The fractional step size (usually < 1.0).
///
/// Performance - about 30 usec for 8 samples, with debug.
LoggerMsg reproject(const int16_t last[3], const LoggerMsg &msg, float start, float increment)
{
    LoggerMsg projected;
    int16_t *a = (int16_t *)last;
    int n = 0; // The output index.

    float alpha = start;
    int k = (int)(alpha);
    alpha -= k;
    if (k == 0)
    {
        // printf("Reproject k=%d n=%d alpha=%f\n", k, n, alpha);

        int16_t *b = (int16_t *)msg.records[k].data;
        int16_t *out = (int16_t *)projected.records[n++].data;
        for (int i = 0; i < 3; i++)
        {
            float value = a[i] + alpha * (b[i] - a[i]);
            out[i] = (int16_t)value;
        }
        alpha += increment;
        if (alpha >= 1.0f)
        {
            k++;
            a = b;
            alpha -= 1.0f;
        }
    }

    while (k < msg.sample_count && n < 32)
    {
        // printf("Reproject k=%d n=%d alpha=%f\n", k, n, alpha);

        int16_t *b = (int16_t *)msg.records[k].data;
        int16_t *out = (int16_t *)projected.records[n++].data;
        for (int i = 0; i < 3; i++)
        {
            float value = a[i] + alpha * (b[i] - a[i]);
            out[i] = (int16_t)value;
        }
        alpha += increment;
        if (alpha >= 1.0f)
        {
            k++;
            a = b;
            alpha -= 1.0f;
        }
    }

    projected.sample_count = n;
    projected.read_time = msg.read_time;

    return projected;
}

void test_reproject()
{
    LoggerMsg msg;
    msg.sample_count = 4;
    for (int i = 0; i < msg.sample_count; i++)
    {
        msg.records[i].data[0] = i * 100;
        msg.records[i].data[1] = i * 100 + 1;
        msg.records[i].data[2] = i * 100 + 2;
    }
    for (int i = 0; i < msg.sample_count; i++)
    {
        printf("Initial[%d]: %d %d %d\n", i, msg.records[i].data[0], msg.records[i].data[1], msg.records[i].data[2]);
    }

    int16_t last[3] = {-100, -99, -98};
    float start = 0.9f;
    float increment = 0.85f;
    LoggerMsg projected = reproject(last, msg, start, increment);
    for (int i = 0; i < projected.sample_count; i++)
    {
        printf("Projected[%d]: %d %d %d\n", i, projected.records[i].data[0], projected.records[i].data[1], projected.records[i].data[2]);
    }

    assert(projected.sample_count == 4);
    assert(projected.records[0].data[0] == -10);
    assert(projected.records[1].data[0] == 75);
    assert(projected.records[2].data[0] == 160);
    assert(projected.records[3].data[0] == 245);
}

// This module merges data from two IMUs.  It leaves the faster
// IMU data unchanged, and interpolates the slower IMU data to
// match the timing of the faster IMU data.

struct MergeMessage
{
    int16_t data[6]; // This will translate into 16 bytes of base64.
};

/// @brief Tracks an individual IMU's data and data rate.
class IMUTracker
{
private:
    int16_t last_record[3] = {0}; // Last record from previous message.

public:
    long msg_count = 0;  // Number of messages processed.
    long base_count = 0; // Cumulative sample count of the first record in current_msg.
    TimeFitter fitter;
    LoggerMsg current_msg;

    IMUTracker() : fitter(0.001f), current_msg(LoggerMsg()) {}

    void update(LoggerMsg &msg)
    {
        if (msg.sample_count == 0)
            return;
        msg_count++;
        if (msg.sample_count >= 32)
        {
            printf("Problem: large IMU message size: %d\n", msg.sample_count);
            esp_backtrace_print(10);
            vTaskSuspend(NULL);
        }

        base_count += current_msg.sample_count;
        fitter.coord(base_count + msg.sample_count, msg.read_time);

        // Update the fitter with the new data.
        if (current_msg.sample_count > 0)
        {
            if (current_msg.sample_count >= 20)
            {
                printf("Problem: large IMU message size: %d %p\n", current_msg.sample_count, &msg);
                esp_backtrace_print(10);
                vTaskSuspend(NULL);
            }
            for (int i = 0; i < 3; i++)
                last_record[i] = current_msg.records[current_msg.sample_count - 1].data[i];
        }
        else
        {
            for (int i = 0; i < 3; i++)
                last_record[i] = msg.records[0].data[i];
        }
        current_msg = msg;
    }

    float slope() const
    {
        return fitter.slope();
    }

    // Time attributed to a given sample count.
    int64_t time_for(long sample_count)
    {
        return fitter.time_for(sample_count);
    }

    std::pair<long, float> sample_for(long t) const
    {
        return fitter.sample_for(t);
    }

    /// @brief Project this IMUTracker's data onto another IMUTracker's fitter.
    /// @param other
    /// @return the 'other' sample index of the first projected sample, and the projected values
    /// starting from that sample.
    /// @TODO - this takes quite a bit of stack space.  Can we reduce it?
    std::pair<int64_t, LoggerMsg> project(const TimeFitter &other)
    {
        // This is the time of the first sample in the current msg.
        int64_t start_time = fitter.time_for(base_count);
        // Find the corresponding sample location in the other IMU.
        std::pair<int64_t, float> other_sample_base = other.sample_for(start_time);

        // Compute the size of the other IMU step size in units of this IMU's sample count.
        // NOTE: This should generally be less than 1.0, since we are projecting
        // onto the faster IMU timebase.  It should also be very stable.
        // Units are local steps per other step.
        float increment = other.slope() / slope();

        // This should always be less than 1.0.
        float local_fraction = other_sample_base.second * increment;

        LoggerMsg projected = reproject(
            last_record, current_msg, local_fraction, increment);
        return {other_sample_base.first, projected};
    }
};

LoggerMsg make_test_msg(int sample_count, int64_t read_time, int64_t time_step)
{
    LoggerMsg msg;
    // At the end of the samples, the values should be equal to the read time / 500.
    // And they should increment by either
    auto start_time = read_time - time_step * sample_count;
    msg.read_time = read_time;
    msg.sample_count = sample_count;
    for (int i = 0; i < sample_count; i++)
    {
        msg.records[i].data[0] = start_time;
        msg.records[i].data[1] = start_time + 1;
        msg.records[i].data[2] = start_time + 2;
        start_time += time_step;
    }
    return msg;
}

/// @brief This requires three IMU messages for each IMU.  The first
/// provides the basis for last_record and the first time point.
/// The second and third provide data to fit the timebase.
void test_imu_tracker()
{
    IMUTracker left, right;

    LoggerMsg msg1 = make_test_msg(8, 2000, 500);
    left.update(msg1);
    LoggerMsg msg2 = make_test_msg(7, 4000, 8 * 500 / 7);
    right.update(msg2);

    msg1 = make_test_msg(8, 6000, 500);
    left.update(msg1);
    msg2 = make_test_msg(7, 8000, 8 * 500 / 7);
    right.update(msg2);

    msg1 = make_test_msg(8, 10000, 500);
    left.update(msg1);
    msg2 = make_test_msg(7, 12000, 8 * 500 / 7);
    right.update(msg2);

    printf("Min stack in test_imu_tracker: %d\n", uxTaskGetStackHighWaterMark(NULL));

    printf("Projecting right onto left fitter\n");
    auto [offset, projected] = right.project(left.fitter);
    printf("Min stack after project: %d\n", uxTaskGetStackHighWaterMark(NULL));
    printf("Projected offset: %lld\n", offset);
    // assert(projected.sample_count == 8);
    printf("Left base %ld  Right base %ld\n", left.base_count, right.base_count);
    for (int i = 0; i < projected.sample_count; i++)
    {
        printf("Left  [%d]: %5d %5d %5d", i, left.current_msg.records[i].data[0], left.current_msg.records[i].data[1], left.current_msg.records[i].data[2]);
        printf("  Right [%d]: %5d %5d %5d", i, right.current_msg.records[i].data[0], right.current_msg.records[i].data[1], right.current_msg.records[i].data[2]);
        printf("  Projected[%d]: %5d %5d %5d\n", i, projected.records[i].data[0], projected.records[i].data[1], projected.records[i].data[2]);
    }
}

/// @brief Merges data from two IMUs.
class Merger
{
private:
    bool first_write = true;
    MergeMessage ping_pong[20]; // About 10 msec of data.
    int left_index = 0;         // Next entry to write left data into.
    int right_index = 0;        // Next entry to write right data into.
    bool left_faster = false;   // Is left IMU faster?

    IMUTracker left_imu;
    IMUTracker right_imu;

    bool last_imu = false; // Last IMU seen.

    void output(const MergeMessage *msg)
    {
        // Output the merged message.
        // For now, just print it.
        if (true)
        {
            printf("0 %5d %5d %5d %5d %5d %5d\n",
                   msg->data[0], msg->data[1], msg->data[2],
                   msg->data[3], msg->data[4], msg->data[5]);
            msg += 5;
            printf("5 %5d %5d %5d %5d %5d %5d\n",
                   msg->data[0], msg->data[1], msg->data[2],
                   msg->data[3], msg->data[4], msg->data[5]);
        }
        else
        {
            printf("%5d %5d\n", msg->data[2], msg->data[5]);
            msg += 2;
            printf("%5d %5d\n", msg->data[2], msg->data[5]);
            msg += 2;
            printf("%5d %5d\n", msg->data[2], msg->data[5]);
            msg += 2;
            printf("%5d %5d\n", msg->data[2], msg->data[5]);
            msg += 2;
            printf("%5d %5d\n", msg->data[2], msg->data[5]);
            msg += 2;
        }
    }

public:
    /// @brief Whenever we fill, we should only fill slots that have both left and right data.
    /// @param msg
    void fill_left(LoggerMsg &msg)
    {
        int start_index = 0;
        if (first_write)
        {
            auto start_time = right_imu.time_for(right_imu.base_count + right_imu.current_msg.sample_count);
            auto [left_sample, frac] = left_imu.sample_for(start_time);
            start_index = left_sample - left_imu.base_count;
            first_write = false;
        }
        bool wrap10 = false;
        bool wrap20 = false;
        for (int i = 0; i < msg.sample_count; i++)
        {
            auto left = msg.records[i];
            auto merge = &ping_pong[left_index++];
            if (left_index == 10)
            {
                wrap10 = true;
            }
            if (left_index >= 20)
            {
                left_index = 0;
                wrap20 = true;
            }
            merge->data[0] = left.data[0];
            merge->data[1] = left.data[1];
            merge->data[2] = left.data[2];
        }
        if (wrap10 && right_index >= 10)
            output(&ping_pong[0]);
        if (wrap20 && (right_index < 10))
            output(&ping_pong[10]);
    }

    void fill_right(LoggerMsg &msg)
    {
        int start_index = 0;
        if (first_write)
        {
            auto start_time = left_imu.time_for(left_imu.base_count + left_imu.current_msg.sample_count);
            auto [right_sample, frac] = right_imu.sample_for(start_time);
            start_index = right_sample - right_imu.base_count;
            first_write = false;
        }
        bool wrap10 = false;
        bool wrap20 = false;
        for (int i = start_index; i < msg.sample_count; i++)
        {
            auto right = msg.records[i];
            auto merge = &ping_pong[right_index++];
            if (right_index == 10)
                wrap10 = true;
            if (right_index >= 20)
                right_index = 0;
            merge->data[3] = right.data[0];
            merge->data[4] = right.data[1];
            merge->data[5] = right.data[2];
        }
        if (wrap10 && left_index >= 10)
            output(&ping_pong[0]);
        if (wrap20 && (left_index < 10))
            output(&ping_pong[10]);
    }

    /// @brief  Process left values.
    /// @precondition left_faster has been initialized.
    /// ### TODO ### We need to skip the initial unmatched samples.
    void process_left(LoggerMsg &left)
    {
        left_imu.update(left);
        if (left_imu.msg_count < 10 || right_imu.msg_count < 10)
            return;

        if (left_faster)
        {
            fill_left(left);
        }
        else
        {
            // The right side dictates timing, and where the left samples go.
            // BUG - there may be a problem with number of samples here.
            // Sometimes there should be same number and sometimes +1.
            auto [start, interp] = left_imu.project(right_imu.fitter);
            fill_left(interp);
        }
    }

    void process_right(LoggerMsg &right)
    {
        right_imu.update(right);
        if (left_imu.msg_count < 10 || right_imu.msg_count < 10)
        {
            // We only need to set the faster IMU once, and it doesn't matter
            // whether we do that on a left or right message.
            // This will set it multiple times, until we are ready to start
            // merging.
            if (left_imu.msg_count > 5 && right_imu.msg_count > 5)
            {
                // Determine which IMU is faster.
                float left_slope = left_imu.slope();
                float right_slope = right_imu.slope();
                left_faster = left_slope < right_slope;
            }
            return;
        }

        if (!left_faster)
        {
            fill_right(right);
        }
        else
        {
            auto [start, interp] = right_imu.project(left_imu.fitter);
            fill_right(interp);
        }
    }

    void handle(LoggerMsg &msg)
    {
        auto start = esp_timer_get_time();
        // Rewrite the record, omitting unused sensor types.
        int pack = 0;
        for (int i = 0; i < msg.sample_count; i++)
        {
            auto tag = msg.records[i].tag.tag_sensor;
            if (tag == 2)
                msg.records[pack++] = msg.records[i];
        }
        msg.sample_count = pack;

        if (msg.imu == last_imu)
        {
            printf("****************************************** Warning: duplicate IMU message %d\n", msg.imu);
            return;
        }
        last_imu = msg.imu;

        if (msg.imu)
            process_left(msg);
        else
            process_right(msg);

        auto end = esp_timer_get_time();
        // Printing is slow unless we change the default baud rate.  See main().
        // printf("%d Delay: %6d usec  Merge: %3d usec samples: %2d\n", msg.imu ? 1 : 0, (int)(start - msg.read_time), (int)(end - start), (int)(msg.sample_count));
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
            if (msg.sample_count > 20)
            {
                printf("****************************************** Warning: large IMU message %d samples\n", msg.sample_count);
            }
            merger.handle(msg);
            // printf("Logger: IMU: %d Read %2d samples at %4ld usec (%d)\n", msg.imu, msg.sample_count, msg.read_time, msg.delayed);
        }
        else
        {
            printf("Logger: Queue receive failed!\n");
        }
    }
}
