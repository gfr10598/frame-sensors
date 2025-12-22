
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
    long base_count = 0; // Sample count of the first record in current_msg.
    TimeFitter fitter;
    LoggerMsg current_msg;

    IMUTracker() : fitter(0.001f), current_msg(LoggerMsg()) {}

    void update(LoggerMsg &msg)
    {
        if (msg.sample_count == 0)
            return;
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

    /// @brief Project this IMUTracker's data onto another IMUTracker's fitter.
    /// @param other
    /// @return the 'other' sample index of the first projected sample, and the projected values
    /// starting from that sample.
    std::pair<int64_t, LoggerMsg> project(const TimeFitter &other)
    {
        // This is the time of the first sample in the current msg.
        int64_t start_time = fitter.time_for(base_count);
        // Find the corresponding sample location in the other IMU.
        std::pair<int64_t, float> other_sample_base = other.sample_for(start_time);

        // printf("Last:  %5d %5d %5d\n", last_record[0], last_record[1], last_record[2]);
        // printf("First: %5d %5d %5d\n",
        //        current_msg.records[0].data[0],
        //        current_msg.records[0].data[1],
        //        current_msg.records[0].data[2]);
        // printf("base_count=%ld start_time=%lld\n", base_count, start_time);

        // Compute the size of the other IMU step size in units of this IMU's sample count.
        // NOTE: This should generally be less than 1.0, since we are projecting
        // onto the faster IMU timebase.  It should also be very stable.
        // Units are local steps per other step.
        float increment = other.slope() / slope();
        // printf("This IMU slope: %f other IMU slope: %f\n", slope(), other.slope());
        // printf("Projecting IMU: start_time=%lld other_sample_base=(%lld,%f) increment=%f\n",
        //        start_time, other_sample_base.first, other_sample_base.second, increment);

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

    printf("Projecting right onto left fitter\n");
    auto [offset, projected] = right.project(left.fitter);
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

// TODO - looking at pointers, it appears that Tracker is taking up 256 bytes?
class Tracker
{
private:
    int delta_time = 0;

    LoggerMsg msg;
    int next = 0;
    int count = 0;

public:
    Tracker() {}

    void replace(LoggerMsg &new_msg)
    {
        delta_time = new_msg.read_time - msg.read_time;
        msg = new_msg;
        if (msg.sample_count == 0 || msg.sample_count > 32)
        {
            printf("Problem: bad IMU message size: %d at %p\n", msg.sample_count, &msg);
            esp_backtrace_print(10);
            vTaskSuspend(NULL);
        }
        next = 0;
        count += msg.sample_count;
    }

    int next_time()
    {
        if (delta_time == 0)
            return -1;
        if (next >= msg.sample_count)
            return -1;
        return msg.read_time - (delta_time * (msg.sample_count - next) / msg.sample_count);
    }

    lsm6dsv16x_fifo_record_t &next_record()
    {
        return msg.records[next++];
    }

    int total_count()
    {
        return count;
    }
};

/// @brief Merges data from two IMUs.
class Merger
{
private:
    MergeMessage ping_pong[20]; // About 10 msec of data.
    int ping_pong_index = 0;    // Next entry to write into.
    Tracker left_tracker;
    Tracker right_tracker;

    IMUTracker left_imu;
    IMUTracker right_imu;

    bool last_imu = false; // Last IMU seen.
    int left_skips = 0;
    int right_skips = 0;

public:
    bool next(bool log = false)
    {
        int left_time = left_tracker.next_time();
        int right_time = right_tracker.next_time();
        if (left_time <= 0 || right_time <= 0)
            return false;
        while (left_time < right_time - 500)
        {
            // Drop the left sample.
            left_skips++;

            left_tracker.next_record();
            left_time = left_tracker.next_time();
            if (left_time <= 0)
                return false;
        }
        while (right_time < left_time - 500)
        {
            // Drop the right sample.
            right_skips++;

            right_tracker.next_record();
            right_time = right_tracker.next_time();
            if (right_time <= 0)
                return false;
        }
        if (log)
        {
            // printf("Next times: Left=%9d Right=%9d LS=%4d/%6d RS=%4d/%6d\n", left_time, right_time, left_skips, left_tracker.total_count(), right_skips, right_tracker.total_count());
        }
        lsm6dsv16x_fifo_record_t &left = left_tracker.next_record();
        lsm6dsv16x_fifo_record_t &right = right_tracker.next_record();

        if (&left == &right)
        {
            printf("Left and right records are the same!");
            esp_backtrace_print(10);
            vTaskSuspend(NULL);
        }

        auto merge = &ping_pong[ping_pong_index++];
        merge->data[0] = left.data[0];
        merge->data[1] = left.data[1];
        merge->data[2] = left.data[2];
        merge->data[3] = right.data[0];
        merge->data[4] = right.data[1];
        merge->data[5] = right.data[2];

        if (ping_pong_index == 10)
        {
            // Output merged data.
            auto &m = ping_pong[0];
        }
        else if (ping_pong_index == 20)
        {
            // Output merged data.
            auto &m = ping_pong[0];
            ping_pong_index = 0;
        }

        return true;
    }

    void handle(LoggerMsg &msg)
    {
        auto start = esp_timer_get_time();
        if (msg.imu == last_imu)
        {
            printf("****************************************** Warning: duplicate IMU message %d\n", msg.imu);
            return;
        }
        last_imu = msg.imu;

        if (msg.imu)
        {
            right_tracker.replace(msg);
            right_imu.update(msg);
        }
        else
        {
            left_tracker.replace(msg);
            left_imu.update(msg);
        }

        // Try to merge as much data as possible.
        next(true);
        while (next())
            ;

        if (left_tracker.total_count() > 16 && right_tracker.total_count() > 16)
        {
            if (left_imu.slope() < right_imu.slope())
            {
                // Left is faster.
                if (msg.imu)
                {
                    // Right IMU message just arrived.  Do projection onto left fitter.
                    auto projected = right_imu.project(left_imu.fitter);
                }
            }
            else
            {
                // Right is faster.
                if (!msg.imu)
                {
                    // Left IMU message just arrived.  Do projection onto right fitter.
                    auto projected = left_imu.project(right_imu.fitter);
                }
            }
        }
        auto end = esp_timer_get_time();
        // This printf is slow.  It doesn't help much to use -O2.  Cast to int also doesn't help.
        printf("Delay = %6d Merge time: %3d usec for %2d samples\n", (int)(start - msg.read_time), (int)(end - start), (int)(msg.sample_count));
        // This print alone takes over 1 msec.
        // Is the serial output just slow?  Do we need to change the baud rate?
        // printf("Delay %8lld\n", start - msg.read_time);
        // printf("Print time  : %3d usec\n", (int)(esp_timer_get_time() - end));
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
