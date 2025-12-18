
#include <stdio.h>
#include "Arduino.h"
#include <string>
#include "IMU.h"
#include "lsm6dsv16x_reg.h"

#include "fitter.h"

#include "merge.h"

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
    TimeFitter fitter;
    LoggerMsg current_msg;
    long base_count = 0;          // Sample count of the first record in current_msg.
    int16_t last_record[3] = {0}; // Last record from previous message.
    int next;

public:
    IMUTracker() : fitter(0.001f) {}
    void update(LoggerMsg msg)
    {
        // Update the fitter with the new data.
        if (current_msg.sample_count > 0)
        {
            base_count += current_msg.sample_count;
            fitter.coord(base_count, current_msg.read_time);
            for (int i = 0; i < 3; i++)
                last_record[i] = current_msg.records[current_msg.sample_count - 1].data[i];
        }
        current_msg = msg;
        next = 0;
    }

    // Time attributed to a given sample count.
    int64_t time_for(long sample_count)
    {
        return fitter.time_for(sample_count);
    }

    // Time between samples in usec.
    int64_t delta_t()
    {
        return (int64_t)(fitter.slope());
    }

    /// @brief Project this IMUTracker's data onto another IMUTracker's fitter.
    /// @param other
    /// @return the 'other' sample index of the first projected sample, and the projected values
    /// starting from that sample.
    std::pair<int64_t, LoggerMsg> project(const TimeFitter &other)
    {
        LoggerMsg projected;
        // This is the time of the first sample in the current msg.
        int64_t start_time = fitter.time_for(base_count);
        // Find the corresponding sample location in the other IMU.
        std::pair<int64_t, float> other_sample_base = other.sample_for(start_time);

        // Compute the size of the other IMU step size in units of this IMU's sample count.
        // NOTE: This should generally be less than 1.0, since we are projecting
        // onto the faster IMU timebase.  It should also be very stable.
        // Units are local steps per other step.
        float increment = other.slope() / fitter.slope();

        float fraction_index = other_sample_base.second * increment;

        // Find the index into the local data.
        long other_time = start_time - (other_sample_base.second);

        // We need to project the sample before the first sample in the LoggerMsg, using
        // the last_record.
        int k = 0;
        auto rec = &projected.records[k++];
        float alpha = other_sample_base.second; // The distance from k-1 sample from other IMU.
        // compute the interpolated values.
        for (int i = 0; i < 3; i++)
        {
            float value = last_record[i] + (current_msg.records[0].data[i] - last_record[i]) * alpha;
            rec->data[i] = (int16_t)value;
        }

        // Now project the rest of the samples.
        for (int i = 0; i < current_msg.sample_count; i++)
        {
            float frac = other_sample_base.second + increment;
        }

        return {other_sample_base.first, projected};
    }
};

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
        }
        else
        {
            left_tracker.replace(msg);
        }

        // Try to merge as much data as possible.
        next(true);
        while (next())
            ;

        auto end = esp_timer_get_time();
        printf("Merge time: %d usec for %d samples\n", (int)(end - start), msg.sample_count);
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
