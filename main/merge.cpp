
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
    int8_t left_cnt;
    int8_t right_cnt;
};

/// @brief Tracks sampling skew between two IMUs.
/// This tracks the incoming sample timings of the two IMUs,
/// estimating the timing only from the timestamp associated with
/// the end of each collection.  It estimates time as a function
/// of sample count, using a decaying linear fitter.
///
/// BUG - float only has a precision of 24 bits, so the deltas will
/// become inaccurate after about 10 million samples, which is only
/// about 2 hours at 2000 samples/sec.
class SkewTracker
{
public:
    LinearFitter left;
    LinearFitter right;
    long left_count = 0; // At 2000 samples/sec, this will saturate in 20 days
    long right_count = 0;

    SkewTracker() : left(0.001f), right(0.001f) {}

    /// @brief
    /// @param count - number of samples collected
    /// @param time  - time in usec at end of collection
    void update_left(int count, int64_t usec)
    {
        left_count += count;
        left.coord(left_count, usec);
        left.recenter();
    }

    void add_right(int count, int64_t usec)
    {
        right_count += count;
        right.coord(right_count, usec);
        right.recenter();
    }

    // We want to interpolate whichever IMU has the higher slope, meaning
    // that the interval between samples is longer.
    // We plug in the sample count for the faster IMU to get a timestamp,
    // then interpolate the slower IMU samples to match that timestamp.
    //
    // The slower IMU samples each have a corresponding timestamp from
    // the linear fitter.  We estimate the timestamps by just incrementing
    // by the slope for each IMU.
    //
    // Note that the incoming sample timestamps are used only for updating
    // the linear fitters.
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

    SkewTracker skew_tracker;

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
        merge->left_cnt = left.tag.tag_cnt;
        merge->right_cnt = right.tag.tag_cnt;

        if (ping_pong_index == 10)
        {
            // Output merged data.
            auto &m = ping_pong[0];
            printf("LCnt=%2d RCnt=%2d LSlope:%f RSlope:%f\n",
                   m.left_cnt, m.right_cnt, skew_tracker.left.slope(), skew_tracker.right.slope());
        }
        else if (ping_pong_index == 20)
        {
            // Output merged data.
            auto &m = ping_pong[0];
            printf("LCnt=%2d RCnt=%2d LSlope:%f RSlope:%f\n",
                   m.left_cnt, m.right_cnt, skew_tracker.left.slope(), skew_tracker.right.slope());
            ping_pong_index = 0;
            {
                auto t = skew_tracker.left.predict(skew_tracker.left_count);
                float right = skew_tracker.right.inverse((float)t);
                printf("%ld -> %8.1f\n", skew_tracker.left_count, right);
            }
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
            skew_tracker.add_right(msg.sample_count, msg.read_time);
            right_tracker.replace(msg);
        }
        else
        {
            skew_tracker.update_left(msg.sample_count, msg.read_time);
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
