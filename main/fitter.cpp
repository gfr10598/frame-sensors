#include "fitter.h"
#include <utility>
#include <stdio.h>

void TimeFitter::coord(long x, long y)
{
    float x_val = x - sample_offset;
    float y_val = y - time_offset;

    if (n == 0.0f)
    {
        ksum = x_val;
        k2sum = x_val * x_val;
        tsum = y_val;
        ktsum = x_val * y_val;
        n = 1.0f;
        return;
    }
    ksum += x_val - ksum * alpha;
    tsum += y_val - tsum * alpha;
    k2sum += x_val * x_val - k2sum * alpha;
    ktsum += x_val * y_val - ktsum * alpha;
    n += 1.0f - n * alpha;

    if (recenter_count++ >= 100)
    {
        recenter();
        recenter_count = 0;
    }
}

/// TODO - need to verify that this is correct.
/// Simple to do - just verify that the line doesn't change.
void TimeFitter::recenter()
{
    long dx = ksum / n;
    long dy = tsum / n;
    sample_offset += dx;
    time_offset += dy;

    ksum -= dx * n;
    tsum -= dy * n;
    k2sum -= 2 * dx * ksum + (dx * dx) * n;
    ktsum -= dy * ksum + dx * tsum + (dx * dy) * n;
}

long TimeFitter::time_for(long x_val) const
{
    x_val -= sample_offset;
    float yb = tsum / n;
    float xb = ksum / n;
    return time_offset + (long)(yb + slope() * (x_val - xb));
}

std::pair<long, float> TimeFitter::sample_for(long t) const
{
    t -= time_offset;
    float yb = tsum / n;
    float xb = ksum / n;
    float local = xb + (t - yb) / slope();
    // Truncate the small part to get the integer sample index.
    long index = sample_offset + (long)local;
    float frac = local - (long)local;
    if (frac < 0)
    {
        frac += 1.0f;
        index -= 1;
    }
    return {index, frac};
}

float TimeFitter::slope() const
{
    return (n * ktsum - ksum * tsum) / (n * k2sum - ksum * ksum);
}

void test_fitter()
{
    TimeFitter fitter(0.01f);

    for (long i = 0; i < 1000; i++)
    {
        fitter.coord(i, i * 10 + 500);
    }
    assert(fitter.slope() > 9.9f && fitter.slope() < 10.1f);

    for (long i = 0; i < 1000; i += 100)
    {
        long t = fitter.time_for(i);
        auto [k, frac] = fitter.sample_for(t);
        printf("Sample %ld => time %ld => sample %ld + %f\n", i, t, k, frac);
    }
}
