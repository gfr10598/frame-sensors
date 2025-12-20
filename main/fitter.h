#pragma once

#include <stdint.h>
#include <utility>

/// LinearFitter is a simple linear regression class that can be used
/// to fit a line to a set of points.  It decays the accumulators
/// with time constant 1/alpha, measured in samples, so that older
/// data is gradually forgotten.

/// LinearFitter is a simple linear regression class that can be used
/// to fit a line to a set of points.  It decays the accumulators
/// with time constant 1/alpha, measured in samples, so that the
/// line reflects the most recent data.
/// It also provides the residual deviation from the line.
class TimeFitter
{
public:
    /// Create a new LinearFitter, that decays at rate (1-alpha)
    /// If dx in non-zero, then newy(y) is equivalent to coord(x+dx, y)
    TimeFitter(float alpha)
        : sample_offset(0), time_offset(0), ksum(0), k2sum(0), tsum(0),
          ktsum(0), alpha(alpha), n(0) {}

    /// Add a point to the linear fitter.
    /// In release mode on x86_64, this takes about 20ns for f64
    void coord(long k, long t);

    /// @brief  Predict time value for given sample index.
    /// @param k  sample index
    /// @return time in microseconds
    long time_for(long k) const;
    /// @brief  Predict sample index for given time value.
    /// @param t
    /// @return The index and fractional index corresponding to the given time.
    std::pair<long, float> sample_for(long t) const;

    float slope() const;

private:
    /// @brief Recenter the fitter around the current kbar/tbar.
    void recenter();

    // All computations are done relative to this center
    long sample_offset{0};
    long time_offset{0};

    float ksum{0};
    float k2sum{0};
    float tsum{0};
    float ktsum{0};
    float alpha{0};
    float n{0};

    int recenter_count;
};

void test_fitter();
