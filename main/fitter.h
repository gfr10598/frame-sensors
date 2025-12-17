#pragma once

/// LinearFitter is a simple linear regression class that can be used
/// to fit a line to a set of points.  It decays the accumulators
/// with time constant 1/alpha, measured in samples, so that older
/// data is gradually forgotten.

struct Line
{
    float x_mean;
    float y_mean;
    float slope;
};

/// LinearFitter is a simple linear regression class that can be used
/// to fit a line to a set of points.  It decays the accumulators
/// with time constant 1/alpha, measured in samples, so that the
/// line reflects the most recent data.
/// It also provides the residual deviation from the line.
class LinearFitter
{
public:
    /// Create a new LinearFitter, that decays at rate (1-alpha)
    /// If dx in non-zero, then newy(y) is equivalent to coord(x+dx, y)
    LinearFitter(float alpha)
        : x_center(0), y_center(0), xsum(0), x2sum(0), ysum(0),
          xysum(0), alpha(alpha), n(0) {}

    /// Add a point to the linear fitter.
    /// In release mode on x86_64, this takes about 20ns for f64
    void coord(long x_val, long y_val);

    /// @brief Recenter the fitter around the current xbar/ybar.
    void recenter();

private:
    // All computations are done relative to this center
    long x_center;
    long y_center;

    float xsum;
    float x2sum;
    float ysum;
    float xysum;
    float alpha;
    float n;

    float y(float x_val) const;

public:
    float predict(float x_val) const;
    float inverse(float y_val) const;
    float slope() const;
    Line fit() const;
};