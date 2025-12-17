#include "fitter.h"

void LinearFitter::coord(long x, long y)
{
    float x_val = x - x_center;
    float y_val = y - y_center;

    if (n == 0.0f)
    {
        xsum = x_val;
        x2sum = x_val * x_val;
        ysum = y_val;
        xysum = x_val * y_val;
        n = 1.0f;
        return;
    }
    xsum += x_val - xsum * alpha;
    ysum += y_val - ysum * alpha;
    x2sum += x_val * x_val - x2sum * alpha;
    xysum += x_val * y_val - xysum * alpha;
    n += 1.0f - n * alpha;
}

/// TODO - need to verify that this is correct.
/// Simple to do - just verify that the line doesn't change.
void LinearFitter::recenter()
{
    long dx = xsum / n;
    long dy = ysum / n;
    x_center += dx;
    y_center += dy;

    xsum -= dx * n;
    ysum -= dy * n;
    x2sum -= 2 * dx * xsum + (dx * dx) * n;
    xysum -= dy * xsum + dx * ysum + (dx * dy) * n;
}

float LinearFitter::predict(float x_val) const
{
    x_val -= x_center;

    float yb = ysum / n;
    float xb = xsum / n;
    return y_center + yb + slope() * (x_val - xb);
}

float LinearFitter::inverse(float y_val) const
{
    y_val -= y_center;

    float yb = ysum / n;
    float xb = xsum / n;
    return x_center + xb + (y_val - yb) / slope();
}

float LinearFitter::slope() const
{
    // printf("slope: n=%f xysum=%f xsum=%f ysum=%f x2sum=%f\n", n, xysum, xsum, ysum, x2sum);
    return (n * xysum - xsum * ysum) / (n * x2sum - xsum * xsum);
}

Line LinearFitter::fit() const
{
    return Line{x_center + xsum / n, y_center + ysum / n, slope()};
}