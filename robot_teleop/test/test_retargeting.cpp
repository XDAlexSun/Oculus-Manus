// Plain-C++ unit test of the retargeting math (no ROS needed).
// Build:  g++ -std=c++17 -Wall -Wextra -I../src -o test_retargeting test_retargeting.cpp && ./test_retargeting

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "retargeting.hpp"

using namespace retarget;

static bool near(double a, double b, double tol = 1e-9) { return std::abs(a - b) < tol; }

// 21 landmarks where the index chain is straight along +y and every other point sits at the origin.
static std::vector<double> straightHand()
{
    std::vector<double> d(63, 0.0);
    const int idx[5] = {0, 5, 6, 7, 8};
    for (int k = 0; k < 5; ++k) d[3 * idx[k] + 1] = 0.03 * k;
    return d;   // (the original version had no return statement: undefined behaviour)
}

int main()
{
    // straight finger -> zero flexion everywhere
    auto d = straightHand();
    auto a = chainAngles(d.data(), kChains[1]);
    assert(near(a[0], 0.0) && near(a[1], 0.0) && near(a[2], 0.0));

    // bend the PIP joint by exactly 90 degrees
    d = straightHand();
    d[3 * 7 + 0] = 0.03;  d[3 * 7 + 1] = 0.06;
    d[3 * 8 + 0] = 0.06;  d[3 * 8 + 1] = 0.06;
    a = chainAngles(d.data(), kChains[1]);
    assert(near(a[0], 0.0));
    assert(near(a[1], M_PI / 2));          // PIP flexion is 90 deg
    assert(near(a[2], 0.0));               // DIP continues straight

    // a bend in the y-z plane must be measured: this fails if point() reads the wrong component
    d = straightHand();
    d[3 * 7 + 2] = 0.03;  d[3 * 7 + 1] = 0.06;     // DIP rises in +z
    d[3 * 8 + 2] = 0.06;  d[3 * 8 + 1] = 0.06;
    a = chainAngles(d.data(), kChains[1]);
    assert(near(a[1], M_PI / 2));

    // zero-length bones never produce NaN
    std::vector<double> z(63, 0.0);
    a = chainAngles(z.data(), kChains[2]);
    assert(near(a[0], 0.0) && near(a[1], 0.0) && near(a[2], 0.0));

    // curl normalisation clamps to [0,1] and tolerates a bad calibration
    assert(near(normalizeCurl(0.0, 0.3, 3.8), 0.0));
    assert(near(normalizeCurl(9.0, 0.3, 3.8), 1.0));
    assert(near(normalizeCurl(2.05, 0.3, 3.8), 0.5));
    assert(near(normalizeCurl(1.0, 1.0, 1.0), 0.0));

    // smoothing is independent of the update rate
    assert(near(smoothStep(0.0, 1.0, 0.09, 0.09), 1.0 - std::exp(-1.0)));
    double fast = 0.0, slow = 0.0;
    for (int i = 0; i < 90; ++i) fast = smoothStep(fast, 1.0, 0.001, 0.09);
    for (int i = 0; i < 9; ++i) slow = smoothStep(slow, 1.0, 0.01, 0.09);
    assert(near(fast, slow));
    assert(near(smoothStep(0.2, 0.8, 0.02, 0.0), 0.8));   // tau 0 = no smoothing

    std::puts("ALL RETARGET TESTS PASSED");
    return 0;
}
