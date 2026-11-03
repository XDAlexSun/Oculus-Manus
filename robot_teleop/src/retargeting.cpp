#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "retargetting.hpp"

using namespace retarget;

static bool near(double a, double b) { return std::abs(a - b) < 1e-9;}

static std::vector<double> straightHand()
{
    std::vector<double> d(63, 0.0);
    const int idx[5] = {0, 5, 6, 7, 8};
    for (int k = 0; k < 5; ++k) d[3*idx[k] + 1] = 0.03 * k;
    //populates the 21 landmarks such that index chain is straight along +y and other 
    //points are 0 at origin
}

int main(){
    //this main is used to test the code and see if it works

    //straight finger test
    auto d = straightHand();
    auto a = chainAngles(d.data(), kChains[1]);
    assert(near(a[0], 0.0) && near(a[1], 0.0) && near(a[2], 0.0));

    // pip joint bend by 90
    d = straightHand();
    d[3 * 7 + 0] = 0.03; 
    d[3 * 7 + 1] = 0.06;
    d[3 * 8 + 0] = 0.06; 
    d[3 * 8 + 1] = 0.06;
    a = chainAngles(d.data(), kChains[1]);
    assert(near(a[0], 0.0));
    assert(near(a[1], M_PI / 2));          // PIP flexion is 90 deg
    assert(near(a[2], 0.0));               // DIP continues straight

    // protection from zero length
    std::vector<double> z(63, 0.0);
    a = chainAngles(z.data(), kChains[2]);
    assert(near(a[0], 0.0) && near(a[1], 0.0) && near(a[2], 0.0));

    // curl normalisation clamps to [0,1] and tolerates a bad calibration
    assert(near(normalizeCurl(0.0, 0.3, 3.8), 0.0));
    assert(near(normalizeCurl(9.0, 0.3, 3.8), 1.0));
    assert(near(normalizeCurl(2.05, 0.3, 3.8), 0.5));
    assert(near(normalizeCurl(1.0, 1.0, 1.0), 0.0));

    // smoothing
    const double s1 = smoothStep(0.0, 1.0, 0.09, 0.09);
    assert(near(s1, 1.0 - std::exp(-1.0)));
    double fast = 0.0, slow = 0.0;
    for (int i = 0; i < 90; ++i) fast = smoothStep(fast, 1.0, 0.001, 0.09);
    for (int i = 0; i < 9; ++i) slow = smoothStep(slow, 1.0, 0.01, 0.09);
    assert(near(fast, slow));              // same time elapsed, same result
    assert(near(smoothStep(0.2, 0.8, 0.02, 0.0), 0.8));  // tau 0 = no smoothing
 
    std::puts("ALL RETARGET TESTS PASSED");
    return 0;
}