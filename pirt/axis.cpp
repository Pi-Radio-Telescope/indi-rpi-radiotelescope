#include <cmath>
#include <functional>
#include <inttypes.h> // uint8_t, etc
#include <iomanip>
#include <iostream>
#include <list>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "axis.h"

namespace PiRaTe {

RotAxis::RotAxis(double min, double max, double ref_unit)
    : fValue { 0. }
{
    if (fRefUnit == 0.)
        throw std::exception();
    if (max <= min)
        throw std::exception();
    fRefUnit = ref_unit;
    fMin = min / fRefUnit;
    fMax = max / fRefUnit;
    fValid = true;
}

RotAxis& RotAxis::operator=(double a_value)
{
    this->setValue(a_value);
    return *this;
}

void RotAxis::setValue(double val)
{
    double princ_val = reduceToPrincipal(val / fRefUnit);
    if (fMin < 0. && princ_val > 0.5)
        princ_val -= 1;
    unsigned int backfolding_iterations { 10 };
    while (backfolding_iterations-- && adjustToRange(princ_val)) {
        if (fGimbalFlipFn)
            fGimbalFlipFn();
    }
    fValue = princ_val;
    fValid = true;
}

auto RotAxis::reduceToPrincipal(double arg) -> double
{
    while (arg > 1.)
        arg -= 1.;
    while (arg < 0.)
        arg += 1.;
    return arg;
}

auto RotAxis::adjustToRange(double& value) const -> bool
{
    if (value > fMax) {
        value = 2. * fMax - value;
        return true;
    } else if (value < fMin) {
        value = 2. * fMin - value;
        return true;
    }
    return false;
}

} // namespace PiRaTe
