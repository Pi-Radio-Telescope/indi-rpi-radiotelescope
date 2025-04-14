#ifndef AXISRT_H
#define AXISRT_H

#include <cmath>
#include <functional>
#include <inttypes.h> // uint8_t, etc
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace PiRaTe {

constexpr double pi() { return std::acos(-1); }
constexpr double twopi() { return 2. * pi(); }
constexpr double toDeg(double unitVal) { return (unitVal * 360.); }
constexpr double toHours(double unitVal) { return (unitVal * 24.); }
constexpr double toRadians(double unitVal) { return (unitVal * twopi()); }

/**
 * @brief A class for handling a rotational axis.
 * This class can represent any kind of values within a cyclic range and a lower and upper boundary. When a value is assigned
 * through {@link RotAxis::setValue} which lies outside the axis range, it will be folded back into the allowed range
 * by symmetry transformations.
 * @author HG Zaunick
 */
class RotAxis {
public:
    /**
	 * @brief The main constructor.
	 * Initializes an object with the given properties. The RotAxis' value will be initialized to zero.
	 * @param min the minimum boundary of the value range
	 * @param max the maximum boundary of the value range
	 * @param ref_unit the periodicity of the value range (default 2pi)
	 * @throws std::exception if min>max or ref_unit==0
	 */
    RotAxis(double min, double max, double ref_unit = twopi());

    /**
	 * @brief Assignment from value.
	 * Assigns a new value to the object. The value is forwarded to {@link RotAxis::setValue}.
	 * @param a_value the value to set
	 */
    RotAxis& operator=(double a_value);

    [[nodiscard]] auto degrees() const -> double { return toDeg(fValue); }
    [[nodiscard]] auto radians() const -> double { return toRadians(fValue); }
    [[nodiscard]] auto hourAngle() const -> double { return toHours(fValue); }

    /**
	 * @brief Assignment from value.
	 * Assigns a new value to the object.
	 * @param val the value to set
	 * @note The value is internally projected back to the principal period (given by ref_unit in the constructor {@link RotAxis::RotAxis})
	 */
    void setValue(double val);

    /**
	 * @brief Current value.
	 * Read the currently set value
	 * @return the current value
	 */
    [[nodiscard]] auto value() const -> double { return fValue * fRefUnit; }

    /**
	 * @brief Register callback function for gimbal flip.
	 * When setting a new value to the object, it may happen that, depending on the boundaries set, the axis value is folded back
	 * due to the axis' symmetry. In such cases, the orthogonal axis flips to the opposite side (gimbal flip). With this method, a
	 * function can be registered which then performs this operation, e.g. the {@link RotAxis::gimbalFlip} member method of the complementary axis object.
	 * Example: an axis [-pi/2..pi/2] walks with increasing values over pi/2. For further increasing angles, the resulting axis
	 * angle will be mirrored at pi/2 and therefore decreases again. During the transition over the symmetry axis, the orthogonal axis
	 * flipped around by pi.
	 * 
	 * @param fn a function object to set the callback function which is called when a gimbal flip occurs in this object.
	 */
    void registerGimbalFlipCallback(std::function<void()> fn) { fGimbalFlipFn = fn; }
    void gimbalFlip() { this->setValue(fRefUnit * (fValue + 0.5)); }

private:
    double fRefUnit { twopi() };
    double fValue { 0. };
    double fMin { -1 };
    double fMax { 1 };
    bool fValid { false };

    [[nodiscard]] static auto reduceToPrincipal(double arg) -> double;
    auto adjustToRange(double& value) const -> bool;
    std::function<void()> fGimbalFlipFn {};
};

} // namespace PiRaTe

#endif
