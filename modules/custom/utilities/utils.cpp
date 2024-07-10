#include "utils.h"
#include <cmath>

// Utility function to constrain a value to a range with set max and min
float Utils::constrain(float val, float min, float max) {
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

// Utility function to round a value to a set number of decimal places
float Utils::roundToDecimalPlaces(double value, int decimalPlaces) {
    double scale = std::pow(10.0, decimalPlaces);
    return std::round(value * scale) / scale;
}

// Utility function to round a value to a set number of decimal places
float Utils::toFixedPoint(double value, int decimalPlaces) {
    double scale = std::pow(10.0, decimalPlaces);
    return std::round(value * scale);
}
