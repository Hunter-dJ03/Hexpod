#ifndef UTILS_H
#define UTILS_H

class Utils
{
public:
    static float constrain(float val, float min, float max);
    static float roundToDecimalPlaces(double value, int decimalPlaces);
    static float toFixedPoint(double value, int decimalPlaces);
};

#endif
