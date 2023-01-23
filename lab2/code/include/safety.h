#ifndef SAFETY_H
#define SAFETY_H

#include <cmath>
#include <vector>

class Safety {
// The class that handles emergency braking
private:
    double speed;
    double safety_threshold;

    double CalculateTTC(const double &speed, const double &distance, const double &angle);
public:
    Safety();
    Safety(double safety_threshold);

    void UpdateSpeed(const double &x_dot, const double &y_dot, const double &z_dot);

    bool SafetyCheck(const std::vector<float> &ranges, const std::vector<float> angles);

    double GetSpeed();

    std::vector<float> GenerateAngles(const float &angle_min, const float &angle_max, const int &count);
};

#endif // SAFETY_H