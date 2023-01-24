#include "safety.h"

Safety::Safety() : Safety(1.0) {
    
}

Safety::Safety(double safety_threshold) {
    speed = 0.0;
    this->safety_threshold = safety_threshold;
}

void Safety::UpdateSpeed(const double &x_dot, const double &y_dot, const double &z_dot) {

    // Calculate the norm of the velocity vector
    speed = pow(x_dot, 2.0) + pow(y_dot, 2.0) + pow(z_dot, 2.0);

    // Calculate the magnitude of the velocity vector
    speed = pow(speed, 0.5);
}

bool Safety::SafetyCheck(const std::vector<float> &ranges, const std::vector<float> angles) {
    
    bool should_stop = false;
    for(int i = 0; i < ranges.size() && !should_stop; i++) {
        double ttc = CalculateTTC(speed, ranges[i], angles[i]);

        // Make sure TTC did not blow up to infinity and check if below threshold
        if(!std::isinf(ttc) && ttc < safety_threshold) {
            should_stop = true;
        }
    }
    
    return should_stop;
}

double Safety::CalculateTTC(const double &speed, const double &distance, const double &angle) {

    // Projection of the velocity onto the distance
    double speed_projection = speed * cos(angle);
    speed_projection = std::max(speed_projection, 0.0);

    return distance / speed_projection;
}

double Safety::GetSpeed() {
    return speed;
}

std::vector<float> Safety::GenerateAngles(const float &angle_min, const float &angle_max, const int &count) {
    float angle;
    float angle_range;
    float bin_size;

    std::vector<float> angles(count);

    angle = angle_min;
    angle_range = angle_max - angle_min;
    bin_size = angle_range / count;

    for(int i = 0; i < count; i++) {
        angles[i] = angle;
        angle += bin_size;
    }

    return angles;
}