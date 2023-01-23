#include <sensor_processor.h>

std::vector<float> SensorProcessor::ProcessSensor(const std::vector<float> &data) {

    // Declare variables
    float farthest_data;
    float nearest_data;

    std::vector<float> range;

    // Calculate the minimum and maximum ranges
    farthest_data = *std::max_element(data.begin(), data.end());
    nearest_data = *std::min_element(data.begin(), data.end());

    range.push_back(nearest_data);
    range.push_back(farthest_data);

    return range;
}