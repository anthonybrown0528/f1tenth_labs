#ifndef SENSOR_PROCESSOR_H
#define SENSOR_PROCESSOR_H

#include <string>
#include <vector>
#include <algorithm>

class SensorProcessor {
    public:

        /**
         * @brief Computes the largest and smallest data within the given vector of data
         * 
         * @param data collection of data points
         * @return a vector of size 2 where the first element
         *          is the nearest data point and the second is
         *          the farthest data point
         */
        static std::vector<float> ProcessSensor(const std::vector<float> &data);
};

#endif SENSOR_PROCESSOR_H