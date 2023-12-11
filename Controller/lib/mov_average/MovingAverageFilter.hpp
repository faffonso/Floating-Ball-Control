#ifndef MOVING_AVERAGE_FILTER_HPP
#define MOVING_AVERAGE_FILTER_HPP

/**
 * @file MovingAverageFilter.hpp
 * @brief Implementation of the actuators driver
 *
 * This files contain a moving average filter implementations to 
 * filter a ultrasonic sensor values to define a control setpoint.
 *
 * Authors: Francisco Affonso Pinto
 *          
 */

/**
 * @brief Class of the moving avarege filter
 * 
 */
class MovingAverageFilter {
    private:
        float *window;      // Vector of window values
        int windowSize;     // Size of this windows
        int currentIndex;   // Reading index
        float sum;          // Sum of window values

    public:
        /**
         * @brief Construct a new Moving Average Filter object
         * 
         * @param size 
         */
        MovingAverageFilter(int size);

        /**
         * @brief Destroy the Moving Average Filter object
         * 
         */
        ~MovingAverageFilter();

        /**
         * @brief Add new value for window
         * 
         * @param newData New Value
         * @return float Data filtered
         */
        float addData(float newData);

        /**
         * @brief Get the Filtered Value object
         * 
         * @return float
         */
        float getFilteredValue();
};

#endif // MOVING_AVERAGE_FILTER_HPP
