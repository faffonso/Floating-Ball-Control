#include "MovingAverageFilter.hpp"

MovingAverageFilter::MovingAverageFilter(int size) {
    windowSize = size;
    window = new float[windowSize];
    
    for (int i = 0; i < windowSize; ++i) 
        window[i] = 0.0;
    
    currentIndex = 0;
    sum = 0.0;
}

MovingAverageFilter::~MovingAverageFilter() {
    delete[] window;
}

float MovingAverageFilter::addData(float newData) {
    sum -= window[currentIndex];
    
    window[currentIndex] = newData;
    sum += newData;
    
    currentIndex = (currentIndex + 1) % windowSize;
    return getFilteredValue();
}

float MovingAverageFilter::getFilteredValue() {
    return sum / windowSize;
}
