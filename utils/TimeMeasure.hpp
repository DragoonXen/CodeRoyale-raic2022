//
// Created by dragoon on 11.12.2020.
//
#ifdef TIMING_ENABLED

#include <chrono>

#endif

#ifndef AICUP2020_TIMEMEASURE_H
#define AICUP2020_TIMEMEASURE_H

namespace TimeMeasure {
#ifdef TIMING_ENABLED
    const int maxT = 11;
    static long timings[maxT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    static std::clock_t startTime;

    inline static void start() {
        startTime = std::clock();
    }

    inline static void end(int idx) {
        timings[idx] += (std::clock() - startTime);
        startTime = std::clock();
    }

    inline static void printTimings() {
        for (int i = 0; i != maxT; ++i) {
            std::cout << timings[i] << " ";
        }
        std::cout << std::endl;
    }

#else
    inline static void start() {}
    inline static void end(int idx) {}
    inline static void printTimings() {}
#endif

}

#endif //AICUP2020_TIMEMEASURE_H
