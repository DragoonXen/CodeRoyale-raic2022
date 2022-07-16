//
// Created by dragoon on 11.12.2020.
//
#ifdef TIMING_ENABLED

#include <chrono>
#include <array>

#endif

#ifndef AICUP2020_TIMEMEASURE_H
#define AICUP2020_TIMEMEASURE_H

namespace TimeMeasure {
#ifdef TIMING_ENABLED
    const int kMaxT = 11;
    static std::array<long long, kMaxT> timings = []() {
        std::array<long long, kMaxT> res = {};
        for (size_t i = 0; i != kMaxT; ++i) {
            res[i] = 0;
        }
        return res;
    }();

    static std::chrono::time_point<std::chrono::high_resolution_clock> startTime;

    inline static void start() {
        startTime = std::chrono::high_resolution_clock::now();
    }

    inline static void end(int idx) {
        timings[idx] += std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - startTime).count();
        startTime = std::chrono::high_resolution_clock::now();
    }

    inline static void printTimings() {
        std::stringstream sstr;
        for (int i = 0; i != kMaxT; ++i) {
            sstr << timings[i] << " ";
        }
        std::cerr << sstr.str() << std::endl;
    }

#else
    inline static void start() {}
    inline static void end(int idx) {}
    inline static void printTimings() {}
#endif

}

#endif //AICUP2020_TIMEMEASURE_H
