#pragma once

#include <chrono>

// The single shared "now" for every node that stamps a PosePacket.

inline double nowSeconds()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}
