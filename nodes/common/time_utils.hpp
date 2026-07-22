#pragma once

#include <chrono>

// The single shared "now" for every node that stamps a PosePacket.
//
// transform_sync pairs an OptiTrack sample with a FANUC sample by
// |t_opti - t_fanuc| <= max_match_gap_sec (20 ms) -- which is only meaningful
// if BOTH streams stamp from the same clock. Stamping one side from Motive's
// fTimestamp (seconds since Motive started) and the other from this machine's
// steady_clock puts them hundreds of thousands of seconds apart, so the match
// never succeeds and pose_camera silently never reaches the frontend.
//
// steady_clock: monotonic, so NTP/wall-clock adjustments mid-session can't
// break the matching either.
inline double nowSeconds()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}
