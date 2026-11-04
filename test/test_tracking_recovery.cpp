#include "realsense_dust_cloth/ballistic_tracker.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace realsense_dust_cloth;

ClothTrackingResult observation(float x, float y, float z) {
    ClothTrackingResult result;
    result.valid = true;
    result.cluster_points_count = 100;
    result.cx = x;
    result.cy = y;
    result.cz = z;
    return result;
}

BallisticTracker configured_tracker() {
    BallisticTracker tracker;
    tracker.launch_min_x = .8;
    tracker.launch_max_x = 2;
    tracker.launch_min_z = 1.45;
    tracker.launch_max_z = 2.5;
    tracker.min_launch_move_m = .12;
    tracker.min_launch_vx = 3;
    tracker.max_anomaly_frames = 3;
    tracker.max_lost_frames = 100;
    return tracker;
}

void launch(BallisticTracker& tracker, double stamp) {
    ClothTrackingResult output;
    assert(!tracker.update(observation(1, 0, 1.6), stamp, output));
    assert(tracker.update(observation(1.2, 0, 1.65), stamp + .05, output));
    assert(tracker.state == TrackerState::TRACKING);
}

void test_new_flight_has_its_own_anomaly_budget() {
    auto tracker = configured_tracker();
    ClothTrackingResult output;
    launch(tracker, 1);
    for (int i = 1; i <= 3; ++i) {
        assert(!tracker.update(observation(1.2 + i * .1, .7, 1.7), 1.05 + i * .03, output));
    }
    assert(tracker.state == TrackerState::LOST);
    assert(tracker.anomaly_count == 3);

    launch(tracker, 2);
    assert(tracker.anomaly_count == 0);
    assert(tracker.last_rejection_reason.empty());
    assert(!tracker.update(observation(1.3, .7, 1.7), 2.08, output));
    assert(tracker.state == TrackerState::TRACKING);
    assert(tracker.anomaly_count == 1);
}

void test_search_radius_expands_and_recovers() {
    auto tracker = configured_tracker();
    const double base_radius = tracker.tube_radius_m;
    auto near = [](double actual, double expected) { return std::abs(actual - expected) < 1e-12; };
    assert(near(tracker.search_radius_m(), base_radius));
    launch(tracker, 10);

    ClothTrackingResult output, missing;
    assert(!tracker.update(missing, 10.08, output));
    assert(near(tracker.search_radius_m(), base_radius + tracker.lost_frame_gate_expansion_m));
    assert(!tracker.update(missing, 10.11, output));
    assert(near(tracker.search_radius_m(), base_radius + 2 * tracker.lost_frame_gate_expansion_m));

    float x, y, z;
    assert(tracker.get_prediction(10.14, x, y, z));
    assert(tracker.update(observation(x, y, z), 10.14, output));
    assert(near(tracker.search_radius_m(), base_radius));

    for (int i = 1; i <= 10; ++i) {
        assert(!tracker.update(missing, 10.14 + i * .03, output));
    }
    assert(near(tracker.search_radius_m(), base_radius + tracker.max_gate_expansion_m));
    tracker.reset();
    assert(near(tracker.search_radius_m(), base_radius));
    assert(tracker.last_rejection_reason.empty());
}

int main() {
    test_new_flight_has_its_own_anomaly_budget();
    test_search_radius_expands_and_recovers();
    std::cout << "New-flight anomaly budget and loss/reacquisition radius passed\n";
}
