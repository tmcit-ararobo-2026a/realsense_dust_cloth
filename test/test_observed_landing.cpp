#include "realsense_dust_cloth/ballistic_tracker.hpp"
#include <cassert>
#include <iostream>
using namespace realsense_dust_cloth;

BallisticTracker configured() {
    BallisticTracker tracker;
    tracker.launch_min_x = 0.8; tracker.launch_max_x = 1.35;
    tracker.launch_min_z = 1.45; tracker.launch_max_z = 2.1;
    tracker.min_cluster_points = 35; tracker.min_track_points_limit = 8;
    tracker.min_launch_move_m = .12; tracker.min_launch_vx = .5;
    tracker.tube_radius_m = .35; tracker.max_total_accel_m_s2 = 25;
    tracker.max_lost_frames = 4; tracker.max_anomaly_frames = 1;
    tracker.max_abs_vy_m_s = .8; tracker.max_vz_m_s = 12;
    tracker.landing_min_z = .15; tracker.vel_smoothing_beta = .5;
    return tracker;
}

int main() {
    ClothTrackingResult point, result;
    point.valid = true; point.cluster_points_count = 100;
    // The same noiseless physical flight must work independently of camera FPS.
    for (bool noisy : {false, true}) for (int fps : {30, 60, 120}) {
        auto tracker = configured();
        double t = 1, x = 1, z = 1.6, vx = 6, vz = 2.5, dt = 1.0/fps;
        bool captured = false, landed = false;
        for (int frame = 0; frame < fps*3; ++frame) {
            point.cx = x + (noisy ? .01*std::sin(frame*1.7) : 0);
            point.cy = noisy ? .01*std::sin(frame*2.1) : 0;
            point.cz = z + (noisy ? .01*std::cos(frame*1.3) : 0);
            const bool matched = tracker.update(point, t, result);
            captured |= tracker.state == TrackerState::TRACKING;
            assert(tracker.state != TrackerState::LOST);
            if (tracker.state == TrackerState::LANDED) {
                assert(matched && result.cz <= tracker.landing_min_z);
                landed = true; break;
            }
            vx *= 1-tracker.drag_coeff*dt;
            vz -= tracker.gravity_m_s2*dt;
            x += vx*dt; z += vz*dt; t += dt;
        }
        assert(captured && landed);
    }
    auto tracker = configured();
    point.cx=1;point.cy=0;point.cz=1.6;
    assert(!tracker.update(point,1,result));
    point.cx=1.15;point.cz=1.65;
    assert(tracker.update(point,1.025,result));
    // An impossible jump must end as LOST, never as an observed landing.
    point.cx=1.30;point.cz=.7;
    assert(!tracker.update(point,1.05,result));
    assert(tracker.state==TrackerState::LOST);
    tracker = configured();
    point.cx=1;point.cz=1.6;tracker.update(point,2,result);
    point.cx=1.15;point.cz=1.65;tracker.update(point,2.025,result);
    ClothTrackingResult missing;
    for (int i=1;i<=5;++i) assert(!tracker.update(missing,2.025+i*.025,result));
    assert(tracker.state==TrackerState::LOST);
    std::cout << "30/60/120 FPS flights, measured landing, anomaly/loss separation passed\n";
}
