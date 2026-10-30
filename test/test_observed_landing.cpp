#include "realsense_dust_cloth/ballistic_tracker.hpp"
#include <cassert>
#include <iostream>
using namespace realsense_dust_cloth;

BallisticTracker flight(bool strict) {
    BallisticTracker b;b.observed_landing_only=strict;
    b.min_cluster_points=3;b.min_track_points_limit=3;b.gate_radius_m=3;
    b.launch_min_x=.5;b.launch_max_x=1.1;b.launch_min_z=1.;b.launch_max_z=2.;
    b.landing_min_z=.15;b.max_lost_frames=3;
    ClothTrackingResult p,out;p.valid=true;p.cluster_points_count=10;p.cx=1.;p.cz=1.5;
    assert(!b.update(p,1.0,out));p.cx=1.2;p.cz=1.55;
    assert(b.update(p,1.05,out));assert(b.launch_displacement_confirmed);return b;
}
int main() {
    ClothTrackingResult missing,out;
    auto strict=flight(true);
    for(int i=1;i<=4;++i) strict.update(missing,1.05+i*.05,out);
    assert(strict.state==TrackerState::LOST);
    auto legacy=flight(false);
    for(int i=1;i<=4;++i) legacy.update(missing,1.05+i*.05,out);
    assert(legacy.state==TrackerState::LANDED);
    auto observed=flight(true);
    ClothTrackingResult low;low.valid=true;low.cluster_points_count=10;low.cx=2.;low.cz=.1;
    assert(observed.update(low,1.45,out));assert(observed.state==TrackerState::LANDED);
    assert(out.valid && out.cz<=observed.landing_min_z);

    // Verify optical jitter noise rejection in launch gate
    BallisticTracker jitter_tracker;
    jitter_tracker.min_cluster_points = 5;
    jitter_tracker.launch_min_x = 0.8; jitter_tracker.launch_max_x = 1.35;
    jitter_tracker.launch_min_z = 1.45; jitter_tracker.launch_max_z = 2.1;
    jitter_tracker.min_launch_move_m = 0.12;
    jitter_tracker.min_launch_vx = 0.50;
    jitter_tracker.launch_timeout_sec = 0.20;

    ClothTrackingResult noise;
    noise.valid = true;
    noise.cluster_points_count = 20;
    for (int frame = 0; frame < 50; ++frame) {
        double t = 1.0 + frame * 0.016;
        noise.cx = 1.0 + (frame % 2 == 0 ? 0.02 : -0.01);
        noise.cy = 0.0;
        noise.cz = 1.7;
        assert(!jitter_tracker.update(noise, t, out));
        assert(jitter_tracker.state == TrackerState::IDLE);
        assert(!jitter_tracker.launch_displacement_confirmed);
    }

    // Genuine launch from gate immediately promotes to TRACKING
    ClothTrackingResult real_cloth;
    real_cloth.valid = true;
    real_cloth.cluster_points_count = 20;
    real_cloth.cx = 1.0; real_cloth.cy = 0.0; real_cloth.cz = 1.7;
    assert(!jitter_tracker.update(real_cloth, 2.0, out));
    real_cloth.cx = 1.16; real_cloth.cz = 1.75; // dx = 0.16m, vx = 8.0 m/s
    assert(jitter_tracker.update(real_cloth, 2.02, out));
    assert(jitter_tracker.state == TrackerState::TRACKING);
    assert(jitter_tracker.launch_displacement_confirmed);

    std::cout << "Map mode distinguishes an observed landing from loss; legacy default preserved\n";
    std::cout << "Optical jitter noise in launch gate rejected; genuine launch verified\n";
}
