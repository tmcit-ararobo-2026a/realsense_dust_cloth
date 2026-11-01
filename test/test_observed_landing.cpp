#include "realsense_dust_cloth/ballistic_tracker.hpp"
#include <cassert>
#include <iostream>
using namespace realsense_dust_cloth;

BallisticTracker flight(bool strict) {
    BallisticTracker b; b.observed_landing_only = strict;
    b.publish_cutoff_as_landing = false;
    b.min_cluster_points = 3; b.min_track_points_limit = 3;
    b.gate_radius_m = 3.0; b.tube_radius_m = 3.0; b.max_heading_deg = 90.0;
    b.launch_min_x = 0.5; b.launch_max_x = 1.1; b.launch_min_z = 1.0; b.launch_max_z = 2.0;
    b.landing_min_z = 0.15; b.max_lost_frames = 3;
    ClothTrackingResult p, out; p.valid = true; p.cluster_points_count = 10; p.cx = 1.0; p.cz = 1.5;
    assert(!b.update(p, 1.0, out)); p.cx = 1.2; p.cz = 1.55;
    assert(b.update(p, 1.05, out)); assert(b.launch_displacement_confirmed); return b;
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
    jitter_tracker.abort_on_anomaly = false;
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

    // In-flight optical noise jumping backward must be rejected
    ClothTrackingResult backward_noise;
    backward_noise.valid = true;
    backward_noise.cluster_points_count = 25;
    backward_noise.cx = 1.10; // Behind current position 1.16m (dx = -0.06m)
    backward_noise.cy = 0.0;
    backward_noise.cz = 1.75;
    assert(!jitter_tracker.update(backward_noise, 2.04, out));
    // Tracker should retain last known state and increment lost_count instead of jumping backward
    assert(jitter_tracker.lost_count == 1);
    assert(std::abs(jitter_tracker.x - 1.16) < 1e-4);

    // Real cloth continuing forward is matched smoothly
    real_cloth.cx = 1.46; real_cloth.cz = 1.83; // forward progress along ballistic trajectory (vx ~ 7.5 m/s)
    assert(jitter_tracker.update(real_cloth, 2.06, out));
    assert(jitter_tracker.state == TrackerState::TRACKING);
    assert(jitter_tracker.lost_count == 0);
    assert(std::abs(jitter_tracker.x - 1.46) < 1e-4);

    // 4. Physical Anomaly Rejection: Ascent after apex is strictly rejected
    BallisticTracker apex_tracker;
    apex_tracker.abort_on_anomaly = false;
    apex_tracker.min_cluster_points = 5;
    apex_tracker.launch_min_x = 0.8; apex_tracker.launch_max_x = 1.35;
    apex_tracker.launch_min_z = 1.45; apex_tracker.launch_max_z = 2.1;
    apex_tracker.min_launch_move_m = 0.10;
    apex_tracker.reject_ascent_after_apex = true;

    ClothTrackingResult c;
    c.valid = true; c.cluster_points_count = 20; c.cx = 1.0; c.cy = 0.0; c.cz = 1.70;
    assert(!apex_tracker.update(c, 3.00, out));
    c.cx = 1.15; c.cz = 1.705; // Launch: vx = 6.0 m/s, vz = 0.20 m/s
    assert(apex_tracker.update(c, 3.025, out));
    assert(apex_tracker.state == TrackerState::TRACKING);
    assert(!apex_tracker.has_reached_apex);

    // Reaching apex: vz <= 0
    c.cx = 1.30; c.cz = 1.702; // Peak: vz = -0.12 m/s, az = -12.8 m/s^2
    assert(apex_tracker.update(c, 3.050, out));
    assert(apex_tracker.has_reached_apex);

    // Descending after apex
    c.cx = 1.45; c.cz = 1.690; // Falling: vz = -0.48 m/s, az = -14.4 m/s^2
    assert(apex_tracker.update(c, 3.075, out));
    assert(apex_tracker.has_reached_apex);
    double z_descending = apex_tracker.z;

    // False upward jump (noise above cloth) must be rejected
    ClothTrackingResult upward_noise;
    upward_noise.valid = true; upward_noise.cluster_points_count = 20;
    upward_noise.cx = 1.55; upward_noise.cy = 0.0;
    upward_noise.cz = z_descending + 0.08; // Jumps up (exceeds vertical jitter tolerance 0.03m)
    assert(!apex_tracker.update(upward_noise, 3.100, out));
    assert(apex_tracker.lost_count == 1);

    // 5. Physical Anomaly Rejection: Impossible downward acceleration (nosedive kidnapping)
    BallisticTracker nosedive_tracker;
    nosedive_tracker.abort_on_anomaly = false;
    nosedive_tracker.min_cluster_points = 5;
    nosedive_tracker.launch_min_x = 0.8; nosedive_tracker.launch_max_x = 1.35;
    nosedive_tracker.launch_min_z = 1.45; nosedive_tracker.launch_max_z = 2.1;
    nosedive_tracker.min_launch_move_m = 0.10;
    nosedive_tracker.min_vertical_accel_m_s2 = -20.0;

    c.cx = 1.0; c.cy = 0.0; c.cz = 1.60;
    nosedive_tracker.update(c, 4.00, out);
    c.cx = 1.15; c.cz = 1.60;
    nosedive_tracker.update(c, 4.025, out); // vx = 6.0 m/s, vz = 0 m/s

    // Noise cluster on desk (e.g. desk_4_b at z = 0.70m, plunge dz = -0.90m -> az < -20.0 m/s^2)
    ClothTrackingResult desk_noise;
    desk_noise.valid = true; desk_noise.cluster_points_count = 20;
    desk_noise.cx = 1.30; desk_noise.cy = 0.0; desk_noise.cz = 0.70;
    assert(!nosedive_tracker.update(desk_noise, 4.050, out));
    assert(nosedive_tracker.lost_count == 1);
    assert(nosedive_tracker.state == TrackerState::TRACKING); // Not kidnapped into LANDED!

    // 6. Physical Anomaly Rejection: Sharp lateral deflection (> 45 deg)
    BallisticTracker heading_tracker;
    heading_tracker.abort_on_anomaly = false;
    heading_tracker.min_cluster_points = 5;
    heading_tracker.launch_min_x = 0.8; heading_tracker.launch_max_x = 1.35;
    heading_tracker.launch_min_z = 1.45; heading_tracker.launch_max_z = 2.1;
    heading_tracker.min_launch_move_m = 0.10;
    heading_tracker.max_heading_deg = 45.0;

    c.cx = 1.0; c.cy = 0.0; c.cz = 1.60;
    heading_tracker.update(c, 5.00, out);
    c.cx = 1.15; c.cy = 0.0; c.cz = 1.60;
    heading_tracker.update(c, 5.025, out); // Flying along +X: vx = 6.0 m/s, vy = 0 m/s

    // Severe sideways jump (e.g. human obstacle at dy = 0.20m, dx = 0.05m -> heading ~ 76 deg)
    ClothTrackingResult side_noise;
    side_noise.valid = true; side_noise.cluster_points_count = 20;
    side_noise.cx = 1.20; side_noise.cy = 0.20; side_noise.cz = 1.60;
    assert(!heading_tracker.update(side_noise, 5.050, out));
    assert(heading_tracker.lost_count == 1);

    // 7. Coast Tracking: Smooth ballistic TF interpolation during temporary occlusion
    BallisticTracker coast_tracker;
    coast_tracker.publish_cutoff_as_landing = false;
    coast_tracker.min_cluster_points = 5;
    coast_tracker.launch_min_x = 0.8; coast_tracker.launch_max_x = 1.35;
    coast_tracker.launch_min_z = 1.45; coast_tracker.launch_max_z = 2.1;
    coast_tracker.min_launch_move_m = 0.10;
    coast_tracker.enable_coast_prediction_tf = true;
    coast_tracker.tube_radius_m = 0.35;

    c.cx = 1.0; c.cy = 0.0; c.cz = 1.60;
    coast_tracker.update(c, 6.00, out);
    c.cx = 1.15; c.cy = 0.0; c.cz = 1.60;
    coast_tracker.update(c, 6.025, out); // vx = 6.0 m/s, vz = 0 m/s

    // Temporary occlusion for 2 frames (no detections)
    ClothTrackingResult empty_result;
    empty_result.valid = false;
    assert(coast_tracker.update(empty_result, 6.050, out)); // Coast frame 1
    assert(coast_tracker.state == TrackerState::TRACKING);
    assert(out.valid);
    assert(out.cx > 1.15); // Continues forward
    assert(coast_tracker.lost_count == 1);

    assert(coast_tracker.update(empty_result, 6.075, out)); // Coast frame 2
    assert(coast_tracker.state == TrackerState::TRACKING);
    assert(out.cx > 1.25);
    assert(coast_tracker.lost_count == 2);

    // Observation reappears along coasted trajectory
    float px = 0.0f, py = 0.0f, pz = 0.0f;
    coast_tracker.get_prediction(6.100, px, py, pz);
    ClothTrackingResult resumed;
    resumed.valid = true; resumed.cluster_points_count = 15;
    resumed.cx = px; resumed.cy = py; resumed.cz = pz;
    assert(coast_tracker.update(resumed, 6.100, out));
    assert(coast_tracker.state == TrackerState::TRACKING);
    assert(coast_tracker.lost_count == 0); // Reset lost counter on match

    // 8. Early Tracking Cutoff on Anomaly (abort_on_anomaly = true, publish_cutoff_as_landing = true)
    BallisticTracker abort_tracker;
    abort_tracker.abort_on_anomaly = true;
    abort_tracker.max_anomaly_frames = 1;
    abort_tracker.publish_cutoff_as_landing = true;
    abort_tracker.min_cluster_points = 5;
    abort_tracker.launch_min_x = 0.8; abort_tracker.launch_max_x = 1.35;
    abort_tracker.launch_min_z = 1.45; abort_tracker.launch_max_z = 2.1;
    abort_tracker.min_launch_move_m = 0.10;

    c.cx = 1.0; c.cy = 0.0; c.cz = 1.60;
    abort_tracker.update(c, 7.00, out);
    c.cx = 1.15; c.cy = 0.0; c.cz = 1.60;
    abort_tracker.update(c, 7.025, out);
    assert(abort_tracker.state == TrackerState::TRACKING);

    // Sudden nosedive anomaly into desk (az < -20.0 m/s^2)
    ClothTrackingResult anomaly_cluster;
    anomaly_cluster.valid = true; anomaly_cluster.cluster_points_count = 20;
    anomaly_cluster.cx = 1.30; anomaly_cluster.cy = 0.0; anomaly_cluster.cz = 0.70;
    // Anomaly causes immediate tracking cutoff!
    assert(!abort_tracker.update(anomaly_cluster, 7.050, out));
    assert(abort_tracker.state == TrackerState::LANDED); // Cutoff position declared final
    assert(std::abs(abort_tracker.x - 1.15) < 1e-4);
    assert(std::abs(abort_tracker.z - 1.60) < 1e-4); // Frozen at last valid mid-air position!

    // 9. Lateral Straightness & Lateral Accel Rejection (max_abs_vy_m_s = 0.80, max_lateral_accel_m_s2 = 8.0)
    BallisticTracker straight_tracker;
    straight_tracker.abort_on_anomaly = false;
    straight_tracker.min_cluster_points = 5;
    straight_tracker.launch_min_x = 0.8; straight_tracker.launch_max_x = 1.35;
    straight_tracker.launch_min_z = 1.45; straight_tracker.launch_max_z = 2.1;
    straight_tracker.min_launch_move_m = 0.10;
    straight_tracker.max_abs_vy_m_s = 0.80;
    straight_tracker.max_lateral_accel_m_s2 = 8.0;

    c.cx = 1.0; c.cy = 0.0; c.cz = 1.60;
    straight_tracker.update(c, 8.00, out);
    c.cx = 1.15; c.cy = 0.0; c.cz = 1.60;
    straight_tracker.update(c, 8.025, out); // vy = 0

    // Lateral swerve: dy = 0.05m in 0.025s -> vy = 2.0 m/s > 0.80 m/s, ay = 80 m/s^2 > 8.0 m/s^2
    ClothTrackingResult swerve_noise;
    swerve_noise.valid = true; swerve_noise.cluster_points_count = 20;
    swerve_noise.cx = 1.30; swerve_noise.cy = 0.05; swerve_noise.cz = 1.59;
    assert(!straight_tracker.update(swerve_noise, 8.050, out));
    assert(straight_tracker.lost_count == 1);

    // 10. Early Tracking Cutoff on Loss (max_lost_frames = 4, publish_cutoff_as_landing = true)
    BallisticTracker loss_cutoff_tracker;
    loss_cutoff_tracker.min_cluster_points = 5;
    loss_cutoff_tracker.launch_min_x = 0.8; loss_cutoff_tracker.launch_max_x = 1.35;
    loss_cutoff_tracker.launch_min_z = 1.45; loss_cutoff_tracker.launch_max_z = 2.1;
    loss_cutoff_tracker.min_launch_move_m = 0.10;
    loss_cutoff_tracker.max_lost_frames = 4;
    loss_cutoff_tracker.enable_coast_prediction_tf = false;
    loss_cutoff_tracker.publish_cutoff_as_landing = true;

    c.cx = 1.0; c.cy = 0.0; c.cz = 1.60;
    loss_cutoff_tracker.update(c, 9.00, out);
    c.cx = 1.15; c.cy = 0.0; c.cz = 1.60;
    loss_cutoff_tracker.update(c, 9.025, out);

    // Missing for 5 frames
    ClothTrackingResult missing_frame;
    missing_frame.valid = false;
    for (int f = 1; f <= 4; ++f) {
        assert(!loss_cutoff_tracker.update(missing_frame, 9.025 + f * 0.025, out));
        assert(loss_cutoff_tracker.state == TrackerState::TRACKING);
    }
    // 5th frame exceeds max_lost_frames (4) -> terminates immediately!
    assert(!loss_cutoff_tracker.update(missing_frame, 9.025 + 5 * 0.025, out));
    assert(loss_cutoff_tracker.state == TrackerState::LANDED);
    assert(std::abs(loss_cutoff_tracker.x - 1.15) < 1e-4);
    assert(std::abs(loss_cutoff_tracker.z - 1.60) < 1e-4);

    std::cout << "Map mode distinguishes an observed landing from loss; legacy default preserved\n";
    std::cout << "Optical jitter noise in launch gate rejected; genuine launch verified\n";
    std::cout << "In-flight backward optical noise rejected; monotonic forward tracking verified\n";
    std::cout << "Ascent rejection after apex verified\n";
    std::cout << "Impossible downward acceleration (nosedive kidnapping) rejected\n";
    std::cout << "Sharp heading deflection rejected\n";
    std::cout << "Coast tracking and ballistic TF continuity verified\n";
    std::cout << "Early tracking cutoff on anomaly verified (frozen at last good position)\n";
    std::cout << "Lateral straightness and lateral acceleration limit verified\n";
    std::cout << "Early tracking cutoff on point cloud loss verified\n";
}
