#include "realsense_dust_cloth/ballistic_tracker.hpp"

namespace realsense_dust_cloth {

namespace {
constexpr double DEFAULT_FRAME_DT_SEC = 0.016;
constexpr double MIN_VELOCITY_DT_SEC = 0.025;
constexpr double DEFAULT_LAUNCH_VX_M_S = 2.0;
constexpr double LOST_FRAME_GATE_EXPANSION_M = 0.08;
constexpr double MAX_GATE_EXPANSION_M = 0.35;
constexpr double HALF_GRAVITY_FACTOR = 0.5;
} // namespace

void BallisticTracker::reset() {
    state = TrackerState::IDLE;
    tracked_count = 0;
    lost_count = 0;
    stationary_count = 0;
    launch_displacement_confirmed = false;
    last_stamp = 0.0;
    launch_stamp = 0.0;
}

bool BallisticTracker::get_prediction(double stamp_sec, float& px, float& py, float& pz) const {
    if (state != TrackerState::TRACKING || last_stamp <= 0.0) {
        return false;
    }
    double dt = stamp_sec - last_stamp;
    if (dt <= 0.0 || (max_flight_duration_sec > 0.0 && dt > max_flight_duration_sec)) {
        dt = DEFAULT_FRAME_DT_SEC;
    }
    double drag_factor = std::clamp(1.0 - drag_coeff * dt, 0.0, 1.0);
    px = static_cast<float>(x + vx * dt * drag_factor);
    py = static_cast<float>(y + vy * dt * drag_factor);
    pz = static_cast<float>(z + vz * dt - HALF_GRAVITY_FACTOR * gravity_m_s2 * dt * dt);
    return true;
}

bool BallisticTracker::update(
    const ClothTrackingResult& raw, double stamp_sec, ClothTrackingResult& out)
{
    // =========================================================================
    // Phase A: IDLE / LANDED -> Launch Candidate Acquisition
    // =========================================================================
    if (state == TrackerState::IDLE || state == TrackerState::LANDED) {
        if (!raw.valid) {
            return false;
        }

        // Candidate must be inside the configured launch gate with sufficient points
        if (raw.cx >= launch_min_x && raw.cx <= launch_max_x &&
            raw.cy >= launch_min_y && raw.cy <= launch_max_y &&
            raw.cz >= launch_min_z && raw.cz <= launch_max_z &&
            raw.cluster_points_count >= min_cluster_points)
        {
            state = TrackerState::TRACKING;
            x = raw.cx;
            y = raw.cy;
            z = raw.cz;
            launch_x = raw.cx;
            launch_y = raw.cy;
            launch_z = raw.cz;
            launch_stamp = stamp_sec;
            stationary_ref_x = raw.cx;
            stationary_ref_y = raw.cy;
            stationary_ref_z = raw.cz;
            launch_displacement_confirmed = false;
            vx = std::max(min_launch_vx, DEFAULT_LAUNCH_VX_M_S);
            vy = 0.0;
            vz = 0.0;
            last_stamp = stamp_sec;
            tracked_count = 0;
            lost_count = 0;
            stationary_count = 0;
            return false;
        } else {
            return false;
        }
    }

    // =========================================================================
    // Phase B: TRACKING -> In-flight Ballistic Tracking & Drag Compensation
    // =========================================================================
    if (max_flight_duration_sec > 0.0 && (stamp_sec - launch_stamp) > max_flight_duration_sec) {
        if (launch_displacement_confirmed && x > launch_max_x) {
            state = TrackerState::LANDED;
            lost_count = 0;
            stationary_count = 0;
        } else {
            reset();
        }
        return false;
    }

    float pred_xf = 0.0f, pred_yf = 0.0f, pred_zf = 0.0f;
    get_prediction(stamp_sec, pred_xf, pred_yf, pred_zf);
    double pred_x = static_cast<double>(pred_xf);
    double pred_y = static_cast<double>(pred_yf);
    double pred_z = static_cast<double>(pred_zf);

    bool matched = false;
    int min_track_pts = std::min(min_cluster_points, min_track_points_limit);

    if (raw.valid && raw.cluster_points_count >= min_track_pts) {
        double dx_pred = raw.cx - pred_x;
        double dy_pred = raw.cy - pred_y;
        double dz_pred = raw.cz - pred_z;
        double dist_pred = std::sqrt(dx_pred * dx_pred + dy_pred * dy_pred + dz_pred * dz_pred);

        double dx_prev = raw.cx - x;
        double dy_prev = raw.cy - y;
        double dz_prev = raw.cz - z;
        double dist_prev = std::sqrt(dx_prev * dx_prev + dy_prev * dy_prev + dz_prev * dz_prev);

        double dt_raw = stamp_sec - last_stamp;
        double dt_safe = (dt_raw > 0.0) ? dt_raw : launch_timeout_sec;
        double dt_vel = std::max(dt_safe, MIN_VELOCITY_DT_SEC);
        double new_vx = dx_prev / dt_vel;
        double new_vy = dy_prev / dt_vel;
        double new_vz = dz_prev / dt_vel;

        // A projectile in free forward flight must remain within the 3D ballistic gate
        // of both the predicted position and the last measured position (prevents free-fall
        // prediction during occlusion after hooking on a pole from teleporting 1.5m onto a human below)
        double effective_gate = gate_radius_m + std::min(lost_count * LOST_FRAME_GATE_EXPANSION_M, MAX_GATE_EXPANSION_M);
        double max_step_dist = effective_gate + max_vx_m_s * dt_safe;
        if (dist_pred <= effective_gate &&
            dist_prev <= max_step_dist &&
            new_vx >= min_flight_vx_m_s && new_vx <= max_vx_m_s &&
            std::abs(new_vy) <= max_abs_vy_m_s &&
            new_vz >= min_vz_m_s && new_vz <= max_vz_m_s)
        {
            // Confirm forward upward/level displacement from launch point before locking on
            if (!launch_displacement_confirmed) {
                double dx_from_launch = raw.cx - launch_x;
                double dz_from_launch = raw.cz - launch_z;
                double dt_launch = stamp_sec - launch_stamp;
                double dt_launch_safe = (dt_launch > 0.0) ? dt_launch : dt_safe;
                double launch_vx = dx_from_launch / dt_launch_safe;

                if (dx_from_launch >= min_launch_move_m &&
                    launch_vx >= min_launch_vx &&
                    dz_from_launch >= -dx_from_launch)
                {
                    launch_displacement_confirmed = true;
                    matched = true;
                    vx = std::clamp(new_vx, min_flight_vx_m_s, max_vx_m_s);
                    vy = std::clamp(new_vy, -max_abs_vy_m_s, max_abs_vy_m_s);
                    vz = std::clamp(new_vz, min_vz_m_s, max_vz_m_s);
                    stationary_ref_x = raw.cx;
                    stationary_ref_y = raw.cy;
                    stationary_ref_z = raw.cz;
                    stationary_count = 0;
                    x = raw.cx;
                    y = raw.cy;
                    z = raw.cz;
                    last_stamp = stamp_sec;
                    lost_count = 0;
                    tracked_count = 1;
                    out = raw;
                    return true;
                }
            } else {
                matched = true;

                // Detect mid-air or pole-hooked resting state
                double ref_dx = raw.cx - stationary_ref_x;
                double ref_dy = raw.cy - stationary_ref_y;
                double ref_dz = raw.cz - stationary_ref_z;
                double dist_from_ref = std::sqrt(ref_dx * ref_dx + ref_dy * ref_dy + ref_dz * ref_dz);
                if (dist_from_ref < min_stationary_move_m) {
                    stationary_count++;
                    if (max_stationary_frames > 0 && stationary_count >= max_stationary_frames) {
                        if (x > launch_max_x) {
                            state = TrackerState::LANDED;
                            lost_count = 0;
                            stationary_count = 0;
                        } else {
                            reset();
                        }
                        return false;
                    }
                } else {
                    stationary_ref_x = raw.cx;
                    stationary_ref_y = raw.cy;
                    stationary_ref_z = raw.cz;
                    stationary_count = 0;
                }

                double drag_factor = std::clamp(1.0 - drag_coeff * dt_safe, 0.0, 1.0);
                double pred_vx = vx * drag_factor;
                double pred_vy = vy * drag_factor;
                double pred_vz = vz - gravity_m_s2 * dt_safe;

                vx = std::clamp((1.0 - vel_smoothing_beta) * pred_vx + vel_smoothing_beta * new_vx, min_flight_vx_m_s, max_vx_m_s);
                vy = std::clamp((1.0 - vel_smoothing_beta) * pred_vy + vel_smoothing_beta * new_vy, -max_abs_vy_m_s, max_abs_vy_m_s);
                vz = std::clamp((1.0 - vel_smoothing_beta) * pred_vz + vel_smoothing_beta * new_vz, min_vz_m_s, max_vz_m_s);

                // Strictly use the actual measured cloth cluster position (never blend predicted coordinates into TF)
                x = raw.cx;
                y = raw.cy;
                z = raw.cz;

                last_stamp = stamp_sec;
                lost_count = 0;
                tracked_count++;
                out = raw;

                // Stop tracking when cloth touches the floor plane (landing_min_z)
                if (z <= landing_min_z) {
                    state = TrackerState::LANDED;
                    lost_count = 0;
                    stationary_count = 0;
                }
                return true;
            }
        }
    }

    // =========================================================================
    // Phase C: Observation Missed / Lost Frame Handling
    // =========================================================================
    if (!matched) {
        if (!launch_displacement_confirmed) {
            if (raw.valid &&
                raw.cx >= launch_min_x && raw.cx <= launch_max_x &&
                raw.cy >= launch_min_y && raw.cy <= launch_max_y &&
                raw.cz >= launch_min_z && raw.cz <= launch_max_z &&
                raw.cluster_points_count >= min_cluster_points)
            {
                x = raw.cx;
                y = raw.cy;
                z = raw.cz;
                launch_x = raw.cx;
                launch_y = raw.cy;
                launch_z = raw.cz;
                launch_stamp = stamp_sec;
                stationary_ref_x = raw.cx;
                stationary_ref_y = raw.cy;
                stationary_ref_z = raw.cz;
                vx = std::max(min_launch_vx, DEFAULT_LAUNCH_VX_M_S);
                vy = 0.0;
                vz = 0.0;
                last_stamp = stamp_sec;
                return false;
            }
            if ((stamp_sec - launch_stamp) > launch_timeout_sec) {
                reset();
            }
            return false;
        }
        lost_count++;
        if (lost_count > max_lost_frames || pred_z <= landing_min_z) {
            state = TrackerState::LANDED;
            lost_count = 0;
            stationary_count = 0;
            return false;
        }
        return false;
    }

    return false;
}

} // namespace realsense_dust_cloth
