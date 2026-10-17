#include "realsense_dust_cloth/ballistic_tracker.hpp"

namespace realsense_dust_cloth {

void BallisticTracker::reset() {
    state = TrackerState::IDLE;
    tracked_count = 0;
    lost_count = 0;
    stationary_count = 0;
    has_pending = false;
    launch_displacement_confirmed = false;
    last_stamp = 0.0;
    pending_stamp = 0.0;
    launch_stamp = 0.0;
}

bool BallisticTracker::get_prediction(double stamp_sec, float& px, float& py, float& pz) const {
    if (state != TrackerState::TRACKING || last_stamp <= 0.0) {
        return false;
    }
    double dt = stamp_sec - last_stamp;
    if (dt <= 0.0 || (max_flight_duration_sec > 0.0 && dt > max_flight_duration_sec)) {
        dt = 0.0;
    }
    double drag_factor = std::clamp(1.0 - drag_coeff * dt, 0.0, 1.0);
    px = static_cast<float>(x + vx * dt * drag_factor);
    py = static_cast<float>(y + vy * dt * drag_factor);
    pz = static_cast<float>(z + vz * dt - 0.5 * gravity_m_s2 * dt * dt);
    return true;
}

bool BallisticTracker::update(
    const ClothTrackingResult& raw, double stamp_sec, ClothTrackingResult& out)
{
    // =========================================================================
    // Phase A: IDLE / LANDED -> Instant 1-Frame Launch Detection
    // =========================================================================
    if (state == TrackerState::IDLE || state == TrackerState::LANDED) {
        if (!raw.valid) {
            has_pending = false;
            return false;
        }

        // Candidate must be inside the small configured launch gate with sufficient points
        if (raw.cx >= launch_min_x && raw.cx <= launch_max_x &&
            raw.cy >= launch_min_y && raw.cy <= launch_max_y &&
            raw.cz >= launch_min_z && raw.cz <= launch_max_z &&
            raw.cluster_points_count >= min_cluster_points)
        {
            // Instant 1-frame launch lock-on: Start tracking immediately!
            state = TrackerState::TRACKING;
            x = raw.cx;
            y = raw.cy;
            z = raw.cz;
            launch_x = raw.cx;
            launch_y = raw.cy;
            launch_z = raw.cz;
            launch_stamp = stamp_sec;
            launch_displacement_confirmed = false;
            vx = min_launch_vx;
            vy = 0.0;
            vz = 0.0;
            last_stamp = stamp_sec;
            tracked_count = 1;
            lost_count = 0;
            stationary_count = 0;
            has_pending = false;
            out = raw;
            return true;
        } else {
            has_pending = false;
            return false;
        }
    }

    // =========================================================================
    // Phase B: TRACKING -> In-flight Ballistic Tracking & Drag Compensation
    // =========================================================================
    if (max_flight_duration_sec > 0.0 && (stamp_sec - launch_stamp) > max_flight_duration_sec) {
        reset();
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

        double dt_raw = stamp_sec - last_stamp;
        double dt_safe = (dt_raw > 0.0) ? dt_raw : launch_timeout_sec;
        double new_vx = (raw.cx - x) / dt_safe;
        double new_vy = (raw.cy - y) / dt_safe;
        double new_vz = (raw.cz - z) / dt_safe;

        // A projectile in free forward flight must remain within the 3D ballistic gate
        // and obey configured physical velocity bounds (rejects sideways wall/ceiling jumps)
        if (dist_pred <= gate_radius_m &&
            new_vx >= min_flight_vx_m_s && new_vx <= max_vx_m_s &&
            std::abs(new_vy) <= max_abs_vy_m_s &&
            new_vz >= min_vz_m_s && new_vz <= max_vz_m_s)
        {
            matched = true;

            // Confirm displacement from launch point to reject stationary noise inside launch gate
            if (!launch_displacement_confirmed) {
                double dx_from_launch = raw.cx - launch_x;
                if (dx_from_launch >= min_launch_move_m) {
                    launch_displacement_confirmed = true;
                } else if (stamp_sec - launch_stamp > launch_timeout_sec) {
                    reset();
                    return false;
                }
            }

            // Reject mid-air stationary noise (e.g. static ceiling or back wall flicker)
            double step_dx = raw.cx - x;
            double step_dy = raw.cy - y;
            double step_dz = raw.cz - z;
            double step_move = std::sqrt(step_dx * step_dx + step_dy * step_dy + step_dz * step_dz);
            if (step_move < min_stationary_move_m) {
                stationary_count++;
                if (max_stationary_frames > 0 && stationary_count >= max_stationary_frames) {
                    reset();
                    return false;
                }
            } else {
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
            tracked_count++;
            lost_count = 0;

            out = raw;

            // Stop tracking when cloth touches the floor plane (landing_min_z)
            if (z <= landing_min_z) {
                state = TrackerState::LANDED;
            }
            return true;
        }
    }

    // =========================================================================
    // Phase C: Observation Missed / Lost Frame Handling
    // =========================================================================
    if (!matched) {
        if (!launch_displacement_confirmed && (stamp_sec - launch_stamp) > launch_timeout_sec) {
            reset();
            return false;
        }
        lost_count++;
        if (lost_count > max_lost_frames || pred_z <= landing_min_z) {
            state = TrackerState::LANDED;
            return false;
        }
        return false;
    }

    return false;
}

} // namespace realsense_dust_cloth
