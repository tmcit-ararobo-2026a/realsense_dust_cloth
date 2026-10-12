#include "realsense_dust_cloth/ballistic_tracker.hpp"

namespace realsense_dust_cloth {

void BallisticTracker::reset() {
    state = TrackerState::IDLE;
    tracked_count = 0;
    lost_count = 0;
    has_pending = false;
    last_stamp = 0.0;
    pending_stamp = 0.0;
}

bool BallisticTracker::get_prediction(double stamp_sec, float& px, float& py, float& pz) const {
    if (state != TrackerState::TRACKING || last_stamp <= 0.0) {
        return false;
    }
    double dt = stamp_sec - last_stamp;
    if (dt <= 0.0 || dt > 0.3) {
        dt = 0.016; // 60fps fallback delta
    }
    px = static_cast<float>(x + vx * dt);
    py = static_cast<float>(y + vy * dt);
    pz = static_cast<float>(z + vz * dt - 0.5 * gravity_m_s2 * dt * dt);
    return true;
}

bool BallisticTracker::update(
    const ClothTrackingResult& raw, double stamp_sec, ClothTrackingResult& out)
{
    // =========================================================================
    // Phase A: IDLE / LANDED -> Waiting for Launch Confirmation
    // =========================================================================
    if (state == TrackerState::IDLE || state == TrackerState::LANDED) {
        if (!raw.valid) {
            has_pending = false;
            return false;
        }

        // Lock onto newly ejected cloth only (must satisfy launch zone and minimum cluster points)
        if (raw.cx >= launch_min_x && raw.cx <= launch_max_x &&
            raw.cy >= launch_min_y && raw.cy <= launch_max_y &&
            raw.cz >= launch_min_z && raw.cz <= launch_max_z &&
            raw.cluster_points_count >= min_cluster_points)
        {
            state = TrackerState::TRACKING;
            x = raw.cx;
            y = raw.cy;
            z = raw.cz;
            vx = 2.0;  // Initial estimated ejection forward velocity
            vy = 0.0;
            vz = 0.5;  // Initial slight upward velocity
            last_stamp = stamp_sec;
            tracked_count = 1;
            lost_count = 0;
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
    double dt = stamp_sec - last_stamp;
    if (dt <= 0.0 || dt > 0.40) {
        dt = 0.033;
    }

    // Ballistic prediction with aerodynamic drag deceleration & gravity acceleration
    double drag_factor = std::max(0.50, 1.0 - drag_coeff * dt);
    double pred_x = x + vx * dt * drag_factor;
    double pred_y = y + vy * dt;
    double pred_z = z + vz * dt - 0.5 * gravity_m_s2 * dt * dt;
    vz -= gravity_m_s2 * dt;

    bool matched = false;
    int min_track_pts = std::max(5, min_cluster_points / 4);

    if (raw.valid && raw.cluster_points_count >= min_track_pts) {
        double dx_prev = raw.cx - x;
        double dy_prev = raw.cy - y;
        double dz_prev = raw.cz - z;
        double dist_prev = std::sqrt(dx_prev * dx_prev + dy_prev * dy_prev + dz_prev * dz_prev);

        double dx_pred = raw.cx - pred_x;
        double dy_pred = raw.cy - pred_y;
        double dz_pred = raw.cz - pred_z;
        double dist_pred = std::sqrt(dx_pred * dx_pred + dy_pred * dy_pred + dz_pred * dz_pred);

        // Persistently track cloth: within gate_radius_m of prediction or previous location and advancing
        if (dist_pred <= gate_radius_m || (dist_prev <= gate_radius_m && dx_prev >= -0.30)) {
            matched = true;
            double dt_safe = std::clamp(dt, 0.005, 0.100);

            double new_vx = (raw.cx - x) / dt_safe;
            double new_vy = (raw.cy - y) / dt_safe;
            double new_vz = (raw.cz - z) / dt_safe;

            // Low-pass exponential smoothing for velocities
            vx = 0.6 * vx + 0.4 * new_vx;
            vy = 0.6 * vy + 0.4 * new_vy;
            vz = 0.6 * vz + 0.4 * new_vz;

            x = raw.cx;
            y = raw.cy;
            z = raw.cz;

            last_stamp = stamp_sec;
            tracked_count++;
            lost_count = 0;

            out = raw;
            out.cx = static_cast<float>(x);
            out.cy = static_cast<float>(y);
            out.cz = static_cast<float>(z);

            // Stop tracking when cloth touches the floor plane or comes to a complete rest
            if (z <= landing_min_z || (z <= landing_min_z * 1.5 && vx <= min_launch_vx * 2.0)) {
                state = TrackerState::LANDED;
            }
            return true;
        }
    }

    // =========================================================================
    // Phase C: Observation Missed / Lost Frame Handling
    // =========================================================================
    if (!matched) {
        lost_count++;
        if (lost_count > max_lost_frames || pred_z <= landing_min_z * 0.5) {
            state = TrackerState::LANDED;
            return false;
        }

        // Maintain continuous search gate advancing along the ballistic arc
        x = pred_x;
        y = pred_y;
        z = std::max(landing_min_z * 0.5, pred_z);
        last_stamp = stamp_sec;
        return false;
    }

    return false;
}

} // namespace realsense_dust_cloth

