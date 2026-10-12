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
    if (dt <= 0.0 || dt > 0.40) {
        dt = 0.016; // 60fps fallback delta
    }
    double drag_factor = std::max(0.50, 1.0 - drag_coeff * dt);
    px = static_cast<float>(x + vx * dt * drag_factor);
    py = static_cast<float>(y + vy * dt * drag_factor);
    pz = static_cast<float>(z + vz * dt - 0.5 * gravity_m_s2 * dt * dt);
    return true;
}

bool BallisticTracker::update(
    const ClothTrackingResult& raw, double stamp_sec, ClothTrackingResult& out)
{
    // =========================================================================
    // Phase A: IDLE / LANDED -> Waiting for 2-Frame Launch Velocity Confirmation
    // =========================================================================
    if (state == TrackerState::IDLE || state == TrackerState::LANDED) {
        if (!raw.valid) {
            has_pending = false;
            return false;
        }

        // Candidate must be inside the configured launch gate with sufficient points
        if (raw.cx >= launch_min_x && raw.cx <= launch_max_x &&
            raw.cy >= launch_min_y && raw.cy <= launch_max_y &&
            raw.cz >= launch_min_z && raw.cz <= launch_max_z &&
            raw.cluster_points_count >= min_cluster_points)
        {
            if (!has_pending) {
                has_pending = true;
                pending_x = raw.cx;
                pending_y = raw.cy;
                pending_z = raw.cz;
                pending_stamp = stamp_sec;
                return false;
            }

            double dt = stamp_sec - pending_stamp;
            if (dt >= 0.005 && dt <= 0.25) {
                double vx_init = (raw.cx - pending_x) / dt;
                double vy_init = (raw.cy - pending_y) / dt;
                double vz_init = (raw.cz - pending_z) / dt;

                if (vx_init >= min_launch_vx) {
                    state = TrackerState::TRACKING;
                    x = raw.cx;
                    y = raw.cy;
                    z = raw.cz;
                    vx = std::clamp(vx_init, min_launch_vx, max_vx_m_s);
                    vy = std::clamp(vy_init, -max_abs_vy_m_s, max_abs_vy_m_s);
                    vz = std::clamp(vz_init, min_vz_m_s, max_vz_m_s);
                    last_stamp = stamp_sec;
                    tracked_count = 1;
                    lost_count = 0;
                    has_pending = false;
                    out = raw;
                    return true;
                }
            }

            // Update pending anchor to latest observation in launch zone
            pending_x = raw.cx;
            pending_y = raw.cy;
            pending_z = raw.cz;
            pending_stamp = stamp_sec;
            return false;
        } else {
            has_pending = false;
            return false;
        }
    }

    // =========================================================================
    // Phase B: TRACKING -> In-flight Ballistic Tracking & Drag Compensation
    // =========================================================================
    float pred_xf = 0.0f, pred_yf = 0.0f, pred_zf = 0.0f;
    get_prediction(stamp_sec, pred_xf, pred_yf, pred_zf);
    double pred_x = static_cast<double>(pred_xf);
    double pred_y = static_cast<double>(pred_yf);
    double pred_z = static_cast<double>(pred_zf);

    bool matched = false;
    int min_track_pts = std::max(5, min_cluster_points / 4);

    if (raw.valid && raw.cluster_points_count >= min_track_pts) {
        double dx_pred = raw.cx - pred_x;
        double dy_pred = raw.cy - pred_y;
        double dz_pred = raw.cz - pred_z;
        double dist_pred = std::sqrt(dx_pred * dx_pred + dy_pred * dy_pred + dz_pred * dz_pred);

        double dt_safe = std::clamp(stamp_sec - last_stamp, 0.008, 0.250);
        double new_vx = (raw.cx - x) / dt_safe;
        double new_vy = (raw.cy - y) / dt_safe;
        double new_vz = (raw.cz - z) / dt_safe;

        // A projectile in free forward flight must remain within the 3D ballistic gate
        // and maintain positive forward motion (cannot reverse or hover on back-wall noise)
        if (dist_pred <= gate_radius_m && new_vx >= 0.0) {
            matched = true;

            double drag_factor = std::max(0.50, 1.0 - drag_coeff * dt_safe);
            double pred_vx = vx * drag_factor;
            double pred_vy = vy * drag_factor;
            double pred_vz = vz - gravity_m_s2 * dt_safe;

            vx = std::clamp((1.0 - vel_smoothing_beta) * pred_vx + vel_smoothing_beta * new_vx, 0.0, max_vx_m_s);
            vy = std::clamp((1.0 - vel_smoothing_beta) * pred_vy + vel_smoothing_beta * new_vy, -max_abs_vy_m_s, max_abs_vy_m_s);
            vz = std::clamp((1.0 - vel_smoothing_beta) * pred_vz + vel_smoothing_beta * new_vz, min_vz_m_s, max_vz_m_s);

            x = (1.0 - pos_smoothing_alpha) * pred_x + pos_smoothing_alpha * raw.cx;
            y = (1.0 - pos_smoothing_alpha) * pred_y + pos_smoothing_alpha * raw.cy;
            z = (1.0 - pos_smoothing_alpha) * pred_z + pos_smoothing_alpha * raw.cz;

            last_stamp = stamp_sec;
            tracked_count++;
            lost_count = 0;

            out = raw;
            out.cx = static_cast<float>(x);
            out.cy = static_cast<float>(y);
            out.cz = static_cast<float>(z);

            // Stop tracking when cloth touches the floor plane or loses forward flight speed
            if (z <= landing_min_z || vx <= min_launch_vx) {
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
        return false;
    }

    return false;
}

} // namespace realsense_dust_cloth

