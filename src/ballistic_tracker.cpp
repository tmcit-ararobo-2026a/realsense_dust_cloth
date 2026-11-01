#include "realsense_dust_cloth/ballistic_tracker.hpp"

namespace realsense_dust_cloth {

namespace {
constexpr double DEFAULT_FRAME_DT_SEC = 0.016;
constexpr double MIN_VELOCITY_DT_SEC = 0.025;
constexpr double DEFAULT_LAUNCH_VX_M_S = 2.0;
constexpr double LOST_FRAME_GATE_EXPANSION_M = 0.04;
constexpr double MAX_GATE_EXPANSION_M = 0.10;
constexpr double HALF_GRAVITY_FACTOR = 0.5;
constexpr double MIN_HEADING_SPEED_THRESHOLD_M_S = 0.20;
constexpr double MIN_HEADING_STEP_THRESHOLD_M = 0.05;
constexpr double DEG_TO_RAD_FACTOR = 180.0 / M_PI;
} // namespace

void BallisticTracker::reset() {
    state = TrackerState::IDLE;
    tracked_count = 0;
    lost_count = 0;
    stationary_count = 0;
    anomaly_count = 0;
    launch_displacement_confirmed = false;
    is_pending_launch = false;
    has_reached_apex = false;
    apex_z = 0.0;
    last_stamp = 0.0;
    launch_stamp = 0.0;
    first_detect_stamp = 0.0;
    anchor_x = 0.0;
    anchor_y = 0.0;
    anchor_z = 0.0;
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
    if (observed_landing_only) pz = std::max(pz,static_cast<float>(landing_min_z));
    return true;
}

bool BallisticTracker::update(
    const ClothTrackingResult& raw, double stamp_sec, ClothTrackingResult& out)
{
    // =========================================================================
    // Phase A: IDLE / LANDED / LOST -> Launch Candidate Acquisition & 2-Stage Verification
    // =========================================================================
    if (state != TrackerState::TRACKING) {
        bool valid_candidate = (raw.valid && raw.cluster_points_count >= min_cluster_points);
        if (!valid_candidate) {
            is_pending_launch = false;
            first_detect_stamp = 0.0;
            return false;
        }

        if (!is_pending_launch) {
            // First detection must originate strictly inside the configured launch gate
            bool in_launch_gate = (raw.cx >= launch_min_x && raw.cx <= launch_max_x &&
                                   raw.cy >= launch_min_y && raw.cy <= launch_max_y &&
                                   raw.cz >= launch_min_z && raw.cz <= launch_max_z);
            if (!in_launch_gate) {
                return false;
            }
            is_pending_launch = true;
            anchor_x = raw.cx;
            anchor_y = raw.cy;
            anchor_z = raw.cz;
            first_detect_stamp = stamp_sec;
            last_stamp = stamp_sec;
            return false;
        }

        // We have an active pending anchor from the launch gate
        double dt_pending = stamp_sec - first_detect_stamp;
        if (dt_pending > launch_timeout_sec) {
            // If candidate lingered or jiggled in the gate too long without moving forward,
            // treat as stationary frame / optical jitter and reset anchor to prevent lock-in.
            bool still_in_gate = (raw.cx >= launch_min_x && raw.cx <= launch_max_x &&
                                  raw.cy >= launch_min_y && raw.cy <= launch_max_y &&
                                  raw.cz >= launch_min_z && raw.cz <= launch_max_z);
            if (still_in_gate) {
                anchor_x = raw.cx;
                anchor_y = raw.cy;
                anchor_z = raw.cz;
                first_detect_stamp = stamp_sec;
            } else {
                is_pending_launch = false;
                first_detect_stamp = 0.0;
            }
            return false;
        }

        double dx_from_anchor = raw.cx - anchor_x;
        double dz_from_anchor = raw.cz - anchor_z;
        double dt_safe = (dt_pending > 0.0) ? dt_pending : DEFAULT_FRAME_DT_SEC;
        double current_vx = dx_from_anchor / dt_safe;

        // Confirm genuine high-speed forward launch displacement and velocity
        if (dx_from_anchor >= min_launch_move_m &&
            current_vx >= min_launch_vx &&
            dz_from_anchor >= -dx_from_anchor)
        {
            state = TrackerState::TRACKING;
            x = raw.cx;
            y = raw.cy;
            z = raw.cz;
            launch_x = anchor_x;
            launch_y = anchor_y;
            launch_z = anchor_z;
            launch_stamp = stamp_sec;
            stationary_ref_x = raw.cx;
            stationary_ref_y = raw.cy;
            stationary_ref_z = raw.cz;
            launch_displacement_confirmed = true;
            vx = std::clamp(current_vx, min_flight_vx_m_s, max_vx_m_s);
            vy = std::clamp((raw.cy - anchor_y) / dt_safe, -max_abs_vy_m_s, max_abs_vy_m_s);
            vz = std::clamp((raw.cz - anchor_z) / dt_safe, min_vz_m_s, max_vz_m_s);
            apex_z = raw.cz;
            has_reached_apex = (vz <= 0.0);
            last_stamp = stamp_sec;
            tracked_count = 1;
            lost_count = 0;
            stationary_count = 0;
            is_pending_launch = false;
            out = raw;
            return true;
        }

        // If candidate moved significantly backwards or dropped vertically below gate, cancel pending
        if (dx_from_anchor < -min_launch_move_m || dz_from_anchor < -0.5) {
            is_pending_launch = false;
            first_detect_stamp = 0.0;
        }
        return false;
    }

    // =========================================================================
    // Phase B: TRACKING -> In-flight Ballistic Tracking & Drag Compensation
    // =========================================================================
    if (max_flight_duration_sec > 0.0 && (stamp_sec - launch_stamp) > max_flight_duration_sec) {
        if (launch_displacement_confirmed && x > launch_max_x) {
            state = publish_cutoff_as_landing ? TrackerState::LANDED : (observed_landing_only ? TrackerState::LOST : TrackerState::LANDED);
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

    double dt_raw = stamp_sec - last_stamp;
    double dt_safe = (dt_raw > 0.0) ? dt_raw : launch_timeout_sec;

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

        double dt_vel = std::max(dt_safe, MIN_VELOCITY_DT_SEC);
        double new_vx = dx_prev / dt_vel;
        double new_vy = dy_prev / dt_vel;
        double new_vz = dz_prev / dt_vel;

        // A projectile in free forward flight must remain within the 3D ballistic tube
        double effective_gate = tube_radius_m + std::min(lost_count * LOST_FRAME_GATE_EXPANSION_M, MAX_GATE_EXPANSION_M);
        double max_step_dist = effective_gate + max_vx_m_s * dt_safe;

        bool physically_plausible = true;

        // 1. Strict forward monotonic progress (no backward velocity or displacement beyond jitter tolerance)
        if (raw.cx < x - backward_jitter_tolerance_m || new_vx < min_flight_vx_m_s || new_vx > max_vx_m_s) {
            physically_plausible = false;
        }

        // 2. No ascent after apex: once descending, a free projectile cannot gain altitude in mid-air
        if (physically_plausible && reject_ascent_after_apex && has_reached_apex) {
            if (raw.cz > z + vertical_jitter_tolerance_m || new_vz > 0.0) {
                physically_plausible = false;
            }
        }

        // 3. Physical 3D Acceleration limits: gravity is -9.8 m/s^2; extreme downward, lateral, or total acceleration is impossible
        if (physically_plausible) {
            double ax = (new_vx - vx) / dt_safe;
            double ay = (new_vy - vy) / dt_safe;
            double az = (new_vz - vz) / dt_safe;
            double total_accel = std::sqrt(ax * ax + ay * ay + az * az);
            if (total_accel > max_total_accel_m_s2 || az < min_vertical_accel_m_s2 || std::abs(ay) > max_lateral_accel_m_s2) {
                physically_plausible = false;
            }
        }

        // 4. Heading deviation angle: projectile momentum prevents sharp orthogonal turns into desks/obstacles
        if (physically_plausible) {
            double prev_speed = std::sqrt(vx * vx + vy * vy + vz * vz);
            double step_dist = std::sqrt(dx_prev * dx_prev + dy_prev * dy_prev + dz_prev * dz_prev);
            if (prev_speed > MIN_HEADING_SPEED_THRESHOLD_M_S && step_dist > MIN_HEADING_STEP_THRESHOLD_M) {
                double dot = (vx * dx_prev + vy * dy_prev + vz * dz_prev) / (prev_speed * step_dist);
                double cos_clamped = std::clamp(dot, -1.0, 1.0);
                double deviation_deg = std::acos(cos_clamped) * DEG_TO_RAD_FACTOR;
                if (deviation_deg > max_heading_deg) {
                    physically_plausible = false;
                }
            }
        }

        if (dist_pred <= effective_gate &&
            dist_prev <= max_step_dist &&
            physically_plausible &&
            std::abs(new_vy) <= max_abs_vy_m_s &&
            new_vz >= min_vz_m_s && new_vz <= max_vz_m_s)
        {
            matched = true;
            anomaly_count = 0;

            // Detect mid-air or pole-hooked resting state
            double ref_dx = raw.cx - stationary_ref_x;
            double ref_dy = raw.cy - stationary_ref_y;
            double ref_dz = raw.cz - stationary_ref_z;
            double dist_from_ref = std::sqrt(ref_dx * ref_dx + ref_dy * ref_dy + ref_dz * ref_dz);
            if (dist_from_ref < min_stationary_move_m) {
                stationary_count++;
                if (max_stationary_frames > 0 && stationary_count >= max_stationary_frames) {
                    if (x > launch_max_x) {
                        state = observed_landing_only ? TrackerState::RESTING : TrackerState::LANDED;
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

            x = raw.cx;
            y = raw.cy;
            z = raw.cz;

            apex_z = std::max(apex_z, z);
            if (vz <= 0.0 || z < apex_z - vertical_jitter_tolerance_m) {
                has_reached_apex = true;
            }

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
        } else {
            // An unphysical or out-of-tube candidate was detected
            anomaly_count++;
            if (abort_on_anomaly && anomaly_count >= max_anomaly_frames) {
                state = publish_cutoff_as_landing ? TrackerState::LANDED : (observed_landing_only ? TrackerState::LOST : TrackerState::LANDED);
                lost_count = 0;
                stationary_count = 0;
                return false;
            }
        }
    }

    // =========================================================================
    // Phase C: Observation Missed / Lost Frame Handling
    // =========================================================================
    if (!matched) {
        lost_count++;
        if (lost_count > max_lost_frames || (!observed_landing_only && pred_z <= landing_min_z)) {
            state = publish_cutoff_as_landing ? TrackerState::LANDED : (observed_landing_only ? TrackerState::LOST : TrackerState::LANDED);
            lost_count = 0;
            stationary_count = 0;
            return false;
        }

        // Coast Tracking: Maintain smooth physical trajectory through temporary occlusion
        if (enable_coast_prediction_tf && pred_z > landing_min_z) {
            double drag_factor = std::clamp(1.0 - drag_coeff * dt_safe, 0.0, 1.0);
            double pred_vx = vx * drag_factor;
            double pred_vy = vy * drag_factor;
            double pred_vz = vz - gravity_m_s2 * dt_safe;

            x = pred_x;
            y = pred_y;
            z = pred_z;
            vx = pred_vx;
            vy = pred_vy;
            vz = pred_vz;

            apex_z = std::max(apex_z, z);
            if (vz <= 0.0 || z < apex_z - vertical_jitter_tolerance_m) {
                has_reached_apex = true;
            }

            last_stamp = stamp_sec;
            tracked_count++;

            out.valid = true;
            out.cx = static_cast<float>(pred_x);
            out.cy = static_cast<float>(pred_y);
            out.cz = static_cast<float>(pred_z);
            out.cluster_points_count = min_track_pts;
            return true;
        }

        return false;
    }

    return false;
}

} // namespace realsense_dust_cloth
