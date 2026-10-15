#pragma once

#include <algorithm>
#include <cmath>
#include "realsense_dust_cloth/cuda_kernels.cuh"

namespace realsense_dust_cloth {

/**
 * @brief States for the dust cloth tracking state machine.
 */
enum class TrackerState {
    IDLE,       ///< Waiting for launch from configured launch zone
    TRACKING,   ///< Actively tracking flying cloth along ballistic trajectory
    LANDED      ///< Cloth touched floor or came to complete rest
};

/**
 * @brief Physics-based ballistic state machine for flying dust cloth tracking.
 *
 * Rejects stationary objects and background noise by:
 * 1. Only locking onto objects originating from the configured launch gate with forward motion.
 * 2. Continuously predicting parabolic flight with aerodynamic drag deceleration.
 * 3. Smoothly filtering velocity and tracking cluster centroids.
 * 4. Detecting landing when the cloth reaches the floor or stops moving.
 */
class BallisticTracker {
public:
    TrackerState state{TrackerState::IDLE};
    double x{0.0}, y{0.0}, z{0.0};
    double vx{0.0}, vy{0.0}, vz{0.0};
    double last_stamp{0.0};
    int tracked_count{0};
    int lost_count{0};

    // Launch tracking and displacement confirmation
    bool has_pending{false};
    double pending_x{0.0}, pending_y{0.0}, pending_z{0.0};
    double pending_stamp{0.0};
    double launch_x{0.0}, launch_y{0.0}, launch_z{0.0};
    double launch_stamp{0.0};
    bool launch_displacement_confirmed{false};
    double min_launch_vx{0.30}; ///< Minimum forward velocity (m/s) to confirm launch

    // Configurable Parameters (Loaded via ROS 2 tracker_params.yaml)
    double launch_min_x{0.40};
    double launch_max_x{0.85};
    double launch_min_y{-0.30};
    double launch_max_y{0.30};
    double launch_min_z{0.65};
    double launch_max_z{1.30};
    double gate_radius_m{0.80};
    int max_lost_frames{15};
    double gravity_m_s2{9.81};
    int min_cluster_points{20};
    double drag_coeff{0.40};
    double landing_min_z{0.10};

    // Velocity Smoothing & Physical Velocity Bounds
    double vel_smoothing_beta{0.30};  ///< Velocity measurement blend weight (0..1)
    double max_vx_m_s{6.00};          ///< Maximum physical forward velocity (m/s)
    double max_abs_vy_m_s{2.00};      ///< Maximum physical lateral velocity (m/s)
    double min_vz_m_s{-6.00};         ///< Minimum physical vertical velocity (m/s)
    double max_vz_m_s{4.00};          ///< Maximum physical vertical velocity (m/s)

    // High-speed Launch & Tracking Controls (Loaded via ROS 2 tracker_params.yaml)
    double min_launch_move_m{0.08};   ///< Minimum forward displacement (m) to confirm launch
    double launch_timeout_sec{0.20};  ///< Timeout to reset stationary launch pending anchor (s)
    int min_track_points_limit{8};    ///< Lower bound for in-flight minimum cluster points

    /**
     * @brief Reset tracker state machine to IDLE and clear tracking history.
     */
    void reset();

    /**
     * @brief Predict the cloth 3D position at stamp_sec using the ballistic model.
     * @param stamp_sec Target timestamp in seconds
     * @param px Output predicted X (forward)
     * @param py Output predicted Y (lateral)
     * @param pz Output predicted Z (vertical)
     * @return true if actively tracking with valid prediction, false otherwise
     */
    bool get_prediction(double stamp_sec, float& px, float& py, float& pz) const;

    /**
     * @brief Update tracker state machine with a raw candidate cloth cluster.
     * @param raw Raw candidate cluster from CUDA depth processor
     * @param stamp_sec Timestamp of current camera frame in seconds
     * @param out Output filtered and smoothed cloth tracking result
     * @return true if currently actively tracking the cloth, false if idle or lost
     */
    bool update(const ClothTrackingResult& raw, double stamp_sec, ClothTrackingResult& out);
};

} // namespace realsense_dust_cloth

