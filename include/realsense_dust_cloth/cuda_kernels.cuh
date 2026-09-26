#pragma once

#include <cstdint>
#include <vector>
#include <cuda_runtime.h>

namespace realsense_dust_cloth
{

    struct CameraIntrinsics
    {
        float fx{0.0f};
        float fy{0.0f};
        float cx{0.0f};
        float cy{0.0f};
        int width{848};
        int height{480};
        float depth_scale{0.001f}; // mm to meters
    };

    struct CameraPoseConfig
    {
        float pitch_deg{45.0f};       // Upward pitch from horizontal
        float roll_deg{90.0f};        // Roll angle (90 deg vertical mount)
        float x_offset_m{0.25f};      // Forward offset in base_link
        float height_offset_m{0.35f}; // Vertical offset in base_link
    };

    struct FilterConfig
    {
        float min_depth_m{0.25f};
        float max_depth_m{6.0f};

        // PassThrough bounds in base_link frame
        float min_x_m{0.30f};
        float max_x_m{6.0f};
        float min_y_m{-1.5f};
        float max_y_m{1.5f};
        float min_z_m{0.05f};
        float max_z_m{3.5f};

        // Background Subtraction
        float bg_diff_thresh_m{0.15f}; // Object must be at least 15cm in front of background
        bool use_bg_subtraction{true};

        // Clustering & Voxel
        float voxel_size_m{0.03f};
        float cluster_eps_m{0.15f};
        int min_cluster_points{10};
        int max_cluster_points{1500};
        float max_cloth_size_m{0.65f};
    };

    struct ClothTrackingResult
    {
        bool valid{false};
        float cx{0.0f};
        float cy{0.0f};
        float cz{0.0f};
        int cluster_points_count{0};
        float bbox_dx{0.0f};
        float bbox_dy{0.0f};
        float bbox_dz{0.0f};
    };

    struct CudaTrackerContext
    {
        int max_pixels{0};
        uint16_t *d_depth{nullptr};

        // Filtered points before voxel downsampling
        float3 *d_valid_points{nullptr};
        int *d_valid_count{nullptr};

        // Voxel downsampled points
        static constexpr int MAX_VOXEL_POINTS = 8192;
        float3 *d_voxel_points{nullptr};
        int *d_voxel_count{nullptr};

        // Spatial hash table for fast voxelization
        static constexpr int HASH_TABLE_SIZE = 32768; // Power of 2
        uint64_t *d_hash_keys{nullptr};

        // Clustering Union-Find & labels
        int *d_parent{nullptr};
        int *d_cluster_sizes{nullptr};
        float3 *d_cluster_min{nullptr};
        float3 *d_cluster_max{nullptr};
        float3 *d_cluster_sum{nullptr};

        uint16_t *d_bg_depth{nullptr};
        bool bg_ready{false};

        // Host pinned memory for fast result transfer
        ClothTrackingResult *h_result_pinned{nullptr};
        ClothTrackingResult *d_result{nullptr};
        int *d_best_root{nullptr};

        cudaStream_t stream{nullptr};
        bool initialized{false};
    };

    bool init_cuda_tracker(CudaTrackerContext &ctx, int max_depth_width = 848, int max_depth_height = 480);
    void destroy_cuda_tracker(CudaTrackerContext &ctx);
    void reset_background_cuda(CudaTrackerContext &ctx);
    void update_background_cuda(CudaTrackerContext &ctx, const uint16_t *host_depth_ptr, int width, int height);

    ClothTrackingResult process_depth_frame_cuda(
        CudaTrackerContext &ctx,
        const uint16_t *host_depth_ptr,
        const CameraIntrinsics &intrin,
        const CameraPoseConfig &pose,
        const FilterConfig &filter,
        std::vector<float3> *out_cluster_points = nullptr,
        float pred_x = 0.0f,
        float pred_y = 0.0f,
        float pred_z = 0.0f,
        bool has_prediction = false);

} // namespace realsense_dust_cloth
