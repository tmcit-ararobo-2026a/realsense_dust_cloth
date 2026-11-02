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

        // Clustering & Voxel
        float voxel_size_m{0.03f};
        float cluster_eps_m{0.15f};
        int min_cluster_points{10};
        int min_tracking_points{6};
        int max_cluster_points{1500};
        float min_cloth_size_m{0.05f};          // 最小3Dサイズ制限 (m, 微小ノイズ除外)
        float max_cloth_size_m{0.65f};          // 最大3D対角サイズ制限 (m)

        // 2D Edge Continuity (Configurable via YAML)
        float continuity_thresh_m{0.30f};       // エッジ連続性チェックの許容深度差 (m)
        int continuity_radius_px{2};
        int continuity_min_neighbors{2};        // 4近傍のうち必要な連続ピクセル数 (0: 無効, 1〜4)

        // Launch Gate Bounds & Ballistic Prioritization
        float launch_min_x{0.40f};
        float launch_max_x{1.80f};
        float launch_min_y{-0.50f};
        float launch_max_y{0.50f};
        float launch_min_z{0.50f};
        float launch_max_z{2.00f};
        float gate_radius_m{0.80f};
        float prediction_distance_weight{1000.0f};
        float cluster_size_weight{0.1f};
        float launch_forward_weight{50.0f};
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
        int max_voxel_points{8192};
        float3 *d_voxel_points{nullptr};
        int *d_voxel_count{nullptr};

        // Spatial hash table for fast voxelization
        int hash_table_size{32768}; // Power of 2, configured before allocation
        uint64_t *d_hash_keys{nullptr};

        // Clustering Union-Find & labels
        int *d_parent{nullptr};
        int *d_cluster_sizes{nullptr};
        float3 *d_cluster_min{nullptr};
        float3 *d_cluster_max{nullptr};
        float3 *d_cluster_sum{nullptr};

        // Host pinned memory for fast result transfer
        ClothTrackingResult *h_result_pinned{nullptr};
        ClothTrackingResult *d_result{nullptr};
        int *d_best_root{nullptr};

        cudaStream_t stream{nullptr};
        bool initialized{false};
    };

    bool init_cuda_tracker(CudaTrackerContext &ctx, int max_depth_width = 848, int max_depth_height = 480);
    void destroy_cuda_tracker(CudaTrackerContext &ctx);

    // External-map path: export ROI voxels once, then cluster returned voxels without re-voxelizing.
    std::vector<float3> prepare_depth_cloud_cuda(CudaTrackerContext&, const uint16_t*,
        const CameraIntrinsics&, const CameraPoseConfig&, const FilterConfig&);
    ClothTrackingResult process_point_cloud_cuda(CudaTrackerContext&, const std::vector<float3>&,
        const FilterConfig&, std::vector<float3>*, std::vector<float3>*,
        float, float, float, bool);

} // namespace realsense_dust_cloth
