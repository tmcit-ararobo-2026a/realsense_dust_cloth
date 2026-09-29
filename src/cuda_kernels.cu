#include "realsense_dust_cloth/cuda_kernels.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>
#include <iostream>

namespace realsense_dust_cloth
{

    namespace
    {

        __device__ __forceinline__ float atomicMinFloat(float *addr, float value)
        {
            float old;
            old = (value >= 0) ? __int_as_float(atomicMin((int *)addr, __float_as_int(value))) : __uint_as_float(atomicMax((unsigned int *)addr, __float_as_uint(value)));
            return old;
        }

        __device__ __forceinline__ float atomicMaxFloat(float *addr, float value)
        {
            float old;
            old = (value >= 0) ? __int_as_float(atomicMax((int *)addr, __float_as_int(value))) : __uint_as_float(atomicMin((unsigned int *)addr, __float_as_uint(value)));
            return old;
        }

        __device__ int find_root(int *parent, int i)
        {
            while (parent[i] != i)
            {
                int p = parent[i];
                parent[i] = parent[p]; // Path halving
                i = p;
            }
            return i;
        }

        __device__ void union_clusters(int *parent, int a, int b)
        {
            int root_a = find_root(parent, a);
            int root_b = find_root(parent, b);
            while (root_a != root_b)
            {
                if (root_a > root_b)
                {
                    int old = atomicMin(&parent[root_a], root_b);
                    if (old == root_a)
                        break;
                    root_a = find_root(parent, old);
                }
                else
                {
                    int old = atomicMin(&parent[root_b], root_a);
                    if (old == root_b)
                        break;
                    root_b = find_root(parent, old);
                }
            }
        }

        // -------------------------------------------------------------
        // Kernel 1: Back-Projection, Coordinate Transform & PassThrough
        // -------------------------------------------------------------
        __global__ void kernel_backproject_and_passthrough(
            const uint16_t *__restrict__ depth,
            const uint16_t *__restrict__ bg_depth,
            bool use_bg_sub,
            CameraIntrinsics intrin,
            CameraPoseConfig pose,
            FilterConfig filter,
            float3 *__restrict__ valid_points,
            int *__restrict__ valid_count,
            int max_valid_points)
        {
            int u = blockIdx.x * blockDim.x + threadIdx.x;
            int v = blockIdx.y * blockDim.y + threadIdx.y;

            if (u >= intrin.width || v >= intrin.height)
                return;

            int idx = v * intrin.width + u;
            uint16_t raw_depth = depth[idx];
            if (raw_depth == 0)
                return;

            float z_raw = raw_depth * intrin.depth_scale;
            if (z_raw < filter.min_depth_m || z_raw > filter.max_depth_m)
                return;

            // Background Subtraction: If this pixel is at or behind the background, discard!
            if (use_bg_sub && bg_depth != nullptr)
            {
                uint16_t bg = bg_depth[idx];
                if (bg > 0)
                {
                    float z_bg = bg * intrin.depth_scale;
                    if (z_raw >= (z_bg - filter.bg_diff_thresh_m))
                    {
                        return; // Part of background (walls, ceiling, floor, furniture)
                    }
                }
            }

            // Camera optical frame (Z forward, X right, Y down)
            float x_cam = (static_cast<float>(u) - intrin.cx) * z_raw / intrin.fx;
            float y_cam = (static_cast<float>(v) - intrin.cy) * z_raw / intrin.fy;
            float z_cam = z_raw;

            // Transform to base_link:
            // Transform to base_link:
            // Vertical mounting (upright with upward pitch 20 deg):
            // Physical 3D transformation for vertical mounting (roll ±90 deg) and upward tilt (pitch_deg):
            float pitch_rad = pose.pitch_deg * (3.14159265358979323846f / 180.0f);
            float cos_p = cosf(pitch_rad);
            float sin_p = sinf(pitch_rad);

            // Upright physical coordinate transformation (180 deg flipped from upside-down):
            // Forward X: z_cam * cos_p - x_cam * sin_p
            // Lateral Y: y_cam
            // Vertical Z: height_offset + (z_cam * sin_p + x_cam * cos_p)
            float x_rob = pose.x_offset_m + (z_cam * cos_p - x_cam * sin_p);
            float y_rob = y_cam;
            float z_rob = pose.height_offset_m + (z_cam * sin_p + x_cam * cos_p);


            // PassThrough in base_link
            if (x_rob < filter.min_x_m || x_rob > filter.max_x_m ||
                y_rob < filter.min_y_m || y_rob > filter.max_y_m ||
                z_rob < filter.min_z_m || z_rob > filter.max_z_m)
            {
                return;
            }

            int out_idx = atomicAdd(valid_count, 1);
            if (out_idx < max_valid_points)
            {
                valid_points[out_idx] = make_float3(x_rob, y_rob, z_rob);
            }
        }

        // -------------------------------------------------------------
        // Kernel 2: Fast Spatial Hash Voxel Downsampling
        // -------------------------------------------------------------
        __global__ void kernel_voxel_downsample(
            const float3 *__restrict__ valid_points,
            int valid_count,
            float voxel_size,
            uint64_t *__restrict__ hash_keys,
            float3 *__restrict__ voxel_points,
            int *__restrict__ voxel_count,
            int max_voxel_points,
            int hash_table_size)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            if (idx >= valid_count)
                return;

            float3 pt = valid_points[idx];
            int vx = __float2int_rd(pt.x / voxel_size);
            int vy = __float2int_rd(pt.y / voxel_size);
            int vz = __float2int_rd(pt.z / voxel_size);

            // 64-bit voxel spatial key
            uint64_t key = ((static_cast<uint64_t>(vx + 2048) & 0x1FFFFFULL) << 42) |
                           ((static_cast<uint64_t>(vy + 2048) & 0x1FFFFFULL) << 21) |
                           (static_cast<uint64_t>(vz + 2048) & 0x1FFFFFULL);

            // Hash function (Murmur / splitmix inspired)
            uint64_t h = key;
            h ^= h >> 33;
            h *= 0xff51afd7ed558ccdULL;
            h ^= h >> 33;
            h *= 0xc4ceb9fe1a85ec53ULL;
            h ^= h >> 33;
            uint32_t slot = static_cast<uint32_t>(h) & (hash_table_size - 1);

            // Linear probing
            for (int probe = 0; probe < 32; ++probe)
            {
                uint32_t s = (slot + probe) & (hash_table_size - 1);
                unsigned long long prev = atomicCAS(
                    reinterpret_cast<unsigned long long *>(&hash_keys[s]),
                    0xFFFFFFFFFFFFFFFFULL,
                    static_cast<unsigned long long>(key));

                if (prev == 0xFFFFFFFFFFFFFFFFULL)
                {
                    // First time this voxel is encountered
                    int v_idx = atomicAdd(voxel_count, 1);
                    if (v_idx < max_voxel_points)
                    {
                        voxel_points[v_idx] = pt;
                    }
                    break;
                }
                else if (prev == static_cast<unsigned long long>(key))
                {
                    // Duplicate voxel
                    break;
                }
            }
        }

        // -------------------------------------------------------------
        // Kernel 3: Clustering Initialization & Parallel Union-Find
        // -------------------------------------------------------------
        __global__ void kernel_init_clusters(int *parent, int count)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            if (idx < count)
            {
                parent[idx] = idx;
            }
        }

        __global__ void kernel_cluster_union_find(
            const float3 *__restrict__ points,
            int count,
            float eps_sq,
            int *__restrict__ parent)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= count)
                return;

            float3 pi = points[i];

            for (int j = i + 1; j < count; ++j)
            {
                float3 pj = points[j];
                float dx = pi.x - pj.x;
                float dy = pi.y - pj.y;
                float dz = pi.z - pj.z;
                float d2 = dx * dx + dy * dy + dz * dz;

                if (d2 <= eps_sq)
                {
                    union_clusters(parent, i, j);
                }
            }
        }

        // -------------------------------------------------------------
        // Kernel 4: Cluster Properties Accumulation
        // -------------------------------------------------------------
        __global__ void kernel_init_cluster_stats(
            int *cluster_sizes,
            float3 *cluster_min,
            float3 *cluster_max,
            float3 *cluster_sum,
            int count)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            if (idx < count)
            {
                cluster_sizes[idx] = 0;
                cluster_min[idx] = make_float3(1e9f, 1e9f, 1e9f);
                cluster_max[idx] = make_float3(-1e9f, -1e9f, -1e9f);
                cluster_sum[idx] = make_float3(0.0f, 0.0f, 0.0f);
            }
        }

        __global__ void kernel_accumulate_cluster_stats(
            const float3 *__restrict__ points,
            int *__restrict__ parent,
            int count,
            int *__restrict__ cluster_sizes,
            float3 *__restrict__ cluster_min,
            float3 *__restrict__ cluster_max,
            float3 *__restrict__ cluster_sum)
        {
            int i = blockIdx.x * blockDim.x + threadIdx.x;
            if (i >= count)
                return;

            int root = find_root(parent, i);
            float3 p = points[i];

            atomicAdd(&cluster_sizes[root], 1);
            atomicMinFloat(&cluster_min[root].x, p.x);
            atomicMinFloat(&cluster_min[root].y, p.y);
            atomicMinFloat(&cluster_min[root].z, p.z);

            atomicMaxFloat(&cluster_max[root].x, p.x);
            atomicMaxFloat(&cluster_max[root].y, p.y);
            atomicMaxFloat(&cluster_max[root].z, p.z);

            atomicAdd(&cluster_sum[root].x, p.x);
            atomicAdd(&cluster_sum[root].y, p.y);
            atomicAdd(&cluster_sum[root].z, p.z);
        }

        // -------------------------------------------------------------
        // Kernel 5: Select Flying Cloth Cluster
        // -------------------------------------------------------------
        __global__ void kernel_select_cloth(
            const int *__restrict__ cluster_sizes,
            const float3 *__restrict__ cluster_min,
            const float3 *__restrict__ cluster_max,
            const float3 *__restrict__ cluster_sum,
            int count,
            FilterConfig filter,
            float3 pred_pos,
            bool has_prediction,
            ClothTrackingResult *__restrict__ result,
            int *__restrict__ best_root_out)
        {
            if (threadIdx.x == 0 && blockIdx.x == 0)
            {
                result->valid = false;
                result->cx = 0.0f;
                result->cy = 0.0f;
                result->cz = 0.0f;
                result->cluster_points_count = 0;
                result->bbox_dx = 0.0f;
                result->bbox_dy = 0.0f;
                result->bbox_dz = 0.0f;
                *best_root_out = -1;

                int best_count = 0;
                int best_idx = -1;
                float best_score = -1e9f;

                for (int i = 0; i < count; ++i)
                {
                    int sz = cluster_sizes[i];
                    if (sz < filter.min_cluster_points || sz > filter.max_cluster_points)
                    {
                        continue;
                    }

                    float dx = cluster_max[i].x - cluster_min[i].x;
                    float dy = cluster_max[i].y - cluster_min[i].y;
                    float dz = cluster_max[i].z - cluster_min[i].z;

                    // Reject opponent robot or huge noise (> max_cloth_size_m)
                    if (dx > filter.max_cloth_size_m || dy > filter.max_cloth_size_m || dz > filter.max_cloth_size_m)
                    {
                        continue;
                    }

                    float c_cx = cluster_sum[i].x / static_cast<float>(sz);
                    float c_cy = cluster_sum[i].y / static_cast<float>(sz);
                    float c_cz = cluster_sum[i].z / static_cast<float>(sz);

                    float score = 0.0f;
                    if (has_prediction)
                    {
                        float dpx = c_cx - pred_pos.x;
                        float dpy = c_cy - pred_pos.y;
                        float dpz = c_cz - pred_pos.z;
                        float dist = sqrtf(dpx * dpx + dpy * dpy + dpz * dpz);
                        if (dist <= 1.80f)
                        {
                            // Close to prediction: highest priority
                            score = 3000.0f - dist * 1000.0f + static_cast<float>(sz) * 0.1f;
                        }
                        else if (dist <= 3.50f || c_cx >= pred_pos.x - 0.40f)
                        {
                            // Extended gate: never drop the cloth even with sudden drag, high arc, or fluttering
                            score = 1000.0f - dist * 200.0f + static_cast<float>(sz) * 0.05f;
                        }
                        else
                        {
                            continue; // Out of tracking gate
                        }
                    }
                    else
                    {
                        // When waiting for launch: prioritize clusters in launcher/throw zone (X: 0.3~2.5m, Y: -1.2~1.2m, Z: 0.2~2.5m)
                        if (c_cx >= 0.30f && c_cx <= 2.50f &&
                            c_cy >= -1.20f && c_cy <= 1.20f &&
                            c_cz >= 0.20f && c_cz <= 2.50f)
                        {
                            score = 1000.0f - c_cx * 20.0f + static_cast<float>(sz) * 0.1f;
                        }
                        else
                        {
                            // Far/floor noise
                            score = static_cast<float>(sz) * 0.01f;
                        }
                    }

                    if (score > best_score)
                    {
                        best_score = score;
                        best_count = sz;
                        best_idx = i;
                    }
                }

                if (best_idx >= 0)
                {
                    result->valid = true;
                    result->cx = cluster_sum[best_idx].x / static_cast<float>(best_count);
                    result->cy = cluster_sum[best_idx].y / static_cast<float>(best_count);
                    result->cz = cluster_sum[best_idx].z / static_cast<float>(best_count);
                    result->cluster_points_count = best_count;
                    result->bbox_dx = cluster_max[best_idx].x - cluster_min[best_idx].x;
                    result->bbox_dy = cluster_max[best_idx].y - cluster_min[best_idx].y;
                    result->bbox_dz = cluster_max[best_idx].z - cluster_min[best_idx].z;
                    *best_root_out = best_idx;
                }
            }
        }

    } // namespace

    bool init_cuda_tracker(CudaTrackerContext &ctx, int max_depth_width, int max_depth_height)
    {
        if (ctx.initialized)
            return true;

        ctx.max_pixels = max_depth_width * max_depth_height;
        cudaStreamCreate(&ctx.stream);

        cudaMalloc(&ctx.d_depth, ctx.max_pixels * sizeof(uint16_t));
        cudaMalloc(&ctx.d_bg_depth, ctx.max_pixels * sizeof(uint16_t));
        cudaMemset(ctx.d_bg_depth, 0, ctx.max_pixels * sizeof(uint16_t));
        ctx.bg_ready = false;

        cudaMalloc(&ctx.d_valid_points, ctx.max_pixels * sizeof(float3));
        cudaMalloc(&ctx.d_valid_count, sizeof(int));

        cudaMalloc(&ctx.d_voxel_points, CudaTrackerContext::MAX_VOXEL_POINTS * sizeof(float3));
        cudaMalloc(&ctx.d_voxel_count, sizeof(int));
        cudaMalloc(&ctx.d_hash_keys, CudaTrackerContext::HASH_TABLE_SIZE * sizeof(uint64_t));

        cudaMalloc(&ctx.d_parent, CudaTrackerContext::MAX_VOXEL_POINTS * sizeof(int));
        cudaMalloc(&ctx.d_cluster_sizes, CudaTrackerContext::MAX_VOXEL_POINTS * sizeof(int));
        cudaMalloc(&ctx.d_cluster_min, CudaTrackerContext::MAX_VOXEL_POINTS * sizeof(float3));
        cudaMalloc(&ctx.d_cluster_max, CudaTrackerContext::MAX_VOXEL_POINTS * sizeof(float3));
        cudaMalloc(&ctx.d_cluster_sum, CudaTrackerContext::MAX_VOXEL_POINTS * sizeof(float3));

        cudaMalloc(&ctx.d_result, sizeof(ClothTrackingResult));
        cudaMallocHost(&ctx.h_result_pinned, sizeof(ClothTrackingResult));
        cudaMalloc(&ctx.d_best_root, sizeof(int));

        ctx.initialized = true;
        return true;
    }

    void reset_background_cuda(CudaTrackerContext &ctx)
    {
        if (!ctx.initialized || ctx.d_bg_depth == nullptr)
            return;
        cudaMemset(ctx.d_bg_depth, 0, ctx.max_pixels * sizeof(uint16_t));
        ctx.bg_ready = false;
    }

    __global__ void kernel_update_bg(
        const uint16_t *__restrict__ depth,
        uint16_t *__restrict__ bg_depth,
        int total_pixels)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= total_pixels)
            return;
        uint16_t d = depth[idx];
        if (d > 0)
        {
            uint16_t old = bg_depth[idx];
            if (old == 0)
            {
                bg_depth[idx] = d;
            }
            else
            {
                // Running average for stable background calibration
                bg_depth[idx] = static_cast<uint16_t>((old * 3 + d + 2) / 4);
            }
        }
    }


    void update_background_cuda(CudaTrackerContext &ctx, const uint16_t *host_depth_ptr, int width, int height)
    {
        if (!ctx.initialized || host_depth_ptr == nullptr || ctx.d_bg_depth == nullptr)
            return;
        int total_pixels = width * height;
        if (total_pixels > ctx.max_pixels)
            return;

        cudaMemcpyAsync(ctx.d_depth, host_depth_ptr, total_pixels * sizeof(uint16_t), cudaMemcpyHostToDevice, ctx.stream);
        int block = 256;
        int grid = (total_pixels + block - 1) / block;
        kernel_update_bg<<<grid, block, 0, ctx.stream>>>(ctx.d_depth, ctx.d_bg_depth, total_pixels);
        cudaStreamSynchronize(ctx.stream);
        ctx.bg_ready = true;
    }

    void destroy_cuda_tracker(CudaTrackerContext &ctx)
    {
        if (!ctx.initialized)
            return;

        if (ctx.d_depth)
            cudaFree(ctx.d_depth);
        if (ctx.d_bg_depth)
            cudaFree(ctx.d_bg_depth);
        if (ctx.d_valid_points)
            cudaFree(ctx.d_valid_points);
        if (ctx.d_valid_count)
            cudaFree(ctx.d_valid_count);

        if (ctx.d_voxel_points)
            cudaFree(ctx.d_voxel_points);
        if (ctx.d_voxel_count)
            cudaFree(ctx.d_voxel_count);
        if (ctx.d_hash_keys)
            cudaFree(ctx.d_hash_keys);

        if (ctx.d_parent)
            cudaFree(ctx.d_parent);
        if (ctx.d_cluster_sizes)
            cudaFree(ctx.d_cluster_sizes);
        if (ctx.d_cluster_min)
            cudaFree(ctx.d_cluster_min);
        if (ctx.d_cluster_max)
            cudaFree(ctx.d_cluster_max);
        if (ctx.d_cluster_sum)
            cudaFree(ctx.d_cluster_sum);

        if (ctx.d_result)
            cudaFree(ctx.d_result);
        if (ctx.h_result_pinned)
            cudaFreeHost(ctx.h_result_pinned);
        if (ctx.d_best_root)
            cudaFree(ctx.d_best_root);

        if (ctx.stream)
            cudaStreamDestroy(ctx.stream);

        ctx.initialized = false;
    }

    ClothTrackingResult process_depth_frame_cuda(
        CudaTrackerContext &ctx,
        const uint16_t *host_depth_ptr,
        const CameraIntrinsics &intrin,
        const CameraPoseConfig &pose,
        const FilterConfig &filter,
        std::vector<float3> *out_cluster_points,
        std::vector<float3> *out_filtered_points,
        float pred_x,
        float pred_y,
        float pred_z,
        bool has_prediction)
    {
        ClothTrackingResult empty_res;
        if (!ctx.initialized || host_depth_ptr == nullptr)
            return empty_res;

        int total_pixels = intrin.width * intrin.height;
        if (total_pixels > ctx.max_pixels)
            return empty_res;

        // 1. Copy Depth Frame to Device asynchronously
        cudaMemcpyAsync(ctx.d_depth, host_depth_ptr, total_pixels * sizeof(uint16_t), cudaMemcpyHostToDevice, ctx.stream);
        cudaMemsetAsync(ctx.d_valid_count, 0, sizeof(int), ctx.stream);

        // 2. Kernel 1: Back-project & PassThrough with Background Subtraction
        dim3 block2d(32, 16);
        dim3 grid2d((intrin.width + block2d.x - 1) / block2d.x, (intrin.height + block2d.y - 1) / block2d.y);

        kernel_backproject_and_passthrough<<<grid2d, block2d, 0, ctx.stream>>>(
            ctx.d_depth,
            ctx.d_bg_depth,
            filter.use_bg_subtraction && ctx.bg_ready,
            intrin,
            pose,
            filter,
            ctx.d_valid_points,
            ctx.d_valid_count,
            ctx.max_pixels);

        // Check count of valid points
        int h_valid_count = 0;
        cudaMemcpyAsync(&h_valid_count, ctx.d_valid_count, sizeof(int), cudaMemcpyDeviceToHost, ctx.stream);
        cudaStreamSynchronize(ctx.stream);

        if (h_valid_count < filter.min_cluster_points)
        {
            return empty_res;
        }

        int active_valid = (h_valid_count > ctx.max_pixels) ? ctx.max_pixels : h_valid_count;

        // 3. Kernel 2: Voxel Downsample
        cudaMemsetAsync(ctx.d_voxel_count, 0, sizeof(int), ctx.stream);
        cudaMemsetAsync(ctx.d_hash_keys, 0xFF, CudaTrackerContext::HASH_TABLE_SIZE * sizeof(uint64_t), ctx.stream);

        int block1d = 256;
        int grid_voxel = (active_valid + block1d - 1) / block1d;

        kernel_voxel_downsample<<<grid_voxel, block1d, 0, ctx.stream>>>(
            ctx.d_valid_points,
            active_valid,
            filter.voxel_size_m,
            ctx.d_hash_keys,
            ctx.d_voxel_points,
            ctx.d_voxel_count,
            CudaTrackerContext::MAX_VOXEL_POINTS,
            CudaTrackerContext::HASH_TABLE_SIZE);

        int h_voxel_count = 0;
        cudaMemcpyAsync(&h_voxel_count, ctx.d_voxel_count, sizeof(int), cudaMemcpyDeviceToHost, ctx.stream);
        cudaStreamSynchronize(ctx.stream);

        if (h_voxel_count < filter.min_cluster_points)
        {
            return empty_res;
        }

        int num_points = (h_voxel_count > CudaTrackerContext::MAX_VOXEL_POINTS) ? CudaTrackerContext::MAX_VOXEL_POINTS : h_voxel_count;

        // 4. Kernel 3: Clustering (Union-Find)
        int grid_pts = (num_points + block1d - 1) / block1d;
        kernel_init_clusters<<<grid_pts, block1d, 0, ctx.stream>>>(ctx.d_parent, num_points);

        float eps_sq = filter.cluster_eps_m * filter.cluster_eps_m;
        kernel_cluster_union_find<<<grid_pts, block1d, 0, ctx.stream>>>(
            ctx.d_voxel_points, num_points, eps_sq, ctx.d_parent);

        // 5. Kernel 4: Accumulate stats
        kernel_init_cluster_stats<<<grid_pts, block1d, 0, ctx.stream>>>(
            ctx.d_cluster_sizes, ctx.d_cluster_min, ctx.d_cluster_max, ctx.d_cluster_sum, num_points);

        kernel_accumulate_cluster_stats<<<grid_pts, block1d, 0, ctx.stream>>>(
            ctx.d_voxel_points, ctx.d_parent, num_points,
            ctx.d_cluster_sizes, ctx.d_cluster_min, ctx.d_cluster_max, ctx.d_cluster_sum);

        // 6. Kernel 5: Select best cloth cluster (guided by ballistic prediction if tracking)
        float3 pred_pt = make_float3(pred_x, pred_y, pred_z);
        kernel_select_cloth<<<1, 1, 0, ctx.stream>>>(
            ctx.d_cluster_sizes, ctx.d_cluster_min, ctx.d_cluster_max, ctx.d_cluster_sum,
            num_points, filter, pred_pt, has_prediction, ctx.d_result, ctx.d_best_root);

        // Copy result back
        cudaMemcpyAsync(ctx.h_result_pinned, ctx.d_result, sizeof(ClothTrackingResult), cudaMemcpyDeviceToHost, ctx.stream);
        int h_best_root = -1;
        cudaMemcpyAsync(&h_best_root, ctx.d_best_root, sizeof(int), cudaMemcpyDeviceToHost, ctx.stream);
        cudaStreamSynchronize(ctx.stream);

        ClothTrackingResult final_res = *ctx.h_result_pinned;

        // Optional: Extract debug all filtered/voxel points
        std::vector<float3> h_all_voxels;
        if (out_filtered_points || out_cluster_points)
        {
            h_all_voxels.resize(num_points);
            cudaMemcpy(h_all_voxels.data(), ctx.d_voxel_points, num_points * sizeof(float3), cudaMemcpyDeviceToHost);
            if (out_filtered_points)
            {
                *out_filtered_points = h_all_voxels;
            }
        }

        // Optional: Extract debug cluster points
        if (out_cluster_points && h_best_root >= 0)
        {
            out_cluster_points->clear();
            std::vector<int> h_parents(num_points);
            cudaMemcpy(h_parents.data(), ctx.d_parent, num_points * sizeof(int), cudaMemcpyDeviceToHost);

            for (int i = 0; i < num_points; ++i)
            {
                int root = i;
                while (h_parents[root] != root)
                    root = h_parents[root];
                if (root == h_best_root)
                {
                    out_cluster_points->push_back(h_all_voxels[i]);
                }
            }
        }

        return final_res;
    }

} // namespace realsense_dust_cloth
