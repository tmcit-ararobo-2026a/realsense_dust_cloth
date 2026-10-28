#pragma once

#include <cuda_runtime.h>
#include <cstdint>
#include <string>
#include <vector>

namespace realsense_dust_cloth {

// Binary layout compatible with gn10-pointcloud-localization ESDF files.
struct MapGridHeader {
    float resolution{0};
    int32_t size_x{0}, size_y{0}, size_z{0};
    float min_x{0}, min_y{0}, min_z{0}, max_distance{0};
};
static_assert(sizeof(MapGridHeader) == 32, "Unexpected ESDF binary layout");

struct MapBuildConfig {
    float resolution{0.05f}, max_distance{0.5f};
    float min_x{-5.5f}, max_x{5.5f}, min_y{-6.0f}, max_y{6.0f};
    float min_z{-0.2f}, max_z{4.0f};
    size_t max_bytes{128 * 1024 * 1024};
};

struct MapFilterView {
    cudaTextureObject_t texture{0};
    MapGridHeader grid;
    // Row-major map <- base_link, including full rotation and translation.
    float transform[12]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
    float exclusion_distance{0.08f};
    float tracking_keep_radius{0.3f};
    float floor_z{0};
    bool enabled{false}, reject_unknown{true}, use_floor{true};
    bool protect_tracking{false};
};

// Each node owns its map. No process-global CUDA textures or pose search.
class MapBackground {
public:
    MapBackground() = default;
    ~MapBackground();
    MapBackground(const MapBackground&) = delete;
    MapBackground& operator=(const MapBackground&) = delete;
    void load(const std::string& path, const MapBuildConfig& config);
    MapFilterView view() const;
    size_t bytes() const { return bytes_; }
private:
    cudaArray_t array_{nullptr};
    cudaTextureObject_t texture_{0};
    MapGridHeader grid_;
    size_t bytes_{0};
};

#ifdef __CUDACC__
__device__ inline bool retain_map_point(const MapFilterView& map, float3 base,
                                        float3 prediction) {
    if (!map.enabled) return true;
    const float* t = map.transform;
    float x = t[0]*base.x + t[1]*base.y + t[2]*base.z + t[3];
    float y = t[4]*base.x + t[5]*base.y + t[6]*base.z + t[7];
    float z = t[8]*base.x + t[9]*base.y + t[10]*base.z + t[11];
    const auto& g = map.grid;
    float u = (x-g.min_x)/g.resolution, v = (y-g.min_y)/g.resolution;
    float w = (z-g.min_z)/g.resolution;
    bool known = isfinite(u) && isfinite(v) && isfinite(w) &&
        u >= 0 && v >= 0 && w >= 0 && u < g.size_x && v < g.size_y && w < g.size_z;
    if (!known) return !map.reject_unknown;
    // Keep the last part of a confirmed flight near a wall/floor. This small
    // region is still constrained by the existing motion/size/ballistic gates.
    if (map.protect_tracking) {
        float dx=base.x-prediction.x, dy=base.y-prediction.y, dz=base.z-prediction.z;
        if (dx*dx+dy*dy+dz*dz <= map.tracking_keep_radius*map.tracking_keep_radius) return true;
    }
    // Grid samples are at min+(index+0.5)*resolution, i.e. texel centers.
    float distance = tex3D<float>(map.texture, u, v, w);
    if (map.use_floor) distance = fminf(distance, fabsf(z-map.floor_z));
    return isfinite(distance) && distance >= map.exclusion_distance;
}
#endif
}  // namespace realsense_dust_cloth
