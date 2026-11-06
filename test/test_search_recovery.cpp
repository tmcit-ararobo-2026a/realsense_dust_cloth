#include "realsense_dust_cloth/ballistic_tracker.hpp"
#include <cassert>
#include <iostream>

using namespace realsense_dust_cloth;

int main() {
    int devices = 0;
    if (cudaGetDeviceCount(&devices) != cudaSuccess || devices == 0) {
        std::cout << "No CUDA device; GPU search recovery test skipped\n";
        return 77;
    }

    CudaTrackerContext context;
    assert(init_cuda_tracker(context, 1, 1));
    BallisticTracker tracker;
    tracker.tube_radius_m = .35;
    tracker.lost_frame_gate_expansion_m = .04;
    tracker.max_gate_expansion_m = .1;
    FilterConfig filter;
    filter.min_tracking_points = 8;
    filter.min_cloth_size_m = .03;
    filter.max_cloth_size_m = .8;
    filter.cluster_eps_m = .1;

    // One connected cloth-sized patch is just outside the nominal prediction gate.
    std::vector<float3> points;
    for (int y = -2; y <= 2; ++y) {
        for (int z = -1; z <= 2; ++z) {
            points.push_back(make_float3(2.38f, y * .02f, 2.f + z * .02f));
        }
    }
    filter.gate_radius_m = tracker.search_radius_m();
    auto nominal = process_point_cloud_cuda(context, points, filter, nullptr, nullptr, 2, 0, 2, true);
    assert(!nominal.valid);

    tracker.lost_count = 1;
    filter.gate_radius_m = tracker.search_radius_m();
    auto recovered = process_point_cloud_cuda(context, points, filter, nullptr, nullptr, 2, 0, 2, true);
    assert(recovered.valid);
    assert(recovered.cluster_points_count == static_cast<int>(points.size()));
    destroy_cuda_tracker(context);
    std::cout << "GPU selects a candidate in the expanded missing-frame gate\n";
}
