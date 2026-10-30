#include "realsense_dust_cloth/map_background.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
using namespace realsense_dust_cloth;

__global__ void evaluate(MapFilterView map,float3 point,float3 prediction,int* result) {
    *result=retain_map_point(map,point,prediction);
}
bool keep(MapFilterView view,float3 point,float3 prediction=make_float3(0,0,0)) {
    int* device=nullptr;assert(cudaMalloc(&device,sizeof(int))==cudaSuccess);
    evaluate<<<1,1>>>(view,point,prediction,device);
    assert(cudaDeviceSynchronize()==cudaSuccess);
    int result=0;assert(cudaMemcpy(&result,device,sizeof(int),cudaMemcpyDeviceToHost)==cudaSuccess);
    assert(cudaFree(device)==cudaSuccess);return result;
}
template<class F> void rejects(F f) { bool rejected=false;try { f(); } catch(const std::exception&) {rejected=true;} assert(rejected); }
int main() {
    int count=0;if (cudaGetDeviceCount(&count)!=cudaSuccess || count==0) return 77;
    auto folder=std::filesystem::temp_directory_path()/("cloth-map-test-"+std::to_string(getpid()));
    std::filesystem::create_directories(folder);
    auto json=folder/"test.json";
    std::ofstream(json) << R"([{"type":"BOX","x":1,"y":0,"z_min":0,"z_max":2,"param1":0.1,"param2":0.1}])";
    MapBuildConfig config;config.min_x=-2;config.max_x=4;config.min_y=-2;config.max_y=2;config.min_z=-1;config.max_z=3;
    MapBackground map;map.load(json.string(),config);auto view=map.view();view.use_floor=false;
    assert(!keep(view,make_float3(1.1f,0,1))); // known box surface
    assert(keep(view,make_float3(0.5f,0,1)));  // independently known free space
    view.transform[3]=0.6f;
    assert(!keep(view,make_float3(0.5f,0,1))); // same surface after translation
    view=map.view();view.use_floor=false;
    view.transform[0]=0;view.transform[1]=-1;view.transform[4]=1;view.transform[5]=0;
    view.transform[3]=1.1f;view.transform[7]=-1;
    assert(!keep(view,make_float3(1,0,1))); // 90-degree yaw and translation
    view=map.view();view.use_floor=false;
    assert(!keep(view,make_float3(8,0,1)));
    view.reject_unknown=false;assert(keep(view,make_float3(8,0,1)));
    view.reject_unknown=true;view.protect_tracking=true;
    assert(keep(view,make_float3(1.1f,0,1),make_float3(1.1f,0,1)));
    assert(!keep(view,make_float3(1.1f,0,1),make_float3(0.5f,0,1)));
    view=map.view();view.use_floor=true;
    assert(!keep(view,make_float3(0.5f,0,0.01f)));
    view.protect_tracking=true;
    assert(keep(view,make_float3(0.5f,0,0.01f),make_float3(0.5f,0,0.01f)));
    std::ofstream(json) << R"([{"type":"BOX","x":1,"y":0,"z_min":0,"z_max":2,"half_x":0.1,"half_y":0.1},
        {"type":"CYLINDER","x":2,"y":0,"z_min":0,"z_max":2,"radius":0.1},
        {"type":"ZONE"},{"type":"PENALTY_ZONE"}])";
    MapBackground canonical;canonical.load(json.string(),config);
    auto canonical_view=canonical.view();canonical_view.use_floor=false;
    assert(!keep(canonical_view,make_float3(1.1f,0,1)));
    assert(!keep(canonical_view,make_float3(2.1f,0,1)));
    assert(keep(canonical_view,make_float3(0.5f,0,1)));
    auto file=folder/"test.esdf";
    MapGridHeader header{0.1f,2,2,2,-1,-1,-1,0.5f};float distances[8]{};
    {std::ofstream stream(file,std::ios::binary);stream.write(reinterpret_cast<const char*>(&header),sizeof(header));stream.write(reinterpret_cast<const char*>(distances),sizeof(distances));}
    MapBackground binary;binary.load(file.string(),config);auto imported=binary.view();imported.use_floor=false;
    assert(!keep(imported,make_float3(-0.95f,-0.95f,-0.95f)));
    std::ofstream(file,std::ios::binary|std::ios::trunc) << "truncated";
    rejects([&] {MapBackground bad;bad.load(file.string(),config);});
    MapBuildConfig tiny=config;tiny.max_bytes=1;
    rejects([&] {MapBackground bad;bad.load(json.string(),tiny);});
    std::ofstream(json) << R"([{"type":"SPHERE"}])";
    rejects([&] {MapBackground bad;bad.load(json.string(),config);});
    std::filesystem::remove_all(folder);
    std::cout << "Map JSON/ESDF, world transforms, unknown area, floor/tracking protection and invalid-file tests passed\n";
}
