#include "realsense_dust_cloth/map_background.hpp"
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace realsense_dust_cloth {
namespace {
void checked(cudaError_t status, const char* operation) {
    if (status != cudaSuccess) throw std::runtime_error(std::string(operation)+": "+cudaGetErrorString(status));
}
size_t count_grid(const MapGridHeader& g, size_t max_bytes) {
    if (!(std::isfinite(g.resolution) && g.resolution > 0 &&
          std::isfinite(g.max_distance) && g.max_distance > 0 &&
          std::isfinite(g.min_x) && std::isfinite(g.min_y) && std::isfinite(g.min_z)) ||
          g.size_x <= 0 || g.size_y <= 0 || g.size_z <= 0) {
        throw std::runtime_error("Invalid ESDF header");
    }
    size_t count=1;
    for (int n : {g.size_x,g.size_y,g.size_z}) {
        if (count > max_bytes/sizeof(float)/static_cast<size_t>(n))
            throw std::runtime_error("ESDF exceeds map_max_memory_mb");
        count *= n;
    }
    return count;
}
struct Object { std::string type; float x,y,z0,z1,a,b; };
float read_number(const cv::FileNode& n, const char* key) {
    auto value=n[key];
    if (value.empty() || !(value.isInt() || value.isReal()))
        throw std::runtime_error(std::string("Map object missing numeric field: ")+key);
    float x=static_cast<float>(static_cast<double>(value));
    if (!std::isfinite(x)) throw std::runtime_error("Map contains non-finite geometry");
    return x;
}
float object_distance(const Object& o, float x, float y, float z) {
    float dz=std::abs(z-(o.z0+o.z1)*0.5f)-(o.z1-o.z0)*0.5f;
    if (o.type=="CYLINDER") {
        float radial=std::hypot(x-o.x,y-o.y)-o.a;
        // Match the existing gn10 cylinder volume-distance convention.
        return std::hypot(std::max(radial,0.0f),std::max(dz,0.0f));
    }
    float dx=std::abs(x-o.x)-o.a, dy=std::abs(y-o.y)-o.b;
    float outside=std::sqrt(std::pow(std::max(dx,0.0f),2)+
        std::pow(std::max(dy,0.0f),2)+std::pow(std::max(dz,0.0f),2));
    return std::abs(outside+std::min(std::max({dx,dy,dz}),0.0f));
}
std::vector<float> read_json(const std::string& path, const MapBuildConfig& c, MapGridHeader& g) {
    for (float x : {c.min_x,c.max_x,c.min_y,c.max_y,c.min_z,c.max_z,c.resolution,c.max_distance})
        if (!std::isfinite(x)) throw std::runtime_error("Non-finite map bounds");
    if (!(c.min_x<c.max_x && c.min_y<c.max_y && c.min_z<c.max_z && c.resolution>0 && c.max_distance>0))
        throw std::runtime_error("Invalid map bounds/resolution");
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open map: "+path);
    std::stringstream text; text << input.rdbuf();
    // OpenCV supports JSON objects at the root; gn10 uses an array of objects.
    cv::FileStorage storage("{\"objects\":"+text.str()+"}",
        cv::FileStorage::READ|cv::FileStorage::MEMORY|cv::FileStorage::FORMAT_JSON);
    auto array=storage["objects"];
    if (!array.isSeq()) throw std::runtime_error("Expected gn10 JSON object array");
    std::vector<Object> objects;
    for (const auto& n : array) {
        std::string type; n["type"] >> type;
        if (type=="VISUAL_BOX") continue;
        if (type!="BOX" && type!="CYLINDER") throw std::runtime_error("Unsupported map type: "+type);
        Object o{type,read_number(n,"x"),read_number(n,"y"),read_number(n,"z_min"),
            read_number(n,"z_max"),read_number(n,"param1"),read_number(n,"param2")};
        if (!(o.z0<o.z1 && o.a>0 && (type!="BOX" || o.b>0)))
            throw std::runtime_error("Invalid map object dimensions");
        objects.push_back(o);
    }
    if (objects.empty()) throw std::runtime_error("Map contains no usable static objects");
    auto dimension=[&](float lo,float hi) {
        double n=std::ceil((double(hi)-lo)/c.resolution);
        if (!(n>0 && n<=std::numeric_limits<int32_t>::max())) throw std::runtime_error("Invalid grid dimensions");
        return static_cast<int32_t>(n);
    };
    g={c.resolution,dimension(c.min_x,c.max_x),dimension(c.min_y,c.max_y),dimension(c.min_z,c.max_z),
       c.min_x,c.min_y,c.min_z,c.max_distance};
    std::vector<float> distances(count_grid(g,c.max_bytes),c.max_distance);
    for (const auto& o : objects) {
        float bx=o.a, by=o.type=="BOX" ? o.b : o.a;
        auto lower=[&](float value,float origin) { return int(std::floor((value-origin)/c.resolution)); };
        int x0=std::max(0,lower(o.x-bx-c.max_distance,c.min_x));
        int x1=std::min(g.size_x-1,lower(o.x+bx+c.max_distance,c.min_x));
        int y0=std::max(0,lower(o.y-by-c.max_distance,c.min_y));
        int y1=std::min(g.size_y-1,lower(o.y+by+c.max_distance,c.min_y));
        int z0=std::max(0,lower(o.z0-c.max_distance,c.min_z));
        int z1=std::min(g.size_z-1,lower(o.z1+c.max_distance,c.min_z));
        for (int z=z0;z<=z1;++z) for (int y=y0;y<=y1;++y) for (int x=x0;x<=x1;++x) {
            size_t i=(size_t(z)*g.size_y+y)*g.size_x+x;
            distances[i]=std::min(distances[i],object_distance(o,c.min_x+(x+0.5f)*c.resolution,
                c.min_y+(y+0.5f)*c.resolution,c.min_z+(z+0.5f)*c.resolution));
        }
    }
    return distances;
}
}  // namespace

MapBackground::~MapBackground() {
    if (texture_) cudaDestroyTextureObject(texture_);
    if (array_) cudaFreeArray(array_);
}
void MapBackground::load(const std::string& path,const MapBuildConfig& config) {
    if (texture_) throw std::runtime_error("Map is already loaded");
    std::vector<float> distances;
    if (std::filesystem::path(path).extension()==".json") {
        distances=read_json(path,config,grid_);
    } else if (std::filesystem::path(path).extension()==".esdf") {
        std::ifstream input(path,std::ios::binary|std::ios::ate);
        if (!input) throw std::runtime_error("Cannot open ESDF: "+path);
        auto size=input.tellg(); input.seekg(0); input.read(reinterpret_cast<char*>(&grid_),sizeof(grid_));
        if (!input) throw std::runtime_error("Truncated ESDF header");
        size_t count=count_grid(grid_,config.max_bytes);
        if (size!=static_cast<std::streamoff>(sizeof(grid_)+count*sizeof(float)))
            throw std::runtime_error("ESDF file length does not match its header");
        distances.resize(count);
        input.read(reinterpret_cast<char*>(distances.data()),count*sizeof(float));
        if (!input) throw std::runtime_error("Truncated ESDF data");
        for (float d : distances) if (!std::isfinite(d) || d<0) throw std::runtime_error("Invalid ESDF distance");
    } else throw std::runtime_error("Map must be gn10 .json or .esdf");
    bytes_=distances.size()*sizeof(float);
    cudaExtent extent=make_cudaExtent(grid_.size_x,grid_.size_y,grid_.size_z);
    auto channel=cudaCreateChannelDesc<float>();
    checked(cudaMalloc3DArray(&array_,&channel,extent),"Allocate map texture");
    cudaMemcpy3DParms copy{};
    copy.srcPtr=make_cudaPitchedPtr(distances.data(),grid_.size_x*sizeof(float),grid_.size_x,grid_.size_y);
    copy.dstArray=array_;copy.extent=extent;copy.kind=cudaMemcpyHostToDevice;
    checked(cudaMemcpy3D(&copy),"Upload map texture");
    cudaResourceDesc resource{}; resource.resType=cudaResourceTypeArray; resource.res.array.array=array_;
    cudaTextureDesc sampler{};
    sampler.addressMode[0]=sampler.addressMode[1]=sampler.addressMode[2]=cudaAddressModeClamp;
    sampler.filterMode=cudaFilterModeLinear;sampler.readMode=cudaReadModeElementType;
    checked(cudaCreateTextureObject(&texture_,&resource,&sampler,nullptr),"Create map texture");
}
MapFilterView MapBackground::view() const {
    MapFilterView view; view.texture=texture_;view.grid=grid_;view.enabled=texture_!=0;return view;
}
}  // namespace realsense_dust_cloth
