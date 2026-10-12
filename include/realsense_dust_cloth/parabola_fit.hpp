#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/msg/point.hpp>

namespace realsense_dust_cloth {

/**
 * @brief Represents a fitted ballistic parabola and lateral straight line.
 *        Z(X) = a*X^2 + b*X + c (vertical parabolic flight)
 *        Y(X) = d*X + e         (lateral straight line)
 */
struct ParabolaFit {
    bool valid{false};
    double a{0.0}, b{0.0}, c{0.0};
    double d{0.0}, e{0.0};
    double x_start{0.0};
    double x_apex{0.0}, z_apex{0.0};
    double x_land{0.0}, y_land{0.0}, z_land{0.0};
    double r2{0.0};
    std::vector<geometry_msgs::msg::Point> curve_pts;
};

/**
 * @brief Interpolates 3D points using Catmull-Rom spline.
 */
inline geometry_msgs::msg::Point interpolate_catmull_rom(
    const geometry_msgs::msg::Point& p0,
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2,
    const geometry_msgs::msg::Point& p3,
    double t)
{
    double t2 = t * t;
    double t3 = t2 * t;

    geometry_msgs::msg::Point pt;
    pt.x = 0.5 * ((2.0 * p1.x) +
                  (-p0.x + p2.x) * t +
                  (2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x) * t2 +
                  (-p0.x + 3.0 * p1.x - 3.0 * p2.x + p3.x) * t3);
    pt.y = 0.5 * ((2.0 * p1.y) +
                  (-p0.y + p2.y) * t +
                  (2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y) * t2 +
                  (-p0.y + 3.0 * p1.y - 3.0 * p2.y + p3.y) * t3);
    pt.z = 0.5 * ((2.0 * p1.z) +
                  (-p0.z + p2.z) * t +
                  (2.0 * p0.z - 5.0 * p1.z + 4.0 * p2.z - p3.z) * t2 +
                  (-p0.z + 3.0 * p1.z - 3.0 * p2.z + p3.z) * t3);
    return pt;
}

/**
 * @brief Solves the 3x3 least squares system for parabolic fit using Cramer's rule.
 */
inline ParabolaFit compute_most_probable_parabola(
    const std::vector<std::pair<double, geometry_msgs::msg::Point>>& raw_pts,
    double target_landing_z = 0.0)
{
    ParabolaFit res;
    if (raw_pts.size() < 5) return res;

    double sx4 = 0.0, sx3 = 0.0, sx2 = 0.0, sx1 = 0.0, s_n = 0.0;
    double sx2z = 0.0, sxz = 0.0, sz = 0.0;
    double sxy = 0.0, sy = 0.0;
    double z_mean_acc = 0.0;

    for (const auto& item : raw_pts) {
        double x = item.second.x;
        double y = item.second.y;
        double z = item.second.z;

        double x2 = x * x;
        double x3 = x2 * x;
        double x4 = x3 * x;

        sx4 += x4;
        sx3 += x3;
        sx2 += x2;
        sx1 += x;
        s_n += 1.0;

        sx2z += x2 * z;
        sxz += x * z;
        sz += z;

        sxy += x * y;
        sy += y;
        z_mean_acc += z;
    }

    // Solve 3x3 quadratic system for Z(X) = a*X^2 + b*X + c using Cramer's Rule
    double D = sx4 * (sx2 * s_n - sx1 * sx1) - sx3 * (sx3 * s_n - sx1 * sx2) + sx2 * (sx3 * sx1 - sx2 * sx2);
    if (std::abs(D) < 1e-9) return res;

    double Da = sx2z * (sx2 * s_n - sx1 * sx1) - sx3 * (sxz * s_n - sx1 * sz) + sx2 * (sxz * sx1 - sx2 * sz);
    double Db = sx4 * (sxz * s_n - sx1 * sz) - sx2z * (sx3 * s_n - sx1 * sx2) + sx2 * (sx3 * sz - sxz * sx2);
    double Dc = sx4 * (sx2 * sz - sxz * sx1) - sx3 * (sx3 * sz - sxz * sx2) + sx2z * (sx3 * sx1 - sx2 * sx2);

    double a = Da / D;
    double b = Db / D;
    double c = Dc / D;

    // Check concave downward (a < 0) for ballistic arc
    if (a >= 0.0) return res;

    // Solve 2x2 linear system for Y(X) = d*X + e
    double D_line = sx2 * s_n - sx1 * sx1;
    double d = (std::abs(D_line) > 1e-9) ? ((sxy * s_n - sx1 * sy) / D_line) : 0.0;
    double e = (std::abs(D_line) > 1e-9) ? ((sx2 * sy - sx1 * sxy) / D_line) : 0.0;

    res.a = a; res.b = b; res.c = c;
    res.d = d; res.e = e;
    res.x_start = raw_pts.front().second.x;

    // Apex (最高到達点)
    res.x_apex = -b / (2.0 * a);
    res.z_apex = a * res.x_apex * res.x_apex + b * res.x_apex + c;

    // Landing calculation (solve a*X^2 + b*X + (c - target_landing_z) = 0)
    double disc = b * b - 4.0 * a * (c - target_landing_z);
    if (disc >= 0.0) {
        double r1 = (-b + std::sqrt(disc)) / (2.0 * a);
        double r2 = (-b - std::sqrt(disc)) / (2.0 * a);
        res.x_land = std::max(r1, r2);
    } else {
        res.x_land = raw_pts.back().second.x + 0.3;
    }
    res.y_land = d * res.x_land + e;
    res.z_land = target_landing_z;

    // R^2 Confidence Score
    double z_mean = z_mean_acc / s_n;
    double ss_tot = 0.0, ss_res = 0.0;
    for (const auto& item : raw_pts) {
        double x = item.second.x;
        double z = item.second.z;
        double z_pred = a * x * x + b * x + c;
        ss_tot += (z - z_mean) * (z - z_mean);
        ss_res += (z - z_pred) * (z - z_pred);
    }
    res.r2 = (ss_tot > 1e-6) ? std::clamp(1.0 - ss_res / ss_tot, 0.0, 1.0) : 0.0;

    // Generate 150 smooth points along the mathematical parabola from launch to landing!
    double x_min = std::min(res.x_start, 0.35);
    double x_max = std::max(res.x_land, raw_pts.back().second.x);
    x_max = std::min(x_max, 6.0);
    int num_steps = 150;
    double dx = (x_max - x_min) / static_cast<double>(num_steps);

    for (int i = 0; i <= num_steps; ++i) {
        double px = x_min + i * dx;
        double py = d * px + e;
        double pz = a * px * px + b * px + c;
        if (pz < 0.0) pz = 0.0;
        geometry_msgs::msg::Point pt;
        pt.x = px;
        pt.y = py;
        pt.z = pz;
        res.curve_pts.push_back(pt);
    }

    res.valid = true;
    return res;
}

} // namespace realsense_dust_cloth
