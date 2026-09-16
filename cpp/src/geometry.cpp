#include "robotsim/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace robotsim {

namespace {

// Clips the ray parameter interval [t_min, t_max] against one slab [lo, hi].
// Returns false if the interval becomes empty.
bool clip_slab(double origin, double dir, double lo, double hi, double& t_min, double& t_max) {
    if (dir == 0.0) {
        // Ray is parallel to the slab: it is either always inside or never.
        return origin >= lo && origin <= hi;
    }
    const double inv = 1.0 / dir;
    double t1 = (lo - origin) * inv;
    double t2 = (hi - origin) * inv;
    if (t1 > t2) {
        std::swap(t1, t2);
    }
    t_min = std::max(t_min, t1);
    t_max = std::min(t_max, t2);
    return t_min <= t_max;
}

}  // namespace

double length(Vec2 v) {
    return std::sqrt(length_squared(v));
}

double wrap_angle(double angle) {
    double wrapped = std::fmod(angle + kPi, kTwoPi);
    if (wrapped < 0.0) {
        wrapped += kTwoPi;
    }
    if (wrapped >= kTwoPi) {
        // Adding 2*pi to a tiny negative value can round up to exactly 2*pi.
        wrapped = 0.0;
    }
    return wrapped - kPi;
}

Vec2 closest_point(const AABB& box, Vec2 p) {
    return {std::clamp(p.x, box.lo.x, box.hi.x), std::clamp(p.y, box.lo.y, box.hi.y)};
}

bool circle_intersects_aabb(Vec2 center, double radius, const AABB& box) {
    const Vec2 offset = center - closest_point(box, center);
    return length_squared(offset) <= radius * radius;
}

std::optional<double> ray_aabb(Vec2 origin, Vec2 dir, const AABB& box) {
    double t_min = 0.0;  // the ray starts at the origin
    double t_max = std::numeric_limits<double>::infinity();

    if (!clip_slab(origin.x, dir.x, box.lo.x, box.hi.x, t_min, t_max)) {
        return std::nullopt;
    }
    if (!clip_slab(origin.y, dir.y, box.lo.y, box.hi.y, t_min, t_max)) {
        return std::nullopt;
    }
    return t_min;
}

}  // namespace robotsim