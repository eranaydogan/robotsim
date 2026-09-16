#pragma once

#include <optional>

namespace robotsim {

inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kTwoPi = 2.0 * kPi;

// ---------------------------------------------------------------------------
// 2D vector
// ---------------------------------------------------------------------------
struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

constexpr Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
constexpr Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
constexpr Vec2 operator*(Vec2 v, double s) { return {v.x * s, v.y * s}; }
constexpr Vec2 operator*(double s, Vec2 v) { return {v.x * s, v.y * s}; }

constexpr double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
constexpr double length_squared(Vec2 v) { return dot(v, v); }
double length(Vec2 v);

// ---------------------------------------------------------------------------
// Axis-aligned bounding box
// Corners are named lo/hi rather than min/max to avoid clashes with the
// min/max macros defined by Windows headers.
// ---------------------------------------------------------------------------
struct AABB {
    Vec2 lo{};
    Vec2 hi{};
};

// ---------------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------------

// Wraps an angle in radians to [-pi, pi).
double wrap_angle(double angle);

// Closest point on (or inside) the box to p. Returns p if p is inside.
Vec2 closest_point(const AABB& box, Vec2 p);

// True if the circle touches or overlaps the box (distance <= radius).
bool circle_intersects_aabb(Vec2 center, double radius, const AABB& box);

// Casts a ray from origin along dir, which must be a unit vector.
// Returns the distance to the first intersection with the box, 0 if the
// origin is inside the box, or std::nullopt if the ray misses.
// Touching an edge or a corner counts as a hit.
std::optional<double> ray_aabb(Vec2 origin, Vec2 dir, const AABB& box);

}  // namespace robotsim