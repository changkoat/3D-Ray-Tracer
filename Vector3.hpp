#pragma once
#include <cmath>

struct Vector3 {
    float x, y, z;

    Vector3 operator+(Vector3 b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vector3 operator-(Vector3 b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vector3 operator*(float s) const { return {x*s, y*s, z*s}; }
    float dot(Vector3 b) const { return x*b.x + y*b.y + z*b.z; }
    Vector3 cross(Vector3 b) const {
        return {y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x};
    }
    bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
    float length() const { return std::sqrt(dot(*this)); }
    Vector3 normalized() const {
        float size = length();
        return size > 0 ? *this * (1.0f/size) : Vector3{0,0,0};
    }
};
