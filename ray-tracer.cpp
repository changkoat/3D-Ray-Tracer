#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Vector3.hpp"
#include "Triangle.hpp"
#include "file_util.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

const float EPSILON = 0.001f;
const int MAX_BOUNCES = 3;
const float PI = 3.14159265f;

struct Ray { Vector3 origin, direction; };
struct Sphere { Vector3 center; float radius; Vector3 color; float reflectivity = 0; };
struct Hit {
    float distance;
    Vector3 position, normal, color;
    float reflectivity = 0;
};
struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Triangle> triangles;
};

bool hitSphere(const Ray& ray, const Sphere& sphere, float limit, Hit& hit) {
    if (!ray.origin.finite() || !ray.direction.finite() || !sphere.center.finite() ||
        !std::isfinite(sphere.radius) || sphere.radius <= 0) return false;

    Vector3 offset = ray.origin - sphere.center;
    float a = ray.direction.dot(ray.direction);
    if (!std::isfinite(a) || a <= 0) return false;
    float halfB = offset.dot(ray.direction);
    float c = offset.dot(offset) - sphere.radius*sphere.radius;
    float discriminant = halfB*halfB - a*c;
    if (!std::isfinite(discriminant) || discriminant < 0) return false;
    float sqrtD = std::sqrt(discriminant);
    // Try the nearer sphere hit first, then the far side if needed.
    float root = (-halfB - sqrtD)/a;

    if (!std::isfinite(root) || root <= EPSILON || root >= limit) {
        root = (-halfB + sqrtD)/a;
        if (!std::isfinite(root) || root <= EPSILON || root >= limit) return false;
    }
    hit.distance = root;
    hit.position = ray.origin + ray.direction*root;
    hit.normal = (hit.position - sphere.center).normalized();
    hit.color = sphere.color;
    hit.reflectivity = sphere.reflectivity;
    return hit.position.finite() && hit.normal.finite();
}

bool hitTriangle(const Ray& ray, const Triangle& tri, float limit, Hit& hit) {
    if (!ray.origin.finite() || !ray.direction.finite() ||
        !tri.a.finite() || !tri.b.finite() || !tri.c.finite()) return false;

    Vector3 edge1 = tri.b-tri.a, edge2 = tri.c-tri.a;
    Vector3 p = ray.direction.cross(edge2);
    float determinant = edge1.dot(p);
    if (!std::isfinite(determinant) || std::abs(determinant) < 0.000001f) return false;
    float inverse = 1.0f/determinant;
    Vector3 fromA = ray.origin-tri.a;
    // u and v locate the hit inside the triangle if both are positive or zero and their sum is at most 1
    float u = fromA.dot(p)*inverse;
    if (!std::isfinite(u) || u < 0 || u > 1) return false;
    Vector3 q = fromA.cross(edge1);
    float v = ray.direction.dot(q)*inverse;
    if (!std::isfinite(v) || v < 0 || u+v > 1) return false;
    float t = edge2.dot(q)*inverse;

    if (!std::isfinite(t) || t <= EPSILON || t >= limit) return false;
    hit = {t, ray.origin+ray.direction*t, edge1.cross(edge2).normalized(), tri.color, tri.reflectivity};
    return hit.position.finite() && hit.normal.finite();
}

bool hitScene(const Ray& ray, const Scene& scene, float limit, Hit& nearest) {
    bool found = false;
    Hit candidate{};

    for (const auto& sphere : scene.spheres) {
        if (hitSphere(ray, sphere, limit, candidate)) {
            nearest = candidate; limit = candidate.distance; found = true;
        }
    }
    for (const auto& triangle : scene.triangles) {
        if (hitTriangle(ray, triangle, limit, candidate)) {
            nearest = candidate; limit = candidate.distance; found = true;
        }
    }
    if (found && nearest.normal.dot(ray.direction) > 0)
        nearest.normal = nearest.normal * -1.0f;
    return found;
}

Vector3 trace(const Ray& ray, const Scene& scene, int bouncesLeft = MAX_BOUNCES) {
    Hit hit{};
    if (!hitScene(ray, scene, 1e30f, hit)) {
        float blend = 0.5f*(ray.direction.y+1.0f);
        return Vector3{0.85f,0.91f,1.0f}*(1-blend) + Vector3{0.28f,0.48f,0.75f}*blend;
    }
    Vector3 lightPosition{-3,6,4};

    // Offset the ray to avoid self-shadowing.
    Vector3 surfaceOrigin = hit.position + hit.normal*EPSILON;
    Vector3 toLight = lightPosition-surfaceOrigin;
    Vector3 lightDirection = toLight.normalized();
    Hit blocker{};

    // Only objects between this point and the light can block it.
    bool shadow = hitScene({surfaceOrigin, lightDirection}, scene, toLight.length(), blocker);
    Vector3 result = hit.color*0.14f;
    if (!shadow) {
        float diffuse = std::max(0.0f, hit.normal.dot(lightDirection));
        result = result + hit.color*(0.8f*diffuse);

        Vector3 halfway = (lightDirection-ray.direction).normalized();
        float specular = diffuse > 0 ? std::pow(std::max(0.0f, hit.normal.dot(halfway)), 48.0f) : 0;
        result = result + Vector3{1,1,1}*(0.25f*specular);
    }
    float reflection = std::max(0.0f, std::min(hit.reflectivity, 1.0f));
    if (reflection > 0 && bouncesLeft > 0) {

        // Flip the part of the ray pointing into the surface to get the reflection.
        Vector3 direction = ray.direction - hit.normal*(2*ray.direction.dot(hit.normal));
        Ray reflectedRay{surfaceOrigin, direction.normalized()};
        Vector3 reflectedColor = trace(reflectedRay, scene, bouncesLeft-1);
        result = result*(1-reflection) + reflectedColor*reflection;
    }
    return result;
}

int main(int argc, char** argv) {
    try {
        Scene scene;

        Vector3 floorColor{0.62f,0.65f,0.69f};
        scene.triangles.push_back({{-6,0,-6},{6,0,6},{6,0,-6},floorColor,0.35f});
        scene.triangles.push_back({{-6,0,-6},{-6,0,6},{6,0,6},floorColor,0.35f});

        if (argc > 2) throw std::runtime_error("Usage: ./ray-tracer [model.obj]");
        if (argc == 2) {
            auto mesh = loadOBJ(argv[1]);
            // Center the model and fit it above the floor.
            Vector3 low = mesh.front().a, high = low;
            for (const auto& face : mesh) {
                for (Vector3 p : {face.a, face.b, face.c}) {
                    low = {std::min(low.x,p.x), std::min(low.y,p.y), std::min(low.z,p.z)};
                    high = {std::max(high.x,p.x), std::max(high.y,p.y), std::max(high.z,p.z)};
                }
            }
            Vector3 size = high - low;
            float scale = 2.5f / std::max({size.x, size.y, size.z});
            Vector3 base{low.x + size.x/2, low.y, low.z + size.z/2};
            for (auto face : mesh) {
                for (auto point : {&face.a, &face.b, &face.c})
                    *point = (*point-base)*scale + Vector3{0, 0.01f, 0};
                scene.triangles.push_back(face);
            }
            std::cout << "Loaded " << mesh.size() << " triangle(s) from " << argv[1] << '\n';
        } else {
            scene.spheres.push_back({{-1.1f,0.8f,0},0.8f,{0.85f,0.18f,0.12f},0.1f});
            scene.spheres.push_back({{0.9f,0.65f,0.6f},0.65f,{0.12f,0.42f,0.85f},0.45f});
            scene.triangles.push_back({{-0.4f,0,-1.2f},{1.5f,0,-1.2f},{0.5f,2.1f,-1.2f},{0.3f,0.75f,0.28f}});
        }
        const int width = 640, height = 480;
        const int samples = 2;
        Vector3 camera{4,3,6}, lookAt{0,0.9f,0}, worldUp{0,1,0};
        Vector3 forward = (lookAt-camera).normalized();
        Vector3 right = forward.cross(worldUp).normalized();
        Vector3 up = right.cross(forward).normalized();
        float fieldOfView = 45;
        float scale = std::tan(fieldOfView*PI/360.0f);
        float aspect = static_cast<float>(width) / height;
        std::vector<unsigned char> pixels(width*height*3);
        for (int y=0; y<height; ++y) {
            for (int x=0; x<width; ++x) {
                Vector3 color{0,0,0};
                // Sample four spots per pixel to smooth the edges.
                for (int sy=0; sy<samples; ++sy) for (int sx=0; sx<samples; ++sx) {
                    float screenX = (2*(x+(sx+0.5f)/samples)/width-1)*aspect*scale;
                    float screenY = (1-2*(y+(sy+0.5f)/samples)/height)*scale;
                    Ray ray{camera, (forward+right*screenX+up*screenY).normalized()};
                    color = color + trace(ray,scene);
                }
                color = color*(1.0f/(samples*samples));
                int index = (y*width+x)*3;
                auto channel = [](float value) {
                    return static_cast<unsigned char>(255*std::sqrt(std::max(0.0f,std::min(value,1.0f)))+0.5f);
                };
                pixels[index] = channel(color.x);
                pixels[index+1] = channel(color.y);
                pixels[index+2] = channel(color.z);
            }
        }
        if (!stbi_write_png("image.png",width,height,3,pixels.data(),width*3))
            throw std::runtime_error("Could not write image.png");
        std::cout << "Saved image.png (" << width << " x " << height << ")\n";
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
