#pragma once
#include "Triangle.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

inline std::vector<Triangle> loadOBJ(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) throw std::runtime_error("Cannot open " + filename);
    std::vector<Vector3> vertices;
    std::vector<Triangle> triangles;
    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;
        std::istringstream row(line);
        std::string type;
        row >> type;
        try {
            if (type == "v") {
                Vector3 v{};
                if (!(row >> v.x >> v.y >> v.z) ||
                    !v.finite())
                    throw std::runtime_error("Invalid vertex");
                vertices.push_back(v);
            } else if (type == "f") {
                std::vector<int> face;
                std::string token;
                while (row >> token) {
                    if (token[0] == '#') break;
                    // In a face entry like 3/2/1, the first number selects the vertex.
                    std::string number = token.substr(0, token.find('/'));
                    size_t used = 0;
                    int index = std::stoi(number, &used);
                    if (used != number.size() || index == 0)
                        throw std::runtime_error("Invalid face index");
                    // OBJ indices start at 1; negative indices count back from the latest vertex.
                    index = index > 0 ? index - 1 : static_cast<int>(vertices.size()) + index;
                    if (index < 0 || index >= static_cast<int>(vertices.size()))
                        throw std::runtime_error("Face index out of range");
                    face.push_back(index);
                }
                if (face.size() < 3 || face.size() > 4)
                    throw std::runtime_error("Use triangle or convex quad faces; triangulate larger polygons");
                if (face.size() == 4) {
                    Vector3 a = vertices[face[0]], b = vertices[face[1]];
                    Vector3 c = vertices[face[2]], d = vertices[face[3]];
                    Vector3 normal = (b-a).cross(c-a).normalized();
                    float scale = std::max({(b-a).length(), (c-a).length(), (d-a).length()});
                    if (normal.length() == 0 || std::abs((d-a).dot(normal)) > 0.00001f*scale)
                        throw std::runtime_error("Nonplanar or degenerate quad; triangulate before export");
                    for (int i=0; i<4; ++i) {
                        Vector3 p = vertices[face[i]];
                        Vector3 q = vertices[face[(i+1)%4]];
                        Vector3 r = vertices[face[(i+2)%4]];
                        if ((q-p).cross(r-q).dot(normal) <= 0)
                            throw std::runtime_error("Nonconvex quad; triangulate before export");
                    }
                }
                // A quad becomes two triangles sharing the same diagonal.
                for (size_t i=1; i+1<face.size(); ++i) {
                    Triangle t{vertices[face[0]], vertices[face[i]], vertices[face[i+1]], {0.2f,0.75f,0.4f}};
                    if ((t.b-t.a).cross(t.c-t.a).length() < 0.000001f)
                        throw std::runtime_error("Triangle has zero area; check vertex positions");
                    triangles.push_back(t);
                }
            }
        } catch (const std::exception& e) {
            throw std::runtime_error("OBJ line " + std::to_string(lineNumber) + ": " + e.what());
        }
    }
    if (triangles.empty()) throw std::runtime_error("No triangle faces found");
    return triangles;
}
