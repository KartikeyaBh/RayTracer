#include "triangleface.h"

TriangleFace::TriangleFace(glm::vec3 a_, glm::vec3 b_, glm::vec3 c_, glm::vec3 a_n_, glm::vec3 b_n_, glm::vec3 c_n_, glm::vec2 a_uv_, glm::vec2 b_uv_, glm::vec2 c_uv_, bool use_per_vertex_normals_) :
    a(a_), b(b_), c(c_), a_n(a_n_), b_n(b_n_), c_n(c_n_), a_uv(a_uv_), b_uv(b_uv_), c_uv(c_uv_), use_per_vertex_normals(use_per_vertex_normals_)
{
    local_bbox.reset(new BoundingBox(glm::min(a,glm::min(b,c)),glm::max(a,glm::max(b,c))));
}

bool TriangleFace::IntersectLocal(const Ray &r, Intersection &i)
{
   // REQUIREMENT: Add triangle intersection code here.
   // it currently ignores all triangles and just returns false.
   //
   // Note that you are only intersecting a single triangle, and the vertices
   // of the triangle are supplied to you by the trimesh class.
   //
   // use_per_vertex_normals tells you if the triangle has per-vertex normals.
   // If it does, you should compute and use the Phong-interpolated normal at the intersection point.
   // If it does not, you should use the normal of the triangle's supporting plane.
   //
   // If the ray r intersects the triangle abc:
   // 1. put the hit parameter in i.t
   // 2. put the normal in i.normal
   // 3. put the texture coordinates in i.uv
   // and return true;
   //
    glm::dvec3 v = r.direction;
    glm::dvec3 o = r.position;

    glm::dvec3 planeNormal = glm::cross((a-c), (b-c));
    planeNormal = glm::normalize(planeNormal);

    // lec3 slide 31 (finding i)
    glm::dvec3 q = a;
    double t = glm::dot((q - o), planeNormal) / glm::dot(v, planeNormal);
    glm::dvec3 p = r.at(t);
    // lec3 slide 35/36 (checking in bounds)
    if 	(glm::dot(glm::cross((b - a), p - a), planeNormal) < 0)
        return false;
    else if (glm::dot(glm::cross((c - b), p - b), planeNormal) < 0)
        return false;
    else if (glm::dot(glm::cross((a - c), p - c), planeNormal) < 0)
        return false;
    else {
        i.t = t ;

        // barycentric coords
        double tri_area = glm::length(glm::cross((b - a), (c - a)));
        double u = glm::length(glm::cross((c - a), (p - a))) / tri_area;
        double v = glm::length(glm::cross((b - a), (p - a))) / tri_area;
        double w = 1.0 - u - v;

        i.uv = (glm::dvec2(u,v));

        if(use_per_vertex_normals) {
            glm::dvec3 phongNormal(0.0,0.0,0.0);
            phongNormal += glm::dvec3(w * a_n[0], w * a_n[1], w * a_n[2]);
            phongNormal += glm::dvec3(u * b_n[0], u * b_n[1], u * b_n[2]);
            phongNormal += glm::dvec3(v * c_n[0], v * c_n[1], v * c_n[2]);
            i.normal = glm::normalize(phongNormal);
        } else {
            i.normal = glm::normalize(planeNormal);
        }

        return true;
    }
}
