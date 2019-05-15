#include "tracesceneobject.h"

TraceGeometry::TraceGeometry(Geometry* geometry_) :
    geometry(geometry_), identity_transform(true), transform(glm::mat4()), inverse_transform(glm::mat4()), normals_transform(glm::mat3())
{
    world_bbox = geometry->HasBoundingBox() ? geometry->GetLocalBoundingBox() : nullptr;
}

TraceGeometry::TraceGeometry(Geometry* geometry_, glm::mat4 transform_) :
    identity_transform(false), transform(transform_), inverse_transform(glm::inverse(transform_)), normals_transform(glm::transpose(glm::inverse(glm::mat3(transform_)))), geometry(geometry_)
{
    world_bbox = geometry->HasBoundingBox() ? geometry->GetWorldBoundingBox(transform_) : nullptr;
}

TraceGeometry::~TraceGeometry()
{
    if (world_bbox) {
        delete world_bbox;
    }
}

bool TraceGeometry::Intersect(const Ray &r, Intersection &i)
{
    //no transforms needed... nice
    if (identity_transform) {
        if (geometry->IntersectLocal(r, i)) {
            i.obj = this;
            return true;
        } else {
            return false;
        }
    }

    // Transform the ray into the object's local coordinate space
    glm::dvec3 pos = glm::dvec3(inverse_transform * glm::dvec4(r.position, 1));
    glm::dvec3 dir = glm::dvec3(inverse_transform * glm::dvec4((r.position + r.direction), 1)) - pos;
    double length = dir.length();
    dir /= length;

    Ray localRay( pos, dir );

    if (geometry->IntersectLocal(localRay, i)) {
        // Transform the intersection point & normal returned back into global space.
        i.normal = glm::normalize(normals_transform * i.normal);
        i.t /= length;
        i.obj = this;
        return true;
    } else {
        return false;
    }
}
