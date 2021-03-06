/****************************************************************************
 * Copyright ©2017 Brian Curless.  All rights reserved.  Permission is hereby
 * granted to students registered for University of Washington CSE 457 or CSE
 * 557 for use solely during Autumn Quarter 2017 for purposes of the course.
 * No other use, copying, distribution, or modification is permitted without
 * prior written consent. Copyrights for third-party components of this work
 * must be honored.  Instructors interested in reusing these course materials
 * should contact the author.
 ****************************************************************************/
#ifndef SPHERE_H
#define SPHERE_H

#include <properties.h>
#include <scene/components/geometry.h>

class Sphere : public Geometry {
public:
    ChoiceProperty Subdivisions;
    DoubleProperty Roundness;

    Sphere();

    virtual Mesh* GetRenderMesh() { return mesh_.get(); }

    // If false, it will just raytrace the render mesh
    virtual bool UseCustomTrace() {
        return true;//Roundness.Get() > 0.49;
    }

    // Override this to define the custom trace
    virtual bool IntersectLocal(const Ray &r, Intersection &i);

protected:
    void OnSubdivisionsSet(int) { RegenerateMesh(); }
    void OnRoundnessSet(double) { RegenerateMesh(); }
    void RegenerateMesh();

    std::unique_ptr<Mesh> CreateMesh(unsigned int subdivisions);
    std::unique_ptr<Mesh> CreateMesh_old(unsigned int subdivisions);
    std::unique_ptr<Mesh> mesh_;
};

#endif // SPHERE_H
