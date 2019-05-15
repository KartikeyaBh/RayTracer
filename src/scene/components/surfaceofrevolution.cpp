/****************************************************************************
 * Copyright ©2017 Brian Curless.  All rights reserved.  Permission is hereby
 * granted to students registered for University of Washington CSE 457 or CSE
 * 557 for use solely during Autumn Quarter 2017 for purposes of the course.
 * No other use, copying, distribution, or modification is permitted without
 * prior written consent. Copyrights for third-party components of this work
 * must be honored.  Instructors interested in reusing these course materials
 * should contact the author.
 ****************************************************************************/
#include "surfaceofrevolution.h"
#include <fileio.h>

REGISTER_COMPONENT(SurfaceOfRevolution, Geometry)

SurfaceOfRevolution::SurfaceOfRevolution() :
    Geometry(),
    Curve(FileType::Curve),
    Quality({"High", "Medium", "Low", "Poor"})
{
    AddProperty("Curve", &Curve);
    AddProperty("Quality", &Quality);

    Curve.ValueSet.Connect(this, &SurfaceOfRevolution::OnCurveSet);
    Quality.ValueSet.Connect(this, &SurfaceOfRevolution::OnQualitySet);

    // Add some default points and generate a default mesh
    static const std::vector<glm::vec2> default_points = {
        glm::vec2(0.0f, 0.5f),
        glm::vec2(0.25f, 0.25f),
        glm::vec2(0.5f, 0.0f),
        glm::vec2(0.25f, -0.25f),
        glm::vec2(0.0f, -0.5f)
    };
    points_list_.push_back(default_points);
    OnQualitySet(Quality.Get());
}

void SurfaceOfRevolution::OnCurveSet(std::string curve_file) {
    // Get a list of points indexed by the quality
    points_list_ = FileIO::ReadCurveFile(curve_file);
    OnQualitySet(0);
}

void SurfaceOfRevolution::OnQualitySet(int quality) {
    if (points_list_.size() == 0) return;
    if ((unsigned int) Quality.Get() >= points_list_.size()) quality = 0;
    // Create a new surface of revolution mesh
    mesh_ = CreateMesh(points_list_[quality], points_list_[quality].size());
}

// Transfers ownership of a new Surface of Revolution Mesh to the caller
std::unique_ptr<Mesh> SurfaceOfRevolution::CreateMesh(const std::vector<glm::vec2>& curve_points, unsigned int subdivisions) {
    std::unique_ptr<Mesh> surface = std::make_unique<Mesh>("Surface of Revolution");

    // Compute the amount of data we will produce
    unsigned int num_bands = curve_points.size();
    unsigned int band_size = subdivisions + 1; // Duplicate the last vertex
    unsigned int num_verts = band_size * num_bands;
    unsigned int num_tris = (band_size - 1) * 2 * (num_bands - 1);

    // Allocate arrays to contain our data
    std::vector<float> vertices(3 * num_verts);
    std::vector<float> normals(3 * num_verts);
    std::vector<float> UVs(2 * num_verts);
    std::vector<unsigned int> triangles(3 * num_tris);


    //TODO: Implement surface of revolution


    surface->SetPositions(vertices);
    surface->SetNormals(normals);
    surface->SetUVs(UVs);
    surface->SetTriangles(triangles);

    return std::move(surface);
}