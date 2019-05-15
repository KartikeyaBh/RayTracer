/****************************************************************************
 * Copyright ©2017 Brian Curless.  All rights reserved.  Permission is hereby
 * granted to students registered for University of Washington CSE 457 or CSE
 * 557 for use solely during Autumn Quarter 2017 for purposes of the course.
 * No other use, copying, distribution, or modification is permitted without
 * prior written consent. Copyrights for third-party components of this work
 * must be honored.  Instructors interested in reusing these course materials
 * should contact the author.
 ****************************************************************************/
#include "particlesystem.h"
#include <scene/sceneobject.h>

REGISTER_COMPONENT(ParticleSystem, ParticleSystem)

ParticleSystem::ParticleSystem() :
    ParticleMaterial(AssetType::Material),
    Period(0.1f, 0.0f, 1.0f, 0.01f),
    Restitution(1.0f, 0.0f, 2.0f, 0.1f),
    num_particles_(0),
    particle_index_(0),
    simulating_(false)
{
    AddProperty("Material", &ParticleMaterial);
    AddProperty("Period (s)", &Period);
    AddProperty("Restitution", &Restitution);

    forces_.push_back(std::make_unique<ConstantForce>(glm::vec3(0.0f, -9.8f, 0.0f)));
}

void ParticleSystem::UpdateModelMatrix(glm::mat4 model_matrix) {
   model_matrix_ = model_matrix;
}

void ParticleSystem::EmitParticles() {
    if (!simulating_) return;

    glm::vec4 local_position(0, 0, 0, 1);
    glm::vec4 local_velocity(5, 5, 0, 0);

    // Calculate the world position
    glm::vec3 world_position = model_matrix_ * local_position;
    glm::vec3 world_velocity = model_matrix_ * local_velocity;

    // Emit a particle at world position
    std::unique_ptr<Particle> particle = std::make_unique<Particle>(0.1f, world_position, world_velocity);

//    p->color = Vec3d(1, 225 / 256.0, 53 / 256.0);
//    p->rotation[0] = rand() % 360;
//    p->rotation[1] = rand() % 360;
//    p->rotation[2] = rand() % 360;
//    double arc_angle = rand() % 60; // Max 90
//    double scale = 4;
//    double radius = sin(arc_angle / 180 * M_PI) * (scale / cos(arc_angle / 180 * M_PI));
//    if(radius == 0.0) radius = 0.1;
//    double circle_angle = rand() % 360;
//    double r_x = sin(circle_angle) * radius;
//    double r_y = 0;
//    double r_z = cos(circle_angle) * radius;
//    Vec3d vec = Vec3d(r_x, r_y, r_z);
//    // vec.normalize();
//    p->velocity = vec;

    if (num_particles_ < MAX_PARTICLES) {
        // Fill the vector
        particles_.push_back(std::move(particle));
        num_particles_ += 1;
    } else {
        // Replace the oldest particle
        particles_[particle_index_++] = std::move(particle);
        if (particle_index_ >= MAX_PARTICLES) particle_index_ = 0;
    }

    // Reset the time
    time_to_emit_ = Period.Get();
}

std::vector<Particle*> ParticleSystem::GetParticles() {
    std::vector<Particle*> particles;
    for (auto& particle : particles_) particles.push_back(particle.get());
    return particles;
}

void ParticleSystem::StartSimulation() {
    simulating_ = true;
    ResetSimulation();
}

void ParticleSystem::UpdateSimulation(float delta_t, const std::vector<std::pair<SceneObject*, glm::mat4>>& colliders) {
    if (!simulating_) return;

    // Emit Particles
    time_to_emit_ -= delta_t;
    if (time_to_emit_ <= 0.0) EmitParticles();

    // Compute Forces and Update Particles
    for (auto& particle : particles_) {
        // Clear Force
        glm::vec3 total_force(0.0f, 0.0f, 0.0f);
        // Calculate Force
        for (auto& force : forces_) {
            total_force += force->GetForce(*particle);
        }
        // Compute Derivative
        glm::vec3 pos_delta = particle->Velocity * delta_t;
        glm::vec3 vel_delta = total_force / particle->Mass * delta_t;
        // Update the particle
        particle->Position = particle->Position + pos_delta;
        particle->Velocity = particle->Velocity + vel_delta;
        static const double EPSILON = 0.00001;
        float particle_radius = 0.5f;
        // Check for collisions
        for (auto& kv : colliders) {
            SceneObject* collider_object = kv.first;
            glm::mat4 collider_model_matrix = kv.second;
            glm::vec3 collider_world_position = glm::vec3(collider_model_matrix * glm::vec4(0, 0, 0, 1));
            glm::vec3 X = particle->Position;
            glm::vec3 V = particle->Velocity;
            // Check for Sphere Collision
            if (SphereCollider* sphere_collider = collider_object->GetComponent<SphereCollider>()) {
                glm::vec3 C = collider_world_position;
                glm::vec3 X_sub_C = X - C;
                glm::vec3 N = glm::normalize(X_sub_C);
                bool collision_occurred = glm::length(X_sub_C) <= (sphere_collider->Radius.Get() + EPSILON + particle_radius); // Inexact Collision
                bool entering = glm::dot(N, V) < 0; // Make sure we're entering not exiting
                if (collision_occurred && entering) {
                    glm::vec3 V_N = glm::dot(N, V) * N;
                    glm::vec3 V_T = V - V_N;
                    glm::vec3 V_response = V_T - V_N * float(Restitution.Get());
                    particle->Velocity = V_response;
                }
            }
            // Check for Plane Collision
            else if (PlaneCollider* plane_collider = collider_object->GetComponent<PlaneCollider>()) {
                glm::vec3 P = collider_world_position;
                glm::vec3 N = glm::normalize(glm::vec3(collider_model_matrix * glm::vec4(0, 0, 1, 0))); // Plane is initially pointing down the Z-Axis
                glm::vec3 Up = glm::normalize(glm::vec3(collider_model_matrix * glm::vec4(0, 1, 0, 0))); // For Width
                glm::vec3 Right = glm::normalize(glm::vec3(collider_model_matrix * glm::vec4(1, 0, 0, 0))); // For Height
                glm::vec3 X_sub_P = X - P;
                float D = glm::dot(X_sub_P, N); // Distance from X to plane
                glm::vec3 Q = X - D * N; // Q is X moved to be on the plane
                glm::vec3 dist = Q - P; // Distance to center point along plane
                float height_component = std::abs(glm::dot(dist, Up));
                float width_component = std::abs(glm::dot(dist, Right));
                float half_height = float(plane_collider->Height.Get()) / 2.0f;
                float half_width = float(plane_collider->Width.Get()) / 2.0f;
                bool collision_occurred = D <= EPSILON + particle_radius && D >= -EPSILON;
                bool collision_within_bounds = height_component < half_height && width_component < half_width;
                bool entering = glm::dot(N, V) < 0; // Make sure we're entering not exiting
                if (collision_occurred && collision_within_bounds && entering) {
                    glm::vec3 V_N = glm::dot(N, V) * N;
                    glm::vec3 V_T = V - V_N;
                    glm::vec3 V_response = V_T - V_N * float(Restitution.Get());
                    particle->Velocity = V_response;
                }
            }
        }
    }
}

void ParticleSystem::StopSimulation() {
    simulating_ = false;
}

void ParticleSystem::ResetSimulation() {
    particles_.clear();
    num_particles_ = 0;
    particle_index_ = 0;
    time_to_emit_ = Period.Get();
}

bool ParticleSystem::IsSimulating() {
    return simulating_;
}
