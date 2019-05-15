/****************************************************************************
 * Copyright ©2017 Brian Curless.  All rights reserved.  Permission is hereby
 * granted to students registered for University of Washington CSE 457 or CSE
 * 557 for use solely during Autumn Quarter 2017 for purposes of the course.
 * No other use, copying, distribution, or modification is permitted without
 * prior written consent. Copyrights for third-party components of this work
 * must be honored.  Instructors interested in reusing these course materials
 * should contact the author.
 ****************************************************************************/
#include "meshprocessing.h"
#include <algorithm>

void MeshProcessing::ComputeNormals(Mesh& mesh) {
    const std::vector<float>& input_positions = mesh.GetPositions();
    const std::vector<unsigned int>& input_faces = mesh.GetTriangles();

    unsigned int num_verts = input_positions.size() / 3;

    // To facilitate vertex-normal computation, compute surface normals
    // and keep a running sum of the surface normal on each vertex
    std::vector<glm::vec3> surface_normals(num_verts);
    for (unsigned int i = 0; i < input_faces.size(); i += 3) {
        unsigned int A = input_faces[i];
        unsigned int B = input_faces[i+1];
        unsigned int C = input_faces[i+2];
        // Compute surface normal
        unsigned int A_index = A * 3;
        unsigned int B_index = B * 3;
        unsigned int C_index = C * 3;
        glm::vec3 A_pos = glm::vec3(input_positions[A_index], input_positions[A_index+1], input_positions[A_index+2]);
        glm::vec3 B_pos = glm::vec3(input_positions[B_index], input_positions[B_index+1], input_positions[B_index+2]);
        glm::vec3 C_pos = glm::vec3(input_positions[C_index], input_positions[C_index+1], input_positions[C_index+2]);
        glm::vec3 AB = B_pos - A_pos;
        glm::vec3 AC = C_pos - A_pos;
        glm::vec3 surface_normal = glm::cross(AB, AC);
        // Keep a running sum of the surface normal on each vertex
        surface_normals[A] += surface_normal;
        surface_normals[B] += surface_normal;
        surface_normals[C] += surface_normal;
    }

    // Compute Vertex Normals
    std::vector<float> output_normals(num_verts * 3);
    for (unsigned int vert = 0; vert < num_verts; vert++) {
        unsigned int vert_index = vert * 3;
        // Vertex Normal is the average of all the neighboring surface normals
        // It's also the same as if we just normalized the sum of them.
        glm::vec3 new_normal = glm::normalize(surface_normals[vert]);
        output_normals[vert_index] = new_normal.x;
        output_normals[vert_index+1] = new_normal.y;
        output_normals[vert_index+2] = new_normal.z;
    }

    mesh.SetNormals(output_normals);
}

void MeshProcessing::FilterMesh(const Mesh& input_mesh, Mesh& filtered_mesh, double a) {
    const std::vector<float>& input_positions = input_mesh.GetPositions();
    const std::vector<float>& input_UVs = input_mesh.GetUVs();
    const std::vector<unsigned int>& input_faces = input_mesh.GetTriangles();

    // Take a weighted sum of the vertex and its neighbors to produce a new mesh
    // with the same connectivity as the original mesh, but with updated vertex positions.
    // Vertices are neighbors if they share an edge. Filter weights will be 1 for the vertex,
    // and a / N for each neighboring vertex. The weights will then be normalized.
    // "a" controls smoothing or sharpening, and N is the number of neighboring vertices.

    // Calculate neighbor information from the triangles array
    unsigned int num_verts = input_positions.size() / 3;
    std::vector<std::set<unsigned int>> neighbors(num_verts);
    for (unsigned int i = 0; i < input_faces.size(); i += 3) {
        // In triples assign neighbors
        unsigned int A = input_faces[i];
        unsigned int B = input_faces[i+1];
        unsigned int C = input_faces[i+2];
        // Add neighbors of A
        neighbors[A].insert(B);
        neighbors[A].insert(C);
        // Add neighbors of B
        neighbors[B].insert(A);
        neighbors[B].insert(C);
        // Add neighbors of C
        neighbors[C].insert(A);
        neighbors[C].insert(B);
    }

    // Filter each vertex position
    std::vector<float> output_positions(num_verts * 3);
    for (unsigned int vert = 0; vert < num_verts; vert++) {
        unsigned int vert_index = vert * 3;
        unsigned int num_neighbors = neighbors[vert].size();
        float weight = a / num_neighbors;
        float norm_factor = 1.f / (a + 1.f);
        float norm_weight = weight * norm_factor;
        glm::vec3 new_pos = glm::vec3(input_positions[vert_index], input_positions[vert_index+1], input_positions[vert_index+2]) * norm_factor;
        for (auto& neighbor : neighbors[vert]) {
            unsigned int neighbor_index = neighbor * 3;
            glm::vec3 neighbor_pos = glm::vec3(input_positions[neighbor_index], input_positions[neighbor_index+1], input_positions[neighbor_index+2]);
            new_pos += neighbor_pos * norm_weight;
        }
        output_positions[vert_index] = new_pos.x;
        output_positions[vert_index+1] = new_pos.y;
        output_positions[vert_index+2] = new_pos.z;
    }

    filtered_mesh.SetPositions(output_positions);
    filtered_mesh.SetUVs(input_UVs);
    filtered_mesh.SetTriangles(input_faces);

    ComputeNormals(filtered_mesh);
}

void MeshProcessing::SubdivideMesh(const Mesh& input_mesh, Mesh& filtered_mesh, bool limit) {
    const std::vector<float>& input_positions = input_mesh.GetPositions();
    const std::vector<float>& input_UVs = input_mesh.GetUVs();
    const std::vector<unsigned int>& input_faces = input_mesh.GetTriangles();

    bool did_error=false;

    bool has_UVs = input_UVs.size() > 0;

    // STEP 1: Split

    // Calculate neighbor information from the triangles array
    unsigned int num_verts = input_positions.size() / 3;

    unsigned int new_num_verts = num_verts;

    std::map<mesh_edge, unsigned int> odd_verticies;

    //Assign a new vertex index to each edge
    for (unsigned int i = 0; i < input_faces.size(); i += 3) {
        // In triples assign neighbors
        unsigned int A = input_faces[i];
        unsigned int B = input_faces[i+1];
        unsigned int C = input_faces[i+2];

        mesh_edge AB = make_edge(A, B);
        mesh_edge BC = make_edge(B, C);
        mesh_edge AC = make_edge(A, C);

        for (mesh_edge edge : {AB, BC, AC}) {
            if(odd_verticies.find(edge) == odd_verticies.end()) {
                odd_verticies[edge] = new_num_verts++;
            }
        }
    }

    std::vector<float> subdivided_positions(new_num_verts * 3);
    std::vector<float> output_UVs(has_UVs ? new_num_verts * 2 : 0);
    std::vector<unsigned int> output_faces(input_faces.size() * 4);
    unsigned int face_creation_index = 0;

    std::copy(std::begin(input_positions), std::end(input_positions), std::begin(subdivided_positions));
    std::copy(std::begin(input_UVs), std::end(input_UVs), std::begin(output_UVs));

    for (unsigned int i = 0; i < input_faces.size(); i += 3) {
        // In triples assign neighbors
        unsigned int A = input_faces[i];
        unsigned int B = input_faces[i+1];
        unsigned int C = input_faces[i+2];

        mesh_edge AB = make_edge(A, B);
        mesh_edge BC = make_edge(B, C);
        mesh_edge AC = make_edge(A, C);

        for (mesh_edge edge : {AB, BC, AC}) {
            unsigned int i = odd_verticies.at(edge);

            subdivided_positions[i*3]=0.5*(input_positions[edge.first*3] + input_positions[edge.second*3]);
            subdivided_positions[i*3+1]=0.5*(input_positions[edge.first*3+1] + input_positions[edge.second*3+1]);
            subdivided_positions[i*3+2]=0.5*(input_positions[edge.first*3+2] + input_positions[edge.second*3+2]);

            if (has_UVs) {
                output_UVs[i*2]=0.5*(input_UVs[edge.first*2] + input_UVs[edge.second*2]);
                output_UVs[i*2+1]=0.5*(input_UVs[edge.first*2+1] + input_UVs[edge.second*2+1]);
            }
        }

        output_faces[face_creation_index++] = A;
        output_faces[face_creation_index++] = odd_verticies.at(AB);
        output_faces[face_creation_index++] = odd_verticies.at(AC);

        output_faces[face_creation_index++] = B;
        output_faces[face_creation_index++] = odd_verticies.at(BC);
        output_faces[face_creation_index++] = odd_verticies.at(AB);

        output_faces[face_creation_index++] = C;
        output_faces[face_creation_index++] = odd_verticies.at(AC);
        output_faces[face_creation_index++] = odd_verticies.at(BC);

        output_faces[face_creation_index++] = odd_verticies.at(AB);
        output_faces[face_creation_index++] = odd_verticies.at(BC);
        output_faces[face_creation_index++] = odd_verticies.at(AC);
    }

    // STEP 2: Average
    std::vector<float> output_positions(new_num_verts * 3);

    // Calculate neighbor information from the triangles array
    std::vector<std::set<unsigned int>> neighbors(new_num_verts);
    std::vector<std::set<std::pair<unsigned int, unsigned int>>> child_neighbors(new_num_verts);
    for (unsigned int i = 0; i < output_faces.size(); i += 3) {
        // In triples assign neighbors
        unsigned int A = output_faces[i];
        unsigned int B = output_faces[i+1];
        unsigned int C = output_faces[i+2];
        // Add neighbors of A
        neighbors[A].insert(B);
        neighbors[A].insert(C);
        child_neighbors[A].emplace(B, C);
        // Add neighbors of B
        neighbors[B].insert(A);
        neighbors[B].insert(C);
        child_neighbors[B].emplace(C, A);
        // Add neighbors of C
        neighbors[C].insert(A);
        neighbors[C].insert(B);
        child_neighbors[C].emplace(A, B);
    }

    for (unsigned int i = 0; i < subdivided_positions.size()/3; i++) {
        int valence = neighbors[i].size();

        if (valence==0) {
            if (!did_error) {
                did_error = true;
                std::cout << "warning, orphaned vertex!";
            }
            continue;
        }

        float n = valence;

        float beta = 1.25 - (pow(3.0 + (2.0 * cos(6.283185/n)), 2.0)/32.0);
        float alpha = (n*(1.0-beta))/beta;

        float masksum = alpha;
        for (int c=0; c<3; c++) {
            output_positions[(i*3)+c] = alpha*subdivided_positions[(i*3)+c];
        }

        for (auto nid : neighbors[i]) {
            masksum += 1.0;
            for (int c=0; c<3; c++) {
                output_positions[(i*3)+c] += subdivided_positions[(nid*3)+c];
            }
        }

        for (int c=0; c<3; c++) {
            output_positions[(i*3)+c] /= masksum;
        }

    }

    // STEP 3: Evaluate
    if (limit) {
        std::vector<float> eval_positions(new_num_verts * 3);

        for (unsigned int i = 0; i < subdivided_positions.size()/3; i++) {
            int valence = neighbors[i].size();

            if (valence==0) {
                if (!did_error) {
                    did_error = true;
                    std::cout << "warning, orphaned vertex!";
                }
                continue;
            }

            float n = valence;

            float beta = 1.25 - (pow(3.0 + (2.0 * cos(6.283185/n)), 2.0)/32.0);
            float epsilon = (n*3.0)/beta;

            float masksum = epsilon;
            for (int c=0; c<3; c++) {
                eval_positions[(i*3)+c] = epsilon*output_positions[(i*3)+c];
            }

            for (auto nid : neighbors[i]) {
                masksum += 1.0;
                for (int c=0; c<3; c++) {
                    eval_positions[(i*3)+c] += output_positions[(nid*3)+c];
                }
            }

            for (int c=0; c<3; c++) {
                eval_positions[(i*3)+c] /= masksum;
            }
        }

        filtered_mesh.SetPositions(eval_positions);
        filtered_mesh.SetTriangles(output_faces);

        std::vector<float> eval_normals(new_num_verts * 3);

        //Compute normals using tangent mask
        for (unsigned int i = 0; i < subdivided_positions.size()/3; i++) {
            int valence = neighbors[i].size();

            if (valence==0) {
                if (!did_error) {
                    did_error = true;
                    std::cout << "warning, orphaned vertex!";
                }
                continue;
            }

            float num = valence;

            int first_vertex = *(neighbors[i].begin());

            std::vector<float> T1(3);
            std::vector<float> T2(3);

            for (int c=0; c<3; c++) {
                T1[c] = 0;
                T2[c] = 0;
            }

            std::list<int> neighbor_loop;

            int v = first_vertex;
            do {
                neighbor_loop.push_back(v);
                int next=-1;
                for (auto n : child_neighbors[v]) {
                    if (n.second==i) {
                        if (neighbors[i].find(n.first) != neighbors[i].end()) {
                            if (next!=-1) {
                                if (!did_error) {
                                    did_error = true;
                                    std::cout << "warning, multiple ccw children!";
                                }
                            }
                            next = n.first;
                            //break;
                        }
                    }
                }
                if (next==-1) {
                    if (!did_error) {
                        did_error = true;
                        std::cout << "warning, unclosed mesh!";
                    }
                    break;
                }
                v = next;
            } while (v != first_vertex);

            float i1 = 0.0;
            for (auto n : neighbor_loop) {
                glm::vec3 Q(output_positions[(n*3)], output_positions[(n*3)+1], output_positions[(n*3)+2]);
                float i2 = i1==0.0? num : i1;
                i1 += 1.0;
                float t1 = cos(i1*6.283185/num);
                float t2 = cos(i2*6.283185/num);

                T1[0] += t1*Q.x;
                T1[1] += t1*Q.y;
                T1[2] += t1*Q.z;
                T2[0] += t2*Q.x;
                T2[1] += t2*Q.y;
                T2[2] += t2*Q.z;
            }

            glm::vec3 newnorm = glm::normalize(glm::cross(glm::vec3(T1[0],T1[1],T1[2]),glm::vec3(T2[0],T2[1],T2[2])));
            eval_normals[(i*3)] = newnorm.x;
            eval_normals[(i*3)+1] = newnorm.y;
            eval_normals[(i*3)+2] = newnorm.z;
        }

        filtered_mesh.SetNormals(eval_normals);
    } else {
        filtered_mesh.SetPositions(output_positions);
        filtered_mesh.SetTriangles(output_faces);
    }

    filtered_mesh.SetUVs(output_UVs);
}

void MeshProcessing::FlipNormals(const Mesh& input_mesh, Mesh& filtered_mesh) {
    const std::vector<float>& input_normals = input_mesh.GetNormals();
    const std::vector<unsigned int>& input_faces = input_mesh.GetTriangles();
    std::vector<float> output_normals;
    std::vector<unsigned int> output_faces;

    for (unsigned int i = 0; i < input_normals.size(); i++) {
        output_normals.push_back(-input_normals[i]);
    }

    // Also need to change face winding order so that backfaces are now frontfaces
    for (unsigned int i = 0; i < input_faces.size(); i += 3) {
        output_faces.push_back(input_faces[i]);
        output_faces.push_back(input_faces[i+2]);
        output_faces.push_back(input_faces[i+1]);
    }

    filtered_mesh.SetPositions(input_mesh.GetPositions());
    filtered_mesh.SetNormals(output_normals);
    filtered_mesh.SetUVs(input_mesh.GetUVs());
    filtered_mesh.SetTriangles(output_faces);
}
