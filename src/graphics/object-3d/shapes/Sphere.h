#pragma once

#include "../Object3D.h"
#include "mesh-generation.h"

namespace gfx
{
    class Sphere : public Object3D
    {
    public:
        Sphere(
            std::shared_ptr<Texture> texture,
            std::shared_ptr<Shader> shader)
        {
            std::vector<glm::vec3> positions;
            std::vector<glm::vec3> normals;
            std::vector<glm::vec2> uvs;
            std::vector<unsigned int> indices;

            GenerateParametricShapeFrom2D(
                positions,
                normals,
                uvs,
                indices,
                ParametricHalfCircle,
                30,
                30);
            init(positions, normals, indices, uvs, texture, shader);
        }

    private:
        Sphere(
            const std::vector<glm::vec3> &positions,
            const std::vector<glm::vec3> &normals,
            const std::vector<unsigned int> &indices,
            const std::vector<glm::vec2> &uvs,
            std::shared_ptr<Texture> texture,
            std::shared_ptr<Shader> shader){};
    };

}