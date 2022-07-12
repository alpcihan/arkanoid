#pragma once

#include "../Object3D.h"

namespace gfx
{

    class Plane : public Object3D
    {
        public:
        Plane(
            std::shared_ptr<Texture> texture,
            std::shared_ptr<Shader> shader 
        )
        {
            std::vector<glm::vec3> positions = {
                glm::vec3(0.5, 0.5, 0.5),
                glm::vec3(0.5, -0.5, 0.5),
                glm::vec3(-0.5, -0.5, 0.5),
                glm::vec3(-0.5, 0.5, 0.5)
            };
            std::vector<glm::vec3> normals = {
                glm::vec3(0.0, 0.0, 1.0),
                glm::vec3(0.0, 0.0, 1.0),
                glm::vec3(0.0, 0.0, 1.0),
                glm::vec3(0.0, 0.0, 1.0)
            };
            std::vector<glm::vec2> uvs = {
                glm::vec2(1.0, 1.0),
                glm::vec2(1.0, 0.0),
                glm::vec2(0.0, 0.0),
                glm::vec2(0.0, 1.0),
            };
            std::vector<unsigned int> indices = {
                0, 1, 2,
                2, 3, 0,
            };

            init(positions, normals, indices, uvs, texture, shader);
        }

        private:
        Plane(
            const std::vector<glm::vec3>& positions, 
            const std::vector<glm::vec3>& normals, 
            const std::vector<unsigned int>& indices, 
            const std::vector<glm::vec2>& uvs,
            std::shared_ptr<Texture> texture,
            std::shared_ptr<Shader> shader
        ){};
    };

}