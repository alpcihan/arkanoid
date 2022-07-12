#pragma once

#include "../Object3D.h"

namespace gfx
{
    class Cube : public Object3D
    {
        public:
        Cube(
            std::shared_ptr<Texture> texture,
            std::shared_ptr<Shader> shader 
        )
        {
            std::vector<glm::vec3> positions = {
                // front
                glm::vec3(0.5, 0.5, 0.5),
                glm::vec3(0.5, -0.5, 0.5),
                glm::vec3(-0.5, -0.5, 0.5),
                glm::vec3(-0.5, 0.5, 0.5),
                // back
                glm::vec3(0.5, 0.5, -0.5),
                glm::vec3(0.5, -0.5, -0.5),
                glm::vec3(-0.5, -0.5, -0.5),
                glm::vec3(-0.5, 0.5, -0.5),
                // right
                glm::vec3(0.5, 0.5, -0.5),
                glm::vec3(0.5, -0.5, -0.5),
                glm::vec3(0.5, -0.5, 0.5),
                glm::vec3(0.5, 0.5, 0.5),
                // left
                glm::vec3(-0.5, 0.5, 0.5),
                glm::vec3(-0.5, -0.5, 0.5),
                glm::vec3(-0.5, -0.5, -0.5),
                glm::vec3(-0.5, 0.5, -0.5),
                // top
                glm::vec3(0.5, 0.5, -0.5),
                glm::vec3(0.5, 0.5, 0.5),
                glm::vec3(-0.5, 0.5, 0.5),
                glm::vec3(-0.5, 0.5, -0.5),
                // bottom
                glm::vec3(0.5, -0.5, -0.5),
                glm::vec3(0.5, -0.5, 0.5),
                glm::vec3(-0.5, -0.5, 0.5),
                glm::vec3(-0.5, -0.5, -0.5)};

            std::vector<glm::vec3> normals = {
                // front
                glm::vec3(0.0, 0.0, 1.0),
                glm::vec3(0.0, 0.0, 1.0),
                glm::vec3(0.0, 0.0, 1.0),
                glm::vec3(0.0, 0.0, 1.0),
                // back
                glm::vec3(0.0, 0.0, -1.0),
                glm::vec3(0.0, 0.0, -1.0),
                glm::vec3(0.0, 0.0, -1.0),
                glm::vec3(0.0, 0.0, -1.0),
                // right
                glm::vec3(1.0, 0.0, 0.0),
                glm::vec3(1.0, 0.0, 0.0),
                glm::vec3(1.0, 0.0, 0.0),
                glm::vec3(1.0, 0.0, 0.0),
                // left
                glm::vec3(-1.0, 0.0, 0.0),
                glm::vec3(-1.0, 0.0, 0.0),
                glm::vec3(-1.0, 0.0, 0.0),
                glm::vec3(-1.0, 0.0, 0.0),
                // top
                glm::vec3(0.0, 1.0, 0.0),
                glm::vec3(0.0, 1.0, 0.0),
                glm::vec3(0.0, 1.0, 0.0),
                glm::vec3(0.0, 1.0, 0.0),
                // bottom
                glm::vec3(0.0, -1.0, 0.0),
                glm::vec3(0.0, -1.0, 0.0),
                glm::vec3(0.0, -1.0, 0.0),
                glm::vec3(0.0, -1.0, 0.0)
            };

            std::vector<glm::vec2> uvs = {
                // front
                glm::vec2(1.0, 1.0),
                glm::vec2(1.0, 0.0),
                glm::vec2(0.0, 0.0),
                glm::vec2(0.0, 1.0),
                // back
                glm::vec2(1.0, 1.0),
                glm::vec2(1.0, 0.0),
                glm::vec2(0.0, 0.0),
                glm::vec2(0.0, 1.0),
                // right
                glm::vec2(1.0, 1.0),
                glm::vec2(1.0, 0.0),
                glm::vec2(0.0, 0.0),
                glm::vec2(0.0, 1.0),
                // left
                glm::vec2(1.0, 1.0),
                glm::vec2(1.0, 0.0),
                glm::vec2(0.0, 0.0),
                glm::vec2(0.0, 1.0),
                // top
                glm::vec2(1.0, 1.0),
                glm::vec2(1.0, 0.0),
                glm::vec2(0.0, 0.0),
                glm::vec2(0.0, 1.0),
                // bottom
                glm::vec2(1.0, 1.0),
                glm::vec2(1.0, 0.0),
                glm::vec2(0.0, 0.0),
                glm::vec2(0.0, 1.0)
            };

            std::vector<unsigned int> indices = {
                // front
                0, 1, 2,
                2, 3, 0,
                // back
                4, 5, 6,
                6, 7, 4,
                // right
                8, 9, 10,
                10, 11, 8,
                // left
                12, 13, 14,
                14, 15, 12,
                // top
                16, 17, 18,
                18, 19, 16,
                // bottom
                20, 21, 22,
                22, 23, 20,
            };

            init(positions, normals, indices, uvs, texture, shader);
        }

        private:
        Cube(
            const std::vector<glm::vec3>& positions, 
            const std::vector<glm::vec3>& normals, 
            const std::vector<unsigned int>& indices, 
            const std::vector<glm::vec2>& uvs,
            std::shared_ptr<Texture> texture,
            std::shared_ptr<Shader> shader
        ){};
    };

}