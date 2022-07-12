#include "Object3D.h"

namespace gfx
{
    Object3D::Object3D(
        const std::vector<glm::vec3>& positions,
        const std::vector<glm::vec3>& normals,
        const std::vector<unsigned int>& indices, 
        const std::vector<glm::vec2>& uvs,
        std::shared_ptr<Texture> texture,
        std::shared_ptr<Shader> shader
    ) : shader(shader), texture(texture)
    {   
    // create vao
    glGenVertexArrays(1, &this->vao);
    glBindVertexArray(this->vao);

    // create attrib (positions)
    glGenBuffers(1, &vboPos);
    glBindBuffer(GL_ARRAY_BUFFER, vboPos);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * positions.size(), positions.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, 0);

    // create attrib (normals)
    glGenBuffers(1, &vboNorm);
    glBindBuffer(GL_ARRAY_BUFFER, vboNorm);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * normals.size(), normals.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, 0);

    // create attrib (uvs)
    glGenBuffers(2, &vboUV);
    glBindBuffer(GL_ARRAY_BUFFER, vboUV);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * uvs.size(), uvs.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    // create indices
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), indices.data(), GL_STATIC_DRAW);

    indexCount = indices.size();
    }

    void Object3D::draw()
    {
        glBindVertexArray(vao);
        shader->bind();

        texture->bind(0);
        shader->setInt("u_texture0", 0);

        glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
    }

    glm::mat4 Object3D::getTransform()
    {
         return glm::translate(glm::mat4(1.0f), translation) *
                glm::toMat4(glm::quat(rotation)) *
                glm::scale(glm::mat4(1.0f), scale);
    }
};
