#include "graphics/shared.h"
#include "graphics/shader/Shader.h"
#include "graphics/texture/Texture.h"

namespace gfx
{

class Object3D
{
    public:
        std::shared_ptr<Shader> shader;
        std::shared_ptr<Texture> texture;
        
        glm::vec3 translation = glm::vec3(0,0,0);
        glm::vec3 rotation = glm::vec3(0,0,0);
        glm::vec3 scale = glm::vec3(1,1,1);

    public:
        Object3D(
            const std::vector<glm::vec3>& positions, 
            const std::vector<glm::vec3>& normals, 
            const std::vector<unsigned int>& indices, 
            const std::vector<glm::vec2>& uvs,
            std::shared_ptr<Texture> texture,
            std::shared_ptr<Shader> shader
        );

        void draw();
        glm::mat4 getTransform();

    private:
        unsigned int indexCount;
        unsigned int vao;
        unsigned int vboNorm, vboPos, vboUV, ebo;
};

}