#pragma once

#include "graphics/shared.h"
#include "graphics/shader/ShaderType.h"

namespace gfx
{
class Shader
{
public:
    Shader(const std::string &path);
    ~Shader();

    void bind() const;

    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setVec3(const std::string &name, glm::vec3 value) const;
    void setVec4(const std::string &name, glm::vec4 value) const;
    void setMat3(const std::string &name, glm::mat4 &value) const;
    void setMat4(const std::string &name, glm::mat4 &value) const;

private:
    void init(const std::string &path);
    void readFile(const std::string &path, std::string& vertSrc, std::string& fragSrc) const;
    unsigned int createShader(const ShaderType shaderType, const char *shaderSource) const;
    void link() const;

    void checkShaderCompileStatus(unsigned int shaderId, const ShaderType shaderType) const;
    void checkProgramLinkStatus() const;

private:
    unsigned int programId;
};
}