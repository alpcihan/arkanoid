#pragma once

#include "graphics/shared.h"
#include "graphics/shader/ShaderType.h"

namespace gfx
{
class Shader
{
public:
    Shader(const std::string &vertexSrcFilePath, const std::string &fragmentSrcFilePath);
    ~Shader();

    void bind() const;

    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setMat3(const std::string &name, glm::mat4 &value) const;
    void setMat4(const std::string &name, glm::mat4 &value) const;

private:
    void init(const std::string &vertexSrcFilePath, const std::string &fragmentSrcFilePath);
    std::string readFile(const std::string &shaderSrcFilePath) const;
    unsigned int createShader(const ShaderType shaderType, const char *shaderSource) const;
    void link() const;

    void checkShaderCompileStatus(unsigned int shaderId, const ShaderType shaderType) const;
    void checkProgramLinkStatus() const;

private:
    unsigned int programId;
};
}