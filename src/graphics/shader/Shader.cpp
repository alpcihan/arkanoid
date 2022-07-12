#include "graphics/shader/Shader.h"

namespace gfx
{
Shader::Shader(const std::string &vertexSrcFilePath, const std::string &fragmentSrcFilePath)
{
    this->init(vertexSrcFilePath, fragmentSrcFilePath);
}

Shader::~Shader()
{
    glDeleteProgram(this->programId);
}

void Shader::bind() const
{
    glUseProgram(this->programId);
}

void Shader::setBool(const std::string &name, bool value) const
{
    const int uniformLocation = glGetUniformLocation(this->programId, name.c_str());
    glUniform1i(uniformLocation, (int)value);
}

void Shader::setInt(const std::string &name, int value) const
{
    const int uniformLocation = glGetUniformLocation(this->programId, name.c_str());
    glUniform1i(uniformLocation, value);
}

void Shader::setFloat(const std::string &name, float value) const
{
    const int uniformLocation = glGetUniformLocation(this->programId, name.c_str());
    glUniform1f(uniformLocation, value);
}

void Shader::setMat3(const std::string &name, glm::mat4 &matrix) const
{
    const int uniformLocation = glGetUniformLocation(this->programId, name.c_str());
    glUniformMatrix3fv(uniformLocation, 1, GL_FALSE, glm::value_ptr(matrix));
}

void Shader::setMat4(const std::string &name, glm::mat4 &matrix) const
{
    const int uniformLocation = glGetUniformLocation(this->programId, name.c_str());
    glUniformMatrix4fv(uniformLocation, 1, GL_FALSE, glm::value_ptr(matrix));
}

void Shader::init(const std::string &vertexSrcFilePath, const std::string &fragmentSrcFilePath)
{
    this->programId = glCreateProgram();

    const std::string vertexShaderSrc = readFile(vertexSrcFilePath);
    const std::string fragmentShaderSrc = readFile(fragmentSrcFilePath);

    const unsigned int vertexShaderId = this->createShader(ShaderType::Vertex, vertexShaderSrc.c_str());
    const unsigned int fragmentShaderId = this->createShader(ShaderType::Fragment, fragmentShaderSrc.c_str());

    glAttachShader(programId, vertexShaderId);
    glAttachShader(programId, fragmentShaderId);

    this->link();

    glDeleteShader(vertexShaderId);
    glDeleteShader(fragmentShaderId);
}

std::string Shader::readFile(const std::string &shaderSrcFilePath) const
{   
    std::ifstream shaderFile(shaderSrcFilePath);
    
    if (shaderFile.fail())
    {
        std::cout << "Failed to open shader file path: " <<  shaderSrcFilePath << std::endl;
        return "";
    }

    std::string shaderSourceCode = "";
    std::string line;
    while (getline(shaderFile, line))
    {
        shaderSourceCode += line + '\n';
    }

    return shaderSourceCode;
}

unsigned int Shader::createShader(const ShaderType shaderType, const char *shaderSource) const
{
    unsigned int shaderId = glCreateShader(shaderTypeToGLShaderType.at(shaderType));
    glShaderSource(shaderId, 1, &shaderSource, NULL);
    glCompileShader(shaderId);

    this->checkShaderCompileStatus(shaderId, shaderType);

    return shaderId;
}

void Shader::link() const
{
    glLinkProgram(this->programId);
    this->checkProgramLinkStatus();
}

void Shader::checkShaderCompileStatus(unsigned int shaderId, const ShaderType shaderType) const
{
    int isShaderCompiled;
    glGetShaderiv(shaderId, GL_COMPILE_STATUS, &isShaderCompiled);

    if (isShaderCompiled)
    {
        return;
    }

    char logMessage[1024];
    glGetShaderInfoLog(shaderId, 1024, NULL, logMessage);
    const std::string str = "Failed to compile shader: " + std::string(logMessage);

    std::cout << "Failed to compile " << (int)shaderType << " shader: " << std::string(logMessage) << std::endl;
}

void Shader::checkProgramLinkStatus() const
{
    int isProgramLinked;
    glGetProgramiv(this->programId, GL_LINK_STATUS, &isProgramLinked);

    if (isProgramLinked)
    {
        return;
    }

    char logMessage[1024];
    glGetShaderInfoLog(this->programId, 1024, NULL, logMessage);
    const std::string str = "Failed to compile shader: " + std::string(logMessage);

    std::cout << "Failed to link program " << this->programId << ": " <<  std::string(logMessage) << std::endl;;
}
}
