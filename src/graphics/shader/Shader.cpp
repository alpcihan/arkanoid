#include "graphics/shader/Shader.h"

namespace gfx
{
Shader::Shader(const std::string &path)
{
    this->init(path);
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

void Shader::init(const std::string &path)
{
    this->programId = glCreateProgram();

    std::string vertexShaderSrc; 
    std::string fragmentShaderSrc;
    readFile(path, vertexShaderSrc, fragmentShaderSrc);

    const unsigned int vertexShaderId = this->createShader(ShaderType::Vertex, vertexShaderSrc.c_str());
    const unsigned int fragmentShaderId = this->createShader(ShaderType::Fragment, fragmentShaderSrc.c_str());

    glAttachShader(programId, vertexShaderId);
    glAttachShader(programId, fragmentShaderId);

    this->link();

    glDeleteShader(vertexShaderId);
    glDeleteShader(fragmentShaderId);
}

void Shader::readFile(const std::string &path, std::string& vertSrc, std::string& fragSrc) const
{   
    std::ifstream shaderFile(path);
    
    if (shaderFile.fail())
    {
        std::cout << "Failed to open shader file path: " <<  path << std::endl;
    }

    vertSrc = "";
    fragSrc = "";
    bool isVert = false, isFrag = false;
    int vertCount = 0, fragCount = 0;
    std::string line;
    while (getline(shaderFile, line))
    {
        // check if vert or frag
        if(line.find("#VERT") != std::string::npos)
        {
            isVert = true;
            isFrag = false;
            vertCount++;
            continue;
        }
        else if(line.find("#FRAG") != std::string::npos)
        {
            isFrag = true;
            isVert = false;
            fragCount++;
            continue;
        }

        if(isVert)
            vertSrc += line + '\n';
        else if(isFrag)
            fragSrc += line + '\n';
    }

    if(vertCount != 1 || fragCount != 1 )
    {
        std::cout << "Invalid amount of shader at path: " << path << std::endl; 
    }
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
