#include "graphics/shader/ShaderType.h"

namespace gfx
{

const std::unordered_map<ShaderDataType, unsigned int> shaderDataTypeByte({
    {ShaderDataType::Int, 4},
    {ShaderDataType::IVec2, 4 * 2},
    {ShaderDataType::IVec3, 4 * 3},
    {ShaderDataType::IVec4, 4 * 4},

    {ShaderDataType::Float, 4},
    {ShaderDataType::Vec2, 4 * 2},
    {ShaderDataType::Vec3, 4 * 3},
    {ShaderDataType::Vec4, 4 * 4},

    {ShaderDataType::Bool, 1},
});

const std::unordered_map<ShaderDataType, unsigned int> shaderDataTypeSize({
    {ShaderDataType::Int, 1},
    {ShaderDataType::IVec2, 2},
    {ShaderDataType::IVec3, 3},
    {ShaderDataType::IVec4, 4},

    {ShaderDataType::Float, 1},
    {ShaderDataType::Vec2, 2},
    {ShaderDataType::Vec3, 3},
    {ShaderDataType::Vec4, 4},

    {ShaderDataType::Bool, 1},
});

const std::unordered_map<ShaderDataType, GLenum> shaderDataTypeToGLShaderDataType({
    {ShaderDataType::Int, GL_INT},
    {ShaderDataType::IVec2, GL_INT},
    {ShaderDataType::IVec3, GL_INT},
    {ShaderDataType::IVec4, GL_INT},

    {ShaderDataType::Float, GL_FLOAT},
    {ShaderDataType::Vec2, GL_FLOAT},
    {ShaderDataType::Vec3, GL_FLOAT},
    {ShaderDataType::Vec4, GL_FLOAT},

    {ShaderDataType::Bool, GL_BOOL},
});

const std::unordered_map<ShaderType, GLenum> shaderTypeToGLShaderType({
    {ShaderType::Fragment, GL_FRAGMENT_SHADER},
    {ShaderType::Vertex, GL_VERTEX_SHADER},
    {ShaderType::Geometry, GL_GEOMETRY_SHADER},
});

}
