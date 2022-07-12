#pragma once

#include "graphics/shared.h"

namespace gfx
{

enum class ShaderType
{
    Fragment, Vertex, Geometry
};

enum class ShaderDataType
{
    Int, IVec2, IVec3, IVec4,
    Float, Vec2, Vec3, Vec4,
    Bool
};

extern const std::unordered_map<ShaderDataType, unsigned int> shaderDataTypeByte;
extern const std::unordered_map<ShaderDataType, unsigned int> shaderDataTypeSize;
extern const std::unordered_map<ShaderDataType, GLenum> shaderDataTypeToGLShaderDataType;
extern const std::unordered_map<ShaderType, GLenum> shaderTypeToGLShaderType;

}