#pragma once

#include "graphics/shared.h"

namespace gfx
{

class Texture
{
public:
    Texture(const std::string &path);
    Texture(unsigned char *textureData, unsigned int width, unsigned int height, unsigned int channels);
    ~Texture();

    void bind(uint32_t slot = 0);
    void unbind() const;

private:
    unsigned int id;

private:
    void init(unsigned char *textureData, unsigned int width, unsigned int height, unsigned int channels, std::string path = "");
};

}