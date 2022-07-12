#pragma once

#include "graphics/shared.h"

namespace gfx
{

namespace OpenGLAPI
{  
    void init();
	void setViewport(uint32_t x, uint32_t y, uint32_t width, uint32_t height);
	void setClearColor(const glm::vec4& color);
	void clear();
}

}