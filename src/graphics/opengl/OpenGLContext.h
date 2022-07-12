#pragma once

#include "graphics/shared.h"

namespace gfx
{

class OpenGLContext
{
public:
    OpenGLContext(GLFWwindow* window);
    
    void init();
    void swapBuffers();

private:
    GLFWwindow* window;

private:
    void loadGlad();
};

}