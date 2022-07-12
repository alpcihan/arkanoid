#include "graphics/opengl/OpenGLContext.h"

namespace gfx
{

OpenGLContext::OpenGLContext(GLFWwindow *window)
    : window(window){}

void OpenGLContext::init()
{
    glfwMakeContextCurrent(this->window);
    this->loadGlad();
}

void OpenGLContext::swapBuffers()
{
    glfwSwapBuffers(this->window);
}

void OpenGLContext::loadGlad()
{ 
    bool success = gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    if (!success)
        std::cout << "Could not initialize GLAD!" << std::endl;
}

}
