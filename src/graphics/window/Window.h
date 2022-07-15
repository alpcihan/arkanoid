#pragma once

#include "graphics/shared.h"
#include "graphics/opengl/OpenGLContext.h"
#include "graphics/opengl/OpenGLAPI.h"

namespace gfx
{

    struct WindowProps
    {
        unsigned width;
        unsigned height;
        std::string title;

        WindowProps(unsigned width = 1920,
                    unsigned height = 1080,
                    const std::string &title = "My Project")
            : title(title), width(width), height(height) {}
    };

    class Window
    {
    public:
        Window(const WindowProps &windowProps = WindowProps());
        ~Window();

        void update() const;
        void clear() const;
        bool isClosed() const;
        GLFWwindow *getGLFWwindow() const;

    private:
        GLFWwindow *window;
        WindowProps windowProps;
        std::unique_ptr<OpenGLContext> openGLContext;

    private:
        void initialize();
        void initializeGLFW() const;
        void createWindow();

        void shutdown() const;
    };

}
