#include "Window.h"

namespace pose
{
    Window::Window(std::string title) : title(title)
    {
        std::cout << title << std::endl;
        cv::namedWindow(title);
    }

    Window::~Window()
    {
        cv::destroyWindow(title);
    }
        
    void Window::display(const Image& frame) const
    {
        cv::imshow(title, frame);
    }
     
    char Window::c = (char)0;
}