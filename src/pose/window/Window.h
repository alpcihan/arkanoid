#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include "pose/types.h"

namespace pose
{
    class Window
    {
    public:
        Window(std::string title = std::string(1,c++));
        ~Window();
        
        void display(const Image& frame) const;
        
    private:
        std::string title;
        bool isVisible = true;

        static char c; 
    };
}