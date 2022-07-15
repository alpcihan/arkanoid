#pragma once

#include "graphics/gfx.h"
#include "pose/pose.h"
#include <string>
#include "config/directories.h"

namespace help
{
    // convert cv::Mat to OpenGL texture
    // flip the cv::Mat and make it RGB 
    std::shared_ptr<gfx::Texture> getBGTexture(const pose::Image &frame);

    // print mat4
    void printMat4(const glm::mat4& mtrx);

    // get the path relative to the resources file
    std::string toResourcePath(std::string str);

    float rand();
}