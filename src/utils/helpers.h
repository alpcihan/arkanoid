#pragma once

#include "graphics/gfx.h"
#include "pose/pose.h"

namespace help
{
    // convert cv::Mat to OpenGL texture
    // flip the cv::Mat and make it RGB 
    std::shared_ptr<gfx::Texture> getBGTexture(const pose::Image &frame);

    void printMat4(const glm::mat4& mtrx);
}