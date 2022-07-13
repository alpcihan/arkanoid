#pragma once

#include "pose/types.h"

namespace pose
{
    class Video
    {
    public:
        Video() = default;
        Video(int cameraIndex);
        Video(const std::string &mediaDirectory, bool isLooped = true);

        bool getFrame(Image* frame);
        
    private:
        std::unique_ptr<cv::VideoCapture> capture;
        bool isLooped;
        bool isWebcam = false;
    };
};