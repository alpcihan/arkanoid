#include "Video.h"
#include <iostream>

namespace pose
{
    Video::Video(int cameraIndex)
    {
        capture = std::make_unique<cv::VideoCapture>(cameraIndex);
        capture->open(cameraIndex);
        isWebcam = true;
    }

    Video::Video(const std::string &mediaDirectory, bool isLooped)
        : isLooped(isLooped)
    {
        capture = std::make_unique<cv::VideoCapture>(mediaDirectory);
        capture->open(mediaDirectory);
        isWebcam = false;
    }

    bool Video::getFrame(Image *frame)
    {
        bool hasMoreFrames = capture->read(*frame);

        static bool isFirstFrame = true;
        if(isFirstFrame)
        {
            std::cout << "Video captured, width: " << frame->cols << " height: " << frame->rows << std::endl;
            isFirstFrame = false;
        }

        if(isWebcam)
        {
            cv::resize(*frame, *frame, cv::Size(640,360));
        }

        if (hasMoreFrames)
            return true;

        if (isLooped && !hasMoreFrames)
        {
            capture->set(cv::CAP_PROP_POS_FRAMES, 0);
            capture->read(*frame);
            return true;
        }

        return false;
    }
};