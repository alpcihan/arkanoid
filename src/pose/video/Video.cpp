#include "Video.h"

namespace pose
{
    Video::Video(int cameraIndex)
    {
        capture = std::make_unique<cv::VideoCapture>(cameraIndex);
        capture->open(cameraIndex);
    }

    Video::Video(const std::string &mediaDirectory, bool isLooped)
        : isLooped(isLooped)
    {
        capture = std::make_unique<cv::VideoCapture>(mediaDirectory);
        capture->open(mediaDirectory);
    }

    bool Video::getFrame(Image *frame) const
    {
        bool hasMoreFrames = capture->read(*frame);

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