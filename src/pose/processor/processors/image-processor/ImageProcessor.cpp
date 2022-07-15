#include <opencv2/opencv.hpp>
#include "pose/types.h"
#include "ImageProcessor.h"
#include "config/config.h"

namespace pose
{
    void GrayScaleImageProcessor::process(const Image &input, Image *output)
    {
        cv::cvtColor(input, result, CV_BGR2GRAY);
    }

    void AdaptiveTHImageProcessor::process(const Image &input, Image *output)
    {   
        cv::adaptiveThreshold(
            input,
            result,
            config::maxIntensity,
            ATH_TYPE,
            cv::THRESH_BINARY,
            config::blockSize,
            config::athConst);
    }

    void DetectContourVecImageProcessor::process(const Image &input, ContourVec *output)
    {
        cv::findContours(
            input,
            result,
            params.retrievalMode,
            params.apprxMode);
    }

    void SobelImageProcessor::process(const Image &input, Image *output)
    {
        unsigned dir = params.isHorizontal ? 1 : 0;

        cv::Sobel(input, result, CV_8UC1, dir, dir == 1 ? 0 : 1);
    }
}