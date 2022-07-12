#include <opencv2/opencv.hpp>
#include "pose/types.h"
#include "ImageProcessor.h"

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
            params.maxIntensity,
            params.type,
            cv::THRESH_BINARY,
            params.blockSize,
            params.constant);
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