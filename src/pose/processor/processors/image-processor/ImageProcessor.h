#pragma once

#include <opencv2/opencv.hpp>
#include "pose/processor/Processor.h"
#include "ImageProcessorParams.h"

namespace pose
{
    template <class Output>
    class ImageProcessor : public Processor<Image, Output>
    {
    };

    class GrayScaleImageProcessor : public ImageProcessor<Image>
    {
    public:
        void process(const Image &input, Image *output) override;
    };

    class AdaptiveTHImageProcessor : public ImageProcessor<Image>
    {
    public:
        AdaptiveTHImageProcessorParams params;

    public:
        void process(const Image &input, Image *output) override;
    };

    class DetectContourVecImageProcessor : public ImageProcessor<ContourVec>
    {
    public:
        ContourVecImageProcessorParams params;

    public:
        void process(const Image &input, ContourVec *output) override;
    };

    class SobelImageProcessor : public ImageProcessor<Image>
    {
    public:
        SobelImageProcessorParams params;

    public:
        void process(const Image &input, Image *output) override;
    };
}