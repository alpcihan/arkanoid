#pragma once

#include <opencv2/opencv.hpp>
#include "pose/processor/Processor.h"
#include "ContourVecProcessorParams.h"
#include "pose/types.h"

namespace pose
{
    template <class Output>
    class ContourVecProcessor : public Processor<ContourVec, Output>
    {
    };

    class ApprxPolysContourVecProcessor : public ContourVecProcessor<ContourVec>
    {
    public:
        ApprxPolyContourVecProcessorParams params;

    public:
        void process(const ContourVec &input, ContourVec *output) override;
    };

    class FilterPolysContourVecProcessor : public ContourVecProcessor<ContourVec>
    {
    public:
        FilterPolysContourVecProcessorParams params;

    public:
        void process(const ContourVec &input, ContourVec *output) override;
    };

    class SubDivideContourVecProcessor : public ContourVecProcessor<ContourVec>
    {
    public:
        SubDivideContourVecProcessorParams params;

    public:
        void process(const ContourVec &input, ContourVec *output) override;
    };

    class PreciseCornersContourVecProcessor : public ContourVecProcessor<Group<cv::Point2f>>
    {
    public:
        PreciseCornersContourVecProcessorParams params;
        const Image *img = nullptr;

    public:
        void process(const ContourVec &input, Group<cv::Point2f> *output) override;

    private:
        StripeVec getStripes(const Contour& contour);
        Image getSubPixImage(const Stripe &stripe);
        Image getSobelImage(const Image &image);
        cv::Point2d getPrecisePoint(const Stripe& stripe, const Image& sobelImage);
        void fitLines(const Vec<Point2d>& points, float (&lineParams)[16]);
        Vec<cv::Point2f> getEdges(const float (&lines)[16]);
    };
}