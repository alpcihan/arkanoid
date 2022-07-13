#pragma once

#include "pose/types.h"

#define DEFAULT_CONTOURS_COLOR cv::Scalar(0, 0, 255)

#define DEFAULT_POINT_COLOR  cv::Scalar(200, 255, 200)
#define DEFAULT_POINT_COLOR1 cv::Scalar(0, 255, 0)
#define DEFAULT_POINT_COLOR2 cv::Scalar(255, 0, 0)

namespace pose
{
    void drawContourVec(const ContourVec &contourVec, Image *frame, const cv::Scalar &color = DEFAULT_CONTOURS_COLOR);

    void drawPoints(
        const ContourVec &contourVec,
        Image *frame,
        unsigned int repeatsEvery = 1,
        const cv::Scalar &color1 = DEFAULT_POINT_COLOR1,
        const cv::Scalar &color2 = DEFAULT_POINT_COLOR2);

    void drawPoints(
        const Group<cv::Point2f> &pointGroup,
        Image *frame,
        const cv::Scalar &color = DEFAULT_POINT_COLOR
    );

    void drawPoints(
        const Group<Point2d> &pointsGroup,
        Image *frame,
        const cv::Scalar &color = DEFAULT_POINT_COLOR);

    void drawStripe(const StripeGroup &stripeGroup, Image *frame);

    void drawDetectionLines(const ContourVec &contourVec, Image *frame);
}