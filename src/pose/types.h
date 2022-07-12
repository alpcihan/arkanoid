#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace pose
{
    typedef std::vector<cv::Point> Contour;
    typedef std::vector<Contour> ContourVec;

    typedef std::vector<cv::Point2f> Contourf;
    typedef std::vector<Contourf> ContourfVec;

    struct Stripe
    {   
        int height = 3;
        int length;
        int nStop;
        int nStart;
        cv::Point center;
        cv::Point2d up;
        cv::Point2d right;
    };
    
    typedef std::vector<Stripe> StripeVec;
    typedef std::vector<StripeVec> StripeGroup;

    typedef cv::Mat Image;
    typedef std::vector<Image> ImageVec;
    typedef std::vector<ImageVec> ImageGroup;

    template<class T>
    using Vec = std::vector<T>;

    template<class T>
    using Group = Vec<std::vector<T>>;

    typedef cv::Point2d Point2d;
}