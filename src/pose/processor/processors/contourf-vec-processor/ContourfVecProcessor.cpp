#include <opencv2/opencv.hpp>
#include "ContourfVecProcessor.h"
#include "pose/window/Window.h"

namespace pose
{
    void DetectMarkersContourfVecProcessor::process(const ContourfVec &input, Vec<Marker> *output)
    {
        result.clear();

        for (const auto &corners : input)
        {
            cv::Mat imgMark = getImageWithoutPerspective(corners);

            if (!isAMarker(imgMark))
                continue;

            int codes[4];
            getMarkerCodes(codes, imgMark);

            Marker marker(codes, corners, imgMark);

            result.push_back(std::move(marker));
        }
    }

    cv::Mat DetectMarkersContourfVecProcessor::getImageWithoutPerspective(const Contourf &corners) const
    {
        cv::Mat imgMark;

        Vec<cv::Point2f> target = {
            cv::Point2f(5.5, 5.5),
            cv::Point2f(-0.5, 5.5),
            cv::Point2f(-0.5, -0.5),
            cv::Point2f(5.5, -0.5),
        };

        cv::Mat perspective = cv::getPerspectiveTransform(corners.data(), target.data());
        cv::warpPerspective(*this->img, imgMark, perspective, cv::Size(6, 6));
        cv::threshold(imgMark, imgMark, 254, 255, CV_THRESH_BINARY);

        return imgMark;
    }

    bool DetectMarkersContourfVecProcessor::isAMarker(Image &image) const
    {
        // check the borders
        int count = 0;
        for (int i = 0; i < 6; ++i)
        {
            int pix1 = image.at<uchar>(0, i); // top
            int pix2 = image.at<uchar>(5, i); // bottom
            int pix3 = image.at<uchar>(i, 0); // left
            int pix4 = image.at<uchar>(i, 5); // right

            count = pix1 + pix2 + pix3 + pix4;

            image.at<uchar>(0, i) = 0;
            image.at<uchar>(5, i) = 0;
            image.at<uchar>(i, 0) = 0;
            image.at<uchar>(i, 5) = 0;
        }

        // some border pixel might appear white due to noise
        // let at most 2 white pixel to be counted as border
        if (count > 2)
            return false;

        return true;
    }

    void DetectMarkersContourfVecProcessor::getMarkerCodes(int (&codes)[4], const Image &image) const
    {
        // Copy the BW values into cP -> codePixel on the marker 4x4 (inner part of the marker, no black border)
        int cP[4][4];
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                // +1 -> no borders!
                cP[i][j] = image.at<uchar>(i + 1, j + 1);
                // If black then 1 else 0
                cP[i][j] = (cP[i][j] == 0) ? 1 : 0;
            }
        }

        // for each rotation of the marker image
        codes[0] = codes[1] = codes[2] = codes[3] = 0;

        // Calculate the code from all sides at once
        for (int i = 0; i < 16; i++)
        {
            // /4 to go through the rows
            int row = i >> 2;
            int col = i % 4;

            // Multiplied by 2 to check for black values -> 0*2 = 0
            codes[0] <<= 1;
            codes[0] |= cP[row][col]; // 0

            // 4x4 structure -> Each column represents one side
            codes[1] <<= 1;
            codes[1] |= cP[3 - col][row]; // 90

            codes[2] <<= 1;
            codes[2] |= cP[3 - row][3 - col]; // 180

            codes[3] <<= 1;
            codes[3] |= cP[col][3 - row]; // 270
        }
    }
}