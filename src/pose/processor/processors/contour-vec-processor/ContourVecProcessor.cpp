#include <opencv2/opencv.hpp>
#include "ContourVecProcessor.h"

namespace pose
{
    void ApprxPolysContourVecProcessor::process(const ContourVec &input, ContourVec *output)
    {
        result.clear();

        for (const auto &contour : input)
        {
            Contour contourApprx;
            cv::approxPolyDP(
                contour,
                contourApprx,
                cv::arcLength(contour, true) * params.epsilonCoefficient,
                params.isClosedCurve);
            result.push_back(std::move(contourApprx));
        }
    }

    void FilterPolysContourVecProcessor::process(const ContourVec &input, ContourVec *output)
    {
        result.clear();

        for (const auto &contour : input)
        {
            if ((contour.size() != params.cornerCount) ||      // corner count
                (cv::contourArea(contour) < params.minArea) || // min area
                (cv::contourArea(contour) > params.maxArea)    // min area)
            )
                continue;

            result.push_back(std::move(contour));
        }
    }

    void SubDivideContourVecProcessor::process(const ContourVec &input, ContourVec *output)
    {
        result.clear();

        for (const auto &contour : input)
        {
            Contour contourResult;
            for (size_t j = 0; j < contour.size(); j++)
            {
                const cv::Point &p = contour[j];
                cv::Point pNext = j == contour.size() - 1 ? contour[0] : contour[j + 1];

                contourResult.push_back(std::move(p));

                const double MAX = params.amount;
                for (int k = 1; k < MAX; k++)
                {
                    double t = k / MAX;
                    double px = (t)*pNext.x + (1 - t) * p.x;
                    double py = (t)*pNext.y + (1 - t) * p.y;

                    contourResult.push_back(std::move(cv::Point(px, py)));
                }
            }
            result.push_back(std::move(contourResult));
        }
    }

    void PreciseCornersContourVecProcessor::process(const ContourVec &input, Group<cv::Point2f> *output)
    {
        result.clear();

        for (const auto &contour : input)
        {
            // get precise edge points
            Vec<Point2d> points;
            for (const auto &stripe : getStripes(contour))
            {
                const Image subPixImage = getSubPixImage(stripe);
                const Image sobelImage = getSobelImage(subPixImage);
                const cv::Point2d precisePoint = getPrecisePoint(stripe, sobelImage);
                points.push_back(precisePoint);
            }

            // fit lines to the edge points
            float lines[16];
            fitLines(points, lines);

            // get precise corners
            Vec<cv::Point2f> edges = getEdges(lines);

            result.push_back(std::move(edges));
        }
    }

    StripeVec PreciseCornersContourVecProcessor::getStripes(const Contour &contour)
    {
        StripeVec stripeVec;
        double dx = 0, dy = 0;
        for (size_t i = 0; i < contour.size(); i++)
        {
            // if current point is an edge update the euclidean distance
            if (i % params.edgeAtEvery == 0)
            {
                dx = ((double)contour[(i + params.edgeAtEvery) % contour.size()].x - (double)contour[i].x);
                dy = ((double)contour[(i + params.edgeAtEvery) % contour.size()].y - (double)contour[i].y);
                continue;
            }

            Stripe st;

            st.center = contour[i];

            double edgeLength = sqrt(dx * dx + dy * dy);

            // make the stripe length proportional to the edge length (as it can differ due to the perspective)
            st.length = (int)(params.lengthCoeff * edgeLength);

            if (st.length < 5)
                st.length = 5;

            // make stripe length odd as both sides will have the same length and +1 pixel for the center
            if (st.length % 2 == 0)
                st.length++;

            st.nStop = st.length / 2;
            st.nStart = -st.nStop;

            // normalize the direction vector
            st.up = cv::Point2d(dx, dy) * (1.0 / edgeLength);
            st.right = cv::Point2d(st.up.y, -st.up.x);

            stripeVec.push_back(std::move(st));
        }
        return stripeVec;
    }

    Image PreciseCornersContourVecProcessor::getSubPixImage(const Stripe &stripe)
    {
        Image imageStripe(cv::Size(stripe.length, stripe.height), CV_8UC1);

        for (int m = -1; m <= 1; ++m)
        {
            for (int n = stripe.nStart; n <= stripe.nStop; n++)
            {
                cv::Point2f sp; // sub point

                sp.x = stripe.center.x + ((double)m * stripe.up.x) + ((double)n * stripe.right.x);
                sp.y = stripe.center.y + ((double)m * stripe.up.y) + ((double)n * stripe.right.y);

                // evaluate the subpixel intensity
                int intensity;

                int fx = int(floorf(sp.x));
                int fy = int(floorf(sp.y));

                if (fx < 0 || fx >= (*img).cols - 1 ||
                    fy < 0 || fy >= (*img).rows - 1)
                {
                    intensity = 127;
                    continue;
                }

                int coeffX = int(256 * (sp.x - floorf(sp.x)));
                int coeffY = int(256 * (sp.y - floorf(sp.y)));

                // Here we get the pixel of the starting point
                unsigned char *i = (unsigned char *)(((*img).data + fy * (*img).step) + fx);

                int a = i[0] + ((coeffX * (i[1] - i[0])) >> 8);
                i += (*img).step;
                int b = i[0] + ((coeffY * (i[1] - i[0])) >> 8);

                intensity = a + ((coeffY * (b - a)) >> 8);

                // fill the image
                int u = m + 1;                    // -1,0,1 -> 0,1,2
                int r = n + (stripe.length >> 1); // [-length/2, length/2] -> [0, length]

                imageStripe.at<uchar>(u, r) = (uchar)intensity;
            }
        }

        return imageStripe;
    }

    Image PreciseCornersContourVecProcessor::getSobelImage(const Image &image)
    {
        Image out;

        cv::Sobel(image, out, CV_8UC1, 1, 0);

        return out;
    }

    cv::Point2d PreciseCornersContourVecProcessor::getPrecisePoint(const Stripe &stripe, const Image &sobelImage)
    {
        double maxIntensity = -1;
        int maxIntensityIndex = 0;

        // Finding the max value
        for (int n = 0; n < stripe.length - 2; ++n)
        {
            if (sobelImage.at<uchar>(n, 1) > maxIntensity)
            {
                maxIntensity = sobelImage.at<uchar>(n, 1);
                maxIntensityIndex = n;
            }
        }

        double y0, y1, y2;

        // Point before and after
        unsigned int max1 = maxIntensityIndex - 1, max2 = maxIntensityIndex + 1;

        y0 = (maxIntensityIndex <= 0) ? 0 : sobelImage.at<uchar>(max1, 1);
        y1 = sobelImage.at<uchar>(maxIntensityIndex, 1);
        y2 = (maxIntensityIndex >= stripe.length - 3) ? 0 : sobelImage.at<uchar>(max2, 1);
        double pos = (y2 - y0) / (4 * y1 - 2 * y0 - 2 * y2);

        if (isnan(pos))
            return stripe.center;

        // Exact point with subpixel accuracy
        cv::Point2d edgeCenter;

        // Where is the edge (max gradient) in the picture?
        int maxIndexShift = maxIntensityIndex - (stripe.length >> 1);

        // Find the original edgepoint -> Is the pixel point at the top or bottom?
        edgeCenter.x = (double)stripe.center.x + (((double)maxIndexShift + pos) * stripe.right.x);
        edgeCenter.y = (double)stripe.center.y + (((double)maxIndexShift + pos) * stripe.right.y);

        return edgeCenter;
    }

    void PreciseCornersContourVecProcessor::fitLines(const Vec<Point2d> &points, float (&lineParams)[16])
    {
        unsigned int step = params.edgeAtEvery - 1;

        cv::Mat lineParamsMat(cv::Size(4, 4), CV_32F, lineParams);

        for (int i = 0; i < points.size(); i += step)
        {
            auto edgePoints = std::vector<cv::Point2f>(points.begin() + i, points.begin() + i + step);
            cv::Mat pointMat(cv::Size(1, step), CV_32FC2, edgePoints.data());

            cv::fitLine(pointMat, lineParamsMat.col(i / step), CV_DIST_L2, 0, 0.01, 0.01);
        }
    }

    Vec<cv::Point2f> PreciseCornersContourVecProcessor::getEdges(const float (&lines)[16])
    {
        Vec<cv::Point2f> edges;
        for (int i = 0; i < 4; i++)
        {
            // line 1
            float dx0 = lines[i] * 10;
            float dy0 = lines[i + 4] * 10;
            float x0 = lines[i + 8];
            float y0 = lines[i + 12];
            float x1 = x0 + dx0;
            float y1 = y0 + dy0;

            float m1 = (y1 - y0) / (x1 - x0);
            float c1 = y0 - (m1 * x0);

            // line 2
            int j = (i + 1) % 4;
            float dx2 = lines[j] * 10;
            float dy2 = lines[j + 4] * 10;
            float x2 = lines[j + 8];
            float y2 = lines[j + 12];
            float x3 = x2 + dx2;
            float y3 = y2 + dy2;

            float m2 = (y3 - y2) / (x3 - x2);
            float c2 = y2 - (m2 * x2);

            // find the intersection
            float x = (c2 - c1) / (m1 - m2);
            float y = m1 * x + c1;

            edges.push_back(cv::Point2f(x, y));
        }

        return edges;
    }

}