#include "draw.h"

namespace pose
{
    void drawContourVec(const ContourVec &contourVec, Image *frame, const cv::Scalar &color)
    {
        cv::drawContours(*frame, contourVec, -1, color, 1);
    }

    void drawPoints(
        const ContourVec &contourVec,
        Image *frame,
        unsigned int repeatsEvery,
        const cv::Scalar &color1,
        const cv::Scalar &color2)
    {
        for (const auto &contour : contourVec)
        {
            for (unsigned int i = 0; i < contour.size(); i++)
            {
                const auto &point = contour[i];

                if (i % repeatsEvery == 0)
                {
                    circle(*frame, point, 2, color1, -1);
                    continue;
                }

                circle(*frame, point, 2, color2, -1);
            }
        }
    }

    void drawPoints(
        const Group<Point2d> &pointGroup,
        Image *frame,
        const cv::Scalar &color)
    {
        for (const auto &pointVec : pointGroup)
        {
            for (const auto &point : pointVec)
            {
                circle(*frame, point, 2, color, -1);
            }
        }
    }

    void drawPoints(
        const Group<cv::Point2f> &pointGroup,
        Image *frame,
        const cv::Scalar &color)
    {
        for (const auto &pointVec : pointGroup)
        {
            for (const auto &point : pointVec)
            {
                circle(*frame, point, 2, color, -1);
            }
        }
    }

    void drawStripe(const StripeGroup &stripeGroup, Image *frame)
    {
        for (auto &stripeVec : stripeGroup)
        {
            for (auto &stripe : stripeVec)
            {
                int topX = ceil(stripe.center.x + stripe.length * 0.5 * (stripe.right.x + stripe.up.x));
                int topY = ceil(stripe.center.y + stripe.height * (stripe.right.y + stripe.up.y));

                int botX = ceil(stripe.center.x - stripe.length * 0.5 * (stripe.right.x + stripe.up.x));
                int botY = ceil(stripe.center.y - stripe.height * (stripe.right.y + stripe.up.y));

                cv::rectangle(*frame, cv::Point(topX, topY), cv::Point(botX, botY), cv::Scalar(255, 255, 0), 1);
            }
        }
    }

    void drawDetectionLines(const ContourVec &contourVec, Image *frame)
	{
		cv::drawContours(*frame, contourVec, -1, cv::Scalar(0, 0, 255), 2);
		for (const auto &contour : contourVec)
		{
			for (size_t j = 0; j < contour.size(); j++)
			{
				cv::Point p = contour[j];
				cv::Point pNext = j == contour.size() - 1 ? contour[0] : contour[j + 1];

				circle(*frame, p, 2, CV_RGB(0, 255, 0), -1);

				const double MAX = 6.0;
				for (int k = 1; k < MAX; k++)
				{
					double t = k / MAX;
					double px = (t)*pNext.x + (1 - t) * p.x;
					double py = (t)*pNext.y + (1 - t) * p.y;

					circle(*frame, cv::Point(px, py), 2, CV_RGB(0, 0, 255), -1);
				}
			}
		}
	};
}