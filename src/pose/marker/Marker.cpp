#include "Marker.h"
#include "pose/math/pose-estimation.h"
#include "config/config.h"

namespace pose
{
    Marker::Marker(const int (&codes)[4], const Contourf &corners, const Image &img)
        : corners(corners), img(img)
    {
        for (int i = 0; i < 4; i++)
            this->codes[i] = codes[i];

        initialize();
    }

    void Marker::initialize()
    {
        setMinCodeAndRotation();
        setCornerOrder();
        estimateTransformation();
    }

    void Marker::setMinCodeAndRotation()
    {
        code = codes[0];
        for (int i = 1; i < 4; ++i)
        {
            if (codes[i] < code)
            {
                code = codes[i];
                rotation = i;
            }
        }
    }

    void Marker::setCornerOrder()
    {
        // if the rotation is 0 no sorting is needed
        if (rotation != 0)
        {
            cv::Point2f corrected_corners[4];
            // Smallest id represents the x-axis, we put the values in the corrected_corners array
            for (int i = 0; i < 4; i++)
                corrected_corners[(i + rotation) % 4] = corners[i];
            // Put the values back in the array in the sorted order
            for (int i = 0; i < 4; i++)
                corners[i] = corrected_corners[i];
        }
    }

    void Marker::estimateTransformation()
    {
        cv::Point2f cornersCameraCoord[4];
 
        for (int i = 0; i < 4; i++)
        {
            float x = corners[i].x - (WIDTH * 0.5);
            float y = -corners[i].y + (HEIGHT * 0.5);

            cornersCameraCoord[(i + 2)%4] = cv::Point2f(x, y);
        }

        float mtrx[16];
        estimateSquarePose(mtrx, (cv::Point2f *)cornersCameraCoord, 0.04346);

        // Transpose -> columns to rows because of OpenGL representation on the GPU
        float transposed[16];

        for (int x = 0; x < 4; ++x)
            for (int y = 0; y < 4; ++y)
                transposed[x * 4 + y] = mtrx[y * 4 + x];

        memcpy(glm::value_ptr(this->transformation), transposed, sizeof(transposed));
    }
}