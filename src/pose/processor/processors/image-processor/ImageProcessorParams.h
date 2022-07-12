#pragma once

#include <opencv2/opencv.hpp>

namespace pose
{
    struct AdaptiveTHImageProcessorParams
    {
        unsigned int maxIntensity = 255; // [0-255]
        unsigned int blockSize = 179;    // must be an odd int
        int constant = 50;               // [0-150]
        unsigned short type = 1;         // 0=binary, 1=gaussian
    };

    struct ContourVecImageProcessorParams
    {
        unsigned short retrievalMode = 3; // 0=external, 1=list, 2=ccomp, 3=tree, 4=floodfill
        unsigned short apprxMode = 2;     // 1=none, 2=simple, 3=tc89_l1, 4=tc89_kcos
    };

    struct SobelImageProcessorParams
    {
        bool isHorizontal = true;
    };
}