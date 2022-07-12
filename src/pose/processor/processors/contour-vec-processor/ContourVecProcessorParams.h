#pragma once

namespace pose
{
    struct ApprxPolyContourVecProcessorParams
    {
        double epsilonCoefficient = 0.02; // [0.0-1.0]
        bool isClosedCurve = 1;           // 0=false, 1=true
    };

    struct FilterPolysContourVecProcessorParams
    {
        unsigned int cornerCount = 4;
        unsigned int minArea = 570;   // TODO: make it proportional to the image resolution
        unsigned int maxArea = 15000; // TODO: make it proportional to the image resolution
        double minMaxEdgeRatio = 2.0; // [0.0-5.0]
    };

    struct SubDivideContourVecProcessorParams
    {
        unsigned int amount = 7;
    };

    struct PreciseCornersContourVecProcessorParams
    {
        unsigned int edgeAtEvery = 7;
        float lengthCoeff = 0.08;
    };
}