#pragma once

#include "pose/types.h"
#include "../Marker.h"
#include "pose/processor/processors/image-processor/ImageProcessor.h"
#include "pose/processor/processors/contour-vec-processor/ContourVecProcessor.h"
#include "pose/processor/processors/contourf-vec-processor/ContourfVecProcessor.h"
#include "pose/processor/GroupProcessor.h"
#include "pose/processor/VecProcessor.h"
#include "pose/window/Window.h"

namespace pose
{
    class MarkerDetector
    {
    public:
        const Vec<Marker>& getMarkersFromImage(Image &image);

    private:
        // processor
        GrayScaleImageProcessor grayScaleIP;
        AdaptiveTHImageProcessor adaptiveTHIP;
        DetectContourVecImageProcessor detectContourVecIP;
        ApprxPolysContourVecProcessor apprxPolysCP;
        FilterPolysContourVecProcessor filterPolysCP;
        SubDivideContourVecProcessor subDivideCP;
        PreciseCornersContourVecProcessor preciseEdgesCP;
        DetectMarkersContourfVecProcessor detectMarkersCP;
        ProcessorChain chain;

    private:
        void init();
    };
}