#pragma once

#include <opencv2/opencv.hpp>
#include "pose/processor/Processor.h"
#include "pose/types.h"
#include "pose/marker/Marker.h"

namespace pose
{
    template <class Output>
    class ContourfVecProcessor : public Processor<ContourfVec, Output>
    {
    };

    class DetectMarkersContourfVecProcessor : public ContourfVecProcessor<Vec<Marker>>
    {
    public:
        const Image* img;

    public:
        void process(const ContourfVec &input, Vec<Marker> *output) override;

    private:
        cv::Mat getImageWithoutPerspective(const Contourf &edges) const;
        bool isAMarker(Image& image) const;
        void getMarkerCodes(int (&codes)[4], const Image &image) const;
    };
}