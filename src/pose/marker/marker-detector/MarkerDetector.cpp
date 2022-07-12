#include "MarkerDetector.h"

namespace pose
{
    const Vec<Marker> &MarkerDetector::getMarkersFromImage(Image &image)
    {
        static bool called = false;
        if (!called)
        {
            init();
            called = true;
        }

        preciseEdgesCP.img = &grayScaleIP.getLastResult();
        detectMarkersCP.img = &adaptiveTHIP.getLastResult();
        chain.process(&image);

        return detectMarkersCP.getLastResult();
    }

    void MarkerDetector::init()
    {
        chain.add(grayScaleIP);
        chain.add(adaptiveTHIP);
        chain.add(detectContourVecIP);
        chain.add(apprxPolysCP);
        chain.add(filterPolysCP);
        chain.add(subDivideCP);
        chain.add(preciseEdgesCP);
        chain.add(detectMarkersCP);
    }

}
