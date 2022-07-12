#include "MarkerWindow.h"

namespace pose
{
    void MarkerWindow::init(Vec<unsigned int> codes)
    {
        for(auto code: codes)
        {
            std::string title = "Marker: " + std::to_string(code);
            windowMap[code] = std::make_shared<Window>(title);
        }
    }

    void MarkerWindow::display(const Vec<Marker> &markers)
    {
        for(auto marker: markers)
        {
            if(!windowMap.contains(marker.getCode()))
                continue;

            Image img = marker.getImage();
		    cv::resize(img, img, cv::Size(300, 300), 0, 0, cv::INTER_NEAREST);
            windowMap[marker.getCode()]->display(img);
        }
    }
}