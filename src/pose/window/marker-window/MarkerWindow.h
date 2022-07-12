#pragma once

#include "pose/types.h"
#include "pose/marker/Marker.h"
#include "../Window.h"
#include <unordered_map>

namespace pose
{
    class MarkerWindow
    {
    public:
        MarkerWindow() = default;

        void init(Vec<unsigned int> codes);
        void display(const Vec<Marker>& markers);
    
    private:
        std::unordered_map<unsigned int, std::shared_ptr<Window>> windowMap;
    };
}