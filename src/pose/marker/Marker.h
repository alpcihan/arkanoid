#pragma once

#include "pose/types.h"

namespace pose
{
    class Marker
    {
    public:
        Marker(const int (&codes)[4], const Contourf &corners, const Image& img);

        unsigned int getCode() const { return code; }
        const Contourf &getCorners() const { return corners; }
        const Image &getImage() const { return img; }

    private:
        int code = 0; // min code among the 4 rotation
        float transformation[16]; // TODO: extract into a transformation entity
        int codes[4]; // codes for 4 rotation
        int rotation = 0;
        Contourf corners;
        Image img;

    private:
        void initialize();
        void setMinCodeAndRotation();
        void setCornerOrder();
        void estimateTransformation();
    };
}
