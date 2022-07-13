#pragma once

#include "pose/types.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace pose
{
    class Marker
    {
    public:
        Marker(const int (&codes)[4], const Contourf &corners, const Image& img);

        unsigned int getCode() const { return code; }
        const Contourf &getCorners() const { return corners; }
        const Image &getImage() const { return img; }
        glm::mat4 getTransformation() const { return transformation;}

    private:
        int code = 0; // min code among the 4 rotation
        glm::mat4 transformation; // TODO: extract into a transformation entity
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
