#pragma once

#include "graphics/gfx.h"
#include "config/config.h"

namespace game
{
    bool isInsideAABB(glm::vec3 p, float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
    
    bool sphereAABBCollision(
        const gfx::Sphere &sphere,
        const gfx::Cube &cube,
        const glm::vec3 &previousT,
        glm::vec3 &newPoint,
        glm::vec3 &normal);
}