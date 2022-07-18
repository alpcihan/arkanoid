#include "physics.h"
#include <glm/gtx/intersect.hpp>

namespace game
{
    bool isInsideAABB(glm::vec3 p, float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
    {
        return (
            (p.x >= minX) && (p.x <= maxX) &&
            (p.y >= minY) && (p.y <= maxY) &&
            (p.z >= minZ) && (p.z <= maxZ));
    }

    bool sphereAABBCollision(
        const gfx::Sphere &sphere,
        const gfx::Cube &cube,
        const glm::vec3 &previousT,
        glm::vec3 &newPoint,
        glm::vec3 &normal)
    {
        glm::vec3 st = sphere.translation;
        glm::vec3 ss = sphere.scale;

        glm::vec3 ct = cube.translation;
        glm::vec3 cs = cube.scale * 0.5f;

        glm::vec3 cubeCorners[8] = {
            ct + glm::vec3(+cs.x, +cs.y, +cs.z),
            ct + glm::vec3(-cs.x, +cs.y, +cs.z),
            ct + glm::vec3(+cs.x, -cs.y, +cs.z),
            ct + glm::vec3(+cs.x, +cs.y, -cs.z),
            ct + glm::vec3(+cs.x, -cs.y, -cs.z),
            ct + glm::vec3(-cs.x, -cs.y, +cs.z),
            ct + glm::vec3(-cs.x, +cs.y, -cs.z),
            ct + glm::vec3(-cs.x, -cs.y, -cs.z)};

        glm::vec3 cubeSurfaceCenters[6] = {
            ct + glm::vec3(+cs.x, 0, 0),
            ct + glm::vec3(-cs.x, 0, 0),
            ct + glm::vec3(0, +cs.y, 0),
            ct + glm::vec3(0, -cs.y, 0),
            ct + glm::vec3(0, 0, +cs.z),
            ct + glm::vec3(0, 0, -cs.z),
        };

        float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;
        for (int i = 0; i < 8; i++)
        {
            auto corner = cubeCorners[i];

            minX = corner.x < minX ? corner.x : minX;
            maxX = corner.x > maxX ? corner.x : maxX;
            minY = corner.y < minY ? corner.y : minY;
            maxY = corner.y > maxY ? corner.y : maxY;
            minZ = corner.z < minZ ? corner.z : minZ;
            maxZ = corner.z > maxZ ? corner.z : maxZ;
        }
        // get box closest point to sphere center by clamping
        auto x = std::max(minX, std::min(st.x, maxX));
        auto y = std::max(minY, std::min(st.y, maxY));
        auto z = std::max(minZ, std::min(st.z, maxZ));

        float distance = glm::distance(glm::vec3(x, y, z), st);

        // only if sphere got scaled equally at each direction
        float radius = ss.x;

        // if there is collision
        if (distance < radius)
        {
            float minDist = FLT_MAX;
            glm::vec3 minNorm;
            glm::vec3 minDir;

            for (int i = 0; i < 6; i++)
            {
                const glm::vec3 planeOrig = cubeSurfaceCenters[i];
                const glm::vec3 planeNormal = glm::normalize(planeOrig - ct);
                const glm::vec3 orig = st - planeNormal*ss.x;
                const glm::vec3 prev = previousT - planeNormal*ss.x;
                const glm::vec3 dir = glm::normalize(prev-orig);
                float distance = 0;

                // if intersects with the cube surface
                if (glm::intersectRayPlane(orig, dir, planeOrig, planeNormal, distance))
                {
                    if (distance < minDist)
                    {
                        minDist = distance;
                        minNorm = planeNormal;
                        minDir = dir;
                    }
                }
            }
            glm::vec3 hitPoint = (st - minNorm*ss.x) + glm::normalize(minDir) * minDist;
            glm::vec3 dir = glm::reflect(-minDir, minNorm);

            newPoint = hitPoint + dir*minDist + minNorm*ss.x;
            normal = minNorm;
            
            return true;
        }

        normal = glm::vec3(0);
        return false;
    }
}