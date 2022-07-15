#pragma once

#include "graphics/gfx.h"
#include "config/config.h"

namespace game
{
    struct Brick
    {
    public:
        int health = BRICK_HEALTH;
        std::shared_ptr<gfx::Cube> obj;
    };

    struct Player
    {
    public:
        int life = PLAYER_HEALTH;
        std::shared_ptr<gfx::Cube> obj;
    };

    struct Ball
    {
    public:
        glm::vec3 velocity = glm::vec3(0);
        glm::vec3 previousPos = glm::vec3(0); // for collision detection
        std::shared_ptr<gfx::Sphere> obj;
        
    public:
        void setPos(glm::vec3 pos)
        {
            previousPos = obj->translation;
            obj->translation = pos;
        }
        
        void move(glm::vec3 velocity)
        {
            previousPos = obj->translation;
            obj->translation += velocity;
        }
        
    };
}