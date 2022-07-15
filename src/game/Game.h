#pragma once

#include "helpers/help.h"
#include "graphics/gfx.h"
#include "config/config.h"
#include "game-objects/game-objects.h"
#define path(x) help::toResourcePath(x)

namespace game
{
    class Game
    {   
        public:
            // props
            std::shared_ptr<gfx::Texture> bgTexture;
            glm::mat4 extrinsicMat;
            bool isMarkerDetected = false;
            
        public:
            Game() = default;
            void init(void (*callback)());
            void run();

        private:
            // scene
            Player player;
            Ball ball;
            std::vector<Brick> bricks;
            std::vector<std::shared_ptr<gfx::Cube>> walls;
            std::shared_ptr<gfx::Plane> background;
            std::vector<std::shared_ptr<gfx::Cube>> healthBar;

            // window
            std::unique_ptr<gfx::Window> window;

            // callback on update
            void (*callback)();
        
        private:
            void onUpdate();
            void collisionUpdate();
            void draw();     
    };

      
}