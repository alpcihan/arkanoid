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
            glm::mat4 extrinsicMatPlayer;
            glm::mat4 extrinsicMatButton;
            bool isMarkerDetected = false;
            bool isPlayerMarkerDetected = false;
            bool isButtonMarkerDetected = false;
            
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
            std::shared_ptr<gfx::Cube> controller;
            std::shared_ptr<gfx::Cube> button;
            std::shared_ptr<gfx::Cube> endScreen;

            // window
            std::unique_ptr<gfx::Window> window;

            // callback on update
            void (*callback)();
        
        private:
            void onUpdate();
            void collisionUpdate();
            void draw();     
            void replay();
    };

      
}