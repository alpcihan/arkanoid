#pragma once

#include "utils/utils.h"
#include "graphics/gfx.h"
#define path(x) path::resource(x)

namespace arcanoid
{
    class Game
    {   
        public:
            // props
            std::shared_ptr<gfx::Texture> bgTexture;
            glm::mat4 transformation;
            
        public:
            Game() = default;
            void init(void (*callback)());
            void run();

        private:
            // scene
            std::shared_ptr<gfx::Cube> player;
            std::shared_ptr<gfx::Plane> background;
            std::vector<std::shared_ptr<gfx::Object3D>> bricks;

            // window
            std::unique_ptr<gfx::Window> window;

            // callback on update
            void (*callback)();
        
        private:
            void onUpdate();
    };
}