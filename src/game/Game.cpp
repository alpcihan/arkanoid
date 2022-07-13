#include "Game.h"
#include "utils/utils.h"

namespace arcanoid
{
    void Game::init(void (*callback)())
    {
        // set the callback
        this->callback = callback;

        // init window + opengl
        window = std::make_unique<gfx::Window>(gfx::WindowProps(WIDTH, HEIGHT));

        // init scene
        player = std::make_shared<gfx::Cube>(
            std::make_shared<gfx::Texture>(path("texture/pepe-bob.jpg")),
            std::make_shared<gfx::Shader>(path("shader/default.shader")));

        background = std::make_shared<gfx::Plane>(
            std::make_shared<gfx::Texture>(path("texture/blank.jpg")),
            std::make_shared<gfx::Shader>(path("shader/background.shader")));
    }

    void Game::run()
    {
        while (!window->isClosed())
        {
            window->clear();

            this->callback();
            this->onUpdate();
            window->update();
        }
    }

    void Game::onUpdate()
    {
        // mvp
        player->translation = glm::vec3(0.0, 0.0, -0.05);
        player->scale = glm::vec3(0.04,0.04,0.1);

        auto mvp = glm::perspective(glm::radians(30.f), ((float)WIDTH)/HEIGHT, 0.1f, 1000.f) * transformation * player->getTransform();
        player->shader->setMat4("u_mvp", mvp);

        // draw calls
        background->texture = bgTexture;
        background->draw();
        player->draw();
    }

}
