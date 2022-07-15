#include "Game.h"
#include "physics/physics.h"

// brick health
// player health

bool isGameOver = false;
bool isBallMoved = false;
int frameCount = 0;

namespace game
{
    void Game::init(void (*callback)())
    {
        // set the callback
        this->callback = callback;

        // init window + opengl
        window = std::make_unique<gfx::Window>(gfx::WindowProps(WIDTH, HEIGHT));
        auto shader = std::make_shared<gfx::Shader>(path("shader/default.shader"));

        // player
        player.obj = std::make_shared<gfx::Cube>(
            std::make_shared<gfx::Texture>(path("texture/player.jpg")),
            shader);
        player.obj->translation = glm::vec3(0.0, 0.0, -FLOOR_OFFSET * 0.5);
        player.obj->scale = glm::vec3(0.04, 0.04, FLOOR_OFFSET);

        // ball
        ball.obj = std::make_shared<gfx::Sphere>(
            std::make_shared<gfx::Texture>(path("texture/pepe-bob.jpg")),
            std::make_shared<gfx::Shader>(path("shader/ball.shader")));
        ball.obj->shader->setVec3("u_color", glm::vec3(0.5, 0.5, 0.1));
        ball.obj->scale = glm::vec3(BALL_SCALE);

        // bricks
        int coeff = 3;
        for (int z = 0; z < coeff; z++)
        {
            for (int y = -coeff + z; y < coeff - z; y++)
            {
                for (int x = -coeff + z; x < coeff - z; x++)
                {
                    Brick brick;

                    brick.obj = std::make_shared<gfx::Cube>(
                        std::make_shared<gfx::Texture>(path("texture/blank.jpg")),
                        std::make_shared<gfx::Shader>(path("shader/brick.shader")));
                    if(z==0) brick.obj->shader->setVec3("u_color", glm::vec3(1.0, 0.0 ,0.0));
                    if(z==1) brick.obj->shader->setVec3("u_color", glm::vec3(0.0, 1.0 ,0.0));
                    if(z==2) brick.obj->shader->setVec3("u_color", glm::vec3(0.1, 0.1 ,1.0));
                    brick.obj->shader->setFloat("u_alpha", 0.2);

                    brick.obj->scale = glm::vec3(BRICK_SCALE);
                    brick.obj->translation = glm::vec3(BRICK_SCALE * x, BRICK_SCALE * y, -0.05 - BRICK_SCALE * z);

                    bricks.push_back(brick);
                }
            }
        }

        // walls
        for (int i = 0; i < 5; i++)
        {
            auto wall = std::make_shared<gfx::Cube>(
                std::make_shared<gfx::Texture>(path("texture/wall.jpg")),
                std::make_shared<gfx::Shader>(path("shader/wall.shader")));
            wall->shader->setFloat("u_alpha", 1.0f);
            walls.push_back(wall);
        }

        // front right -up
        walls[0]->scale = glm::vec3(WALL_WIDTH, WALL_LENGHT, WALL_HEIGHT);
        walls[0]->translation = glm::vec3(-0.5 * (WALL_HEIGHT + WALL_WIDTH), 0.0, -0.5 * (WALL_HEIGHT + WALL_WIDTH));

        walls[1]->scale = glm::vec3(WALL_WIDTH, WALL_LENGHT, WALL_HEIGHT);
        walls[1]->translation = glm::vec3(0.5 * (WALL_HEIGHT + WALL_WIDTH), 0.0, -0.5 * (WALL_HEIGHT + WALL_WIDTH));

        walls[2]->scale = glm::vec3(WALL_LENGHT, WALL_WIDTH, WALL_HEIGHT);
        walls[2]->translation = glm::vec3(0.0, 0.5 * (WALL_HEIGHT + WALL_WIDTH), -0.5 * (WALL_HEIGHT + WALL_WIDTH));

        walls[3]->scale = glm::vec3(WALL_LENGHT, WALL_WIDTH, WALL_HEIGHT);
        walls[3]->translation = glm::vec3(0.0, -0.5 * (WALL_HEIGHT + WALL_WIDTH), -0.5 * (WALL_HEIGHT + WALL_WIDTH));

        walls[4]->scale = glm::vec3(WALL_LENGHT, WALL_LENGHT, WALL_WIDTH);
        walls[4]->translation = glm::vec3(0.0, 0.0, -(WALL_HEIGHT + WALL_WIDTH));

        // health bar
        for (int i = 0; i < player.life; i++)
        {
            auto life = std::make_shared<gfx::Cube>(
                std::make_shared<gfx::Texture>(path("texture/life.png")),
                std::make_shared<gfx::Shader>(path("shader/life.shader")));
            life->translation = glm::vec3(0.9 - i * 0.12, 0.8, -0.9999);
            life->scale = glm::vec3(0.2 * (HEIGHT / (float)WIDTH), 0.2, 0.000001);
            healthBar.push_back(life);
        }

        // background
        background = std::make_shared<gfx::Plane>(
            std::make_shared<gfx::Texture>(path("texture/blank.jpg")),
            std::make_shared<gfx::Shader>(path("shader/background.shader")));
    }

    void Game::run()
    {
        while (!window->isClosed())
        {
            window->clear(); // clear window

            this->callback(); // call callback
            this->onUpdate(); // call the game's update function

            window->update(); // update the window
        }
    }

    void Game::onUpdate()
    {
        frameCount++;

        // get inputs
        double xpos, ypos;
        glfwGetCursorPos(window->getGLFWwindow(), &xpos, &ypos);
        xpos *= 0.0001, ypos *= 0.0001;
        xpos *= 0.0000, ypos *= 0.0000;

        // move
        player.obj->translation = glm::vec3(-ypos, xpos, 0);

        if (!isBallMoved)
            ball.obj->translation = glm::vec3(player.obj->translation.x, player.obj->translation.y, -FLOOR_OFFSET - ball.obj->scale.x - 0.001f);
        if (frameCount == 100)
        {
            isBallMoved = true;
            ball.velocity = glm::vec3(0, -0.003, -0.0015);
        }

        ball.move(ball.velocity);

        // check the game status
        if (ball.obj->translation.z > 0.1)
        {
            player.life--;
            isBallMoved = false;
        }

        // collision
        collisionUpdate();

        // draw the scene
        draw();
    }

    void Game::collisionUpdate()
    {
        glm::vec3 normal, hitPoint;

        // player-ball
        if (sphereAABBCollision(*ball.obj, *player.obj, ball.previousPos, hitPoint, normal))
        {
            glm::vec3 reflected = glm::reflect(ball.velocity, normal);
            ball.obj->translation = ball.obj->translation + glm::normalize(reflected) * glm::distance(ball.obj->translation, ball.previousPos); // move to stop the collision
            ball.velocity = reflected;
            return;
        }

        // bricks-ball
        for (int i = 0; i < bricks.size(); i++)
        {
            auto &brick = bricks[i];
            if (sphereAABBCollision(*ball.obj, *brick.obj, ball.previousPos, hitPoint, normal))
            {
                glm::vec3 reflected = glm::reflect(ball.velocity, normal);
                ball.obj->translation = ball.obj->translation + glm::normalize(reflected) * glm::distance(ball.obj->translation, ball.previousPos); // move to stop the collision
                ball.velocity = reflected;

                // brick health update
                brick.health--;
                brick.obj->shader->setFloat("u_alpha", 1.0 - 0.4*brick.health);
                if(brick.health==0)
                    bricks.erase(bricks.begin() + i);
                return;
            }
        }

        // walls-ball
        for (auto &wall : walls)
        {
            if (sphereAABBCollision(*ball.obj, *wall, ball.previousPos, hitPoint, normal))
            {
                glm::vec3 reflected = glm::reflect(ball.velocity, normal);
                ball.obj->translation = ball.obj->translation + glm::normalize(reflected) * glm::distance(ball.obj->translation, ball.previousPos); // move to stop the collision
                ball.velocity = reflected;
                return;
            }
        }
    }

    void Game::draw()
    {
        // view projection
        auto vp = glm::perspective(glm::radians(30.f), ((float)WIDTH) / HEIGHT, 0.1f, 1000.f) * extrinsicMat;
        glm::mat4 mvp;

        // background
        background->texture = bgTexture;
        background->draw();

        if(!isMarkerDetected)
            return;
        
        // player
        mvp = vp * player.obj->getTransform();
        player.obj->shader->setMat4("u_mvp", mvp);
        player.obj->draw();

        // ball
        mvp = vp * ball.obj->getTransform();
        ball.obj->shader->setMat4("u_mvp", mvp);
        ball.obj->draw();

        // bricks
        for (auto &brick : bricks)
        {
            auto mvp = vp * brick.obj->getTransform();
            brick.obj->shader->setMat4("u_mvp", mvp);
            brick.obj->draw();
        }

        // walls
        float minDist1 = FLT_MAX, minDist2 = FLT_MAX;
        int idx1, idx2;
        for (int j = 0; j < 2; j++)
        {
            for (int i = 0; i < 5; i++)
            {
                if (j == 1 && i == idx1)
                    continue;

                auto wall = walls[i];
                if (j == 1)
                    wall->shader->setFloat("u_alpha", 1.0f);
                glm::vec3 cam = glm::vec3(glm::inverse(extrinsicMat)[3][0], glm::inverse(extrinsicMat)[3][1], glm::inverse(extrinsicMat)[3][2]);
                float dist = glm::distance(cam, wall->translation);
                if (j == 0 && dist < minDist1)
                {
                    minDist1 = dist;
                    idx1 = i;
                }
                else if (j == 1 && dist < minDist2)
                {
                    minDist2 = dist;
                    idx2 = i;
                }
            }
        }

        walls[idx1]->shader->setFloat("u_alpha", 0.2f);
        walls[idx2]->shader->setFloat("u_alpha", 0.f);

        for (auto &wall : walls)
        {
            auto mvp = vp * wall->getTransform();
            wall->shader->setMat4("u_mvp", mvp);
            wall->draw();
        }

        // health bar
        for (int i = 0; i < player.life; i++)
        {
            auto life = healthBar[i];
            auto model = life->getTransform();
            life->shader->setMat4("u_model", model);
            life->draw();
        }
    }
}
