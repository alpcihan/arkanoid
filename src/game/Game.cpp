#include "Game.h"
#include "physics/physics.h"

// brick health
// player health

bool won = false;
bool isGameOver = false;
bool isBallMoved = false;
int frameCount = 0;
int buttonPressCount = 0;

glm::mat4 prevMat = glm::mat4(0);
bool prevIsPlayerMarkerDetected = false;

namespace game
{
    std::shared_ptr<gfx::Shader> shader;
    std::shared_ptr<gfx::Texture> launchTex;
    std::shared_ptr<gfx::Texture> disableTex;
    std::shared_ptr<gfx::Texture> replayTex;
    std::shared_ptr<gfx::Texture> winTex;
    std::shared_ptr<gfx::Texture> gameOverTex;

    void Game::init(void (*callback)())
    {
        // set the callback
        this->callback = callback;

        // init window + opengl
        window = std::make_unique<gfx::Window>(gfx::WindowProps(WIDTH, HEIGHT, "arkanoid"));

        // set shared shaders and textures
        shader = std::make_shared<gfx::Shader>(path("shader/default.shader"));
        launchTex = std::make_shared<gfx::Texture>(path("texture/button-launch.jpg"));
        disableTex = std::make_shared<gfx::Texture>(path("texture/button-disabled.jpg"));
        replayTex = std::make_shared<gfx::Texture>(path("texture/button-replay.jpg"));
        winTex = std::make_shared<gfx::Texture>(path("texture/win.png"));
        gameOverTex = std::make_shared<gfx::Texture>(path("texture/game-over.jpg"));

        // player
        player.obj = std::make_shared<gfx::Cube>(
            std::make_shared<gfx::Texture>(path("texture/player.jpg")),
            shader);
        player.obj->translation = glm::vec3(0.0, 0.0, -FLOOR_OFFSET * 0.5);
        player.obj->scale = glm::vec3(PLAYER_SIZE, PLAYER_SIZE, FLOOR_OFFSET);

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
                    if (z == 0)
                        brick.obj->shader->setVec3("u_color", glm::vec3(1.0, 0.0, 0.0));
                    if (z == 1)
                        brick.obj->shader->setVec3("u_color", glm::vec3(0.0, 1.0, 0.0));
                    if (z == 2)
                        brick.obj->shader->setVec3("u_color", glm::vec3(0.1, 0.1, 1.0));
                    brick.obj->shader->setFloat("u_alpha", 0.2);

                    brick.obj->scale = glm::vec3(BRICK_SCALE);
                    brick.obj->translation = glm::vec3(BRICK_SCALE * x, BRICK_SCALE * y, -0.045 - BRICK_SCALE * z);

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

        // controller
        controller = std::make_shared<gfx::Cube>(
            std::make_shared<gfx::Texture>(path("texture/move.jpg")),
            shader);
        controller->translation = glm::vec3(0.0, 0.0, -0.000001);
        controller->scale = glm::vec3(0.105, 0.105, 0.00000001);

        // end screen
        endScreen = std::make_shared<gfx::Cube>(
            std::make_shared<gfx::Texture>(path("texture/game-over.jpg")),
            std::make_shared<gfx::Shader>(path("shader/life.shader")));
        endScreen->translation = glm::vec3(0.0, 0.0, -0.9999);
        endScreen->scale = glm::vec3(0.8 * (HEIGHT / (float)WIDTH), 0.8, 0.000001);

        //  button
        button = std::make_shared<gfx::Cube>(
            launchTex,
            std::make_shared<gfx::Shader>(path("shader/button.shader")));
        button->translation = glm::vec3(0.0, 0.0, -0.000001);
        button->scale = glm::vec3(0.105, 0.105, 0.00000001);

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
        if (isButtonMarkerDetected)
            buttonPressCount = 0;

        // get movement delta (x,y) from the controller marker
        float threshold = 0.001;

        auto prevV = glm::vec2(extrinsicMatPlayer[3][0], extrinsicMatPlayer[3][1]);
        auto currentV = glm::vec2(prevMat[3][0], prevMat[3][1]);

        glm::vec2 playerVelocity = (currentV - prevV) * PLAYER_SPEED_CONST;

        // update the player pos by delta
        if ((frameCount > 1) &&
            (glm::distance(currentV, prevV) > threshold) &&
            (prevIsPlayerMarkerDetected && isPlayerMarkerDetected))
        {
            float y = -playerVelocity.y;
            float x = -playerVelocity.x;

            // move the player within the borders
            auto newTrans = player.obj->translation + glm::vec3(-playerVelocity.y, -playerVelocity.x, 0);
            auto th = (WALL_LENGHT-PLAYER_SIZE)*0.5f;
            newTrans.y = std::max(-th, std::min(th, newTrans.y)); 
            newTrans.x = std::max(-th, std::min(th, newTrans.x)); 
            player.obj->translation = newTrans;
        }
        // set the previous states
        if (frameCount > 1)
        {
            prevMat = extrinsicMatPlayer;
            prevIsPlayerMarkerDetected = isPlayerMarkerDetected;
        }

        // if ball not launched stick it to the player
        if (!isBallMoved)
        {
            ball.obj->translation = glm::vec3(player.obj->translation.x, player.obj->translation.y, -FLOOR_OFFSET - ball.obj->scale.x - 0.001f);
            // increase the press duration if the button is pressed
            if (!isButtonMarkerDetected && isMarkerDetected)
                buttonPressCount++;
        }
        if (buttonPressCount >= BUTTON_PRESS && !isBallMoved)
        {
            if (player.life <= 0)
            {
                score = 0;
                replay();
            }
            else
            {
                isBallMoved = true;
                button->texture = disableTex;
                auto rndm = help::rand(); // [0, 1]
                float sign1 = help::rand() > 0.5 ? -1 : 1;
                float sign2 = help::rand() > 0.5 ? -1 : 1;
                float v1 = rndm*BALL_SPEED * sign1;
                float v2 = (1-rndm)*BALL_SPEED * sign2;
                ball.velocity = glm::vec3(v1, v2, -BALL_SPEED);
                buttonPressCount = 0;
            }
        }

        if(isBallMoved)
            ball.velocity = glm::normalize(ball.velocity) * BALL_SPEED;
        ball.move(ball.velocity);

        // check the game state
        // if the ball dropped
        if (ball.obj->translation.z > 0.05)
        {
            player.life--;
            isBallMoved = false;

            // game over
            if (player.life <= 0)
            {
                isGameOver = true;
                won = false;
                button->texture = replayTex;
            }
            else
            {
                button->texture = launchTex;
            }
        }

        // collision
        collisionUpdate();

        // draw the scene
        draw();
    }

    void Game::collisionUpdate()
    {
        glm::vec3 newPoint, normal;

        // player-ball
        if (sphereAABBCollision(*ball.obj, *player.obj, ball.previousPos, newPoint, normal))
        {
            glm::vec3 reflected = glm::reflect(ball.velocity, normal);
            ball.setPos(newPoint); // move to stop the collision
            ball.velocity = reflected;
            return;
        }

        // bricks-ball
        for (int i = 0; i < bricks.size(); i++)
        {
            auto &brick = bricks[i];
            if (sphereAABBCollision(*ball.obj, *brick.obj, ball.previousPos, newPoint, normal))
            {
                glm::vec3 reflected = glm::reflect(ball.velocity, normal);
                ball.setPos(newPoint); // move to stop the collision
                ball.velocity = reflected;

                // brick health update
                brick.health--;
                brick.obj->shader->setFloat("u_alpha", 1.0 - 0.4 * brick.health);
                if (brick.health == 0)
                    bricks.erase(bricks.begin() + i);
                return;
            }
        }

        // walls-ball
        for (auto &wall : walls)
        {
            if (sphereAABBCollision(*ball.obj, *wall, ball.previousPos, newPoint, normal))
            {
                glm::vec3 reflected = glm::reflect(ball.velocity, normal);
                ball.setPos(newPoint); // move to stop the collision
                ball.velocity = reflected;
                return;
            }
        }
    }

    void Game::draw()
    {
        // view projection
        auto perspective = glm::perspective(glm::radians(30.f), ((float)WIDTH) / HEIGHT, 0.1f, 1000.f);
        auto vpScene = perspective * extrinsicMat;
        auto vpController = perspective * extrinsicMatPlayer;
        auto vpButton = perspective * extrinsicMatButton;
        glm::mat4 mvp;

        // background
        background->texture = bgTexture;
        background->draw();

        if (!isMarkerDetected)
            return;

        // player
        mvp = vpScene * player.obj->getTransform();
        player.obj->shader->setMat4("u_mvp", mvp);
        player.obj->draw();

        // ball
        mvp = vpScene * ball.obj->getTransform();
        ball.obj->shader->setMat4("u_mvp", mvp);
        ball.obj->draw();

        // bricks
        auto& bricks = this->bricks;
        auto drawBricks = [&bricks, &vpScene]()
        {
            for (auto &brick : bricks)
            {
                auto mvp = vpScene * brick.obj->getTransform();
                brick.obj->shader->setMat4("u_mvp", mvp);
                brick.obj->draw();
            }
        };

        // walls
        // sort the walls with respect to their distance to the camera
        auto inversed = glm::inverse(extrinsicMat);
        glm::vec3 camPos = glm::vec3(inversed[3][0], inversed[3][1], inversed[3][2]);
        std::sort(walls.begin(), walls.end(), [&camPos](std::shared_ptr<gfx::Cube> a, std::shared_ptr<gfx::Cube> b)
        {
            auto distanceA = glm::distance(camPos, a->translation);
            auto distanceB = glm::distance(camPos, b->translation);
            return distanceA > distanceB; 
        });

        // render the walls and the bricks
        auto size = walls.size();
        for (int i = 0; i < size; i++)
        {
            auto &wall = walls[i];
            
            if (i > size - 4)
                wall->shader->setFloat("u_alpha", 0.2f);
            else
            {
                if (i == size - 4) drawBricks(); 
                wall->shader->setFloat("u_alpha", 1.0f);
            }
               
            auto mvp = vpScene * wall->getTransform();
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

        // controller
        mvp = vpController * controller->getTransform();
        controller->shader->setMat4("u_mvp", mvp);
        controller->draw();

        // button
        if (isButtonMarkerDetected)
        {
            mvp = vpButton * controller->getTransform();
            button->shader->setMat4("u_mvp", mvp);
            button->draw();
        }

        // end screen
        if (isGameOver)
        {
            if (won)
                endScreen->texture = winTex;
            else
                endScreen->texture = gameOverTex;

            auto model = endScreen->getTransform();
            endScreen->shader->setMat4("u_model", model);
            endScreen->draw();
        }
    }

    void Game::replay()
    {
        isGameOver = false;
        // init bricks
        bricks.clear();
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
                    if (z == 0)
                        brick.obj->shader->setVec3("u_color", glm::vec3(1.0, 0.0, 0.0));
                    if (z == 1)
                        brick.obj->shader->setVec3("u_color", glm::vec3(0.0, 1.0, 0.0));
                    if (z == 2)
                        brick.obj->shader->setVec3("u_color", glm::vec3(0.1, 0.1, 1.0));
                    brick.obj->shader->setFloat("u_alpha", 0.2);

                    brick.obj->scale = glm::vec3(BRICK_SCALE);
                    brick.obj->translation = glm::vec3(BRICK_SCALE * x, BRICK_SCALE * y, -0.05 - BRICK_SCALE * z);

                    bricks.push_back(brick);
                }
            }
        }

        // init health bar
        healthBar.clear();
        player.life = PLAYER_HEALTH;
        for (int i = 0; i < player.life; i++)
        {
            auto life = std::make_shared<gfx::Cube>(
                std::make_shared<gfx::Texture>(path("texture/life.png")),
                std::make_shared<gfx::Shader>(path("shader/life.shader")));
            life->translation = glm::vec3(0.9 - i * 0.12, 0.8, -0.9999);
            life->scale = glm::vec3(0.2 * (HEIGHT / (float)WIDTH), 0.2, 0.000001);
            healthBar.push_back(life);
        }
    }
}
