#include <glm/glm.hpp>

namespace config
{
   extern int focal;
   extern float sceneSize;
}

#pragma once

// window
#define WIDTH 1920
#define HEIGHT 1080
#define USE_WEBCAM 1
#define FPS_DROP 10

// marker
#define MARKER_SIZE 0.04846
#define CODE_1 90
#define CODE_2 7236
#define CODE_3 1680

// debug
#define DEBUG_MODE

// game
#define PLAYER_SIZE 0.05f
#define PLAYER_HEALTH 3

#define BALL_SPEED 0.005f

#define BRICK_HEALTH 2

#define BUTTON_PRESS 60

#define FLOOR_OFFSET 0.008

#define WALL_HEIGHT 0.15
#define WALL_LENGHT 0.15
#define WALL_WIDTH 0.01
#define BALL_SCALE 0.005

#define BRICK_SCALE 0.01 

// threshold
#define MAX_INTENSITY 255
#define BLOCK_SIZE 179 
// 50   
#define ATH_CONST 10          
#define ATH_TYPE 1   

#include <glm/glm.hpp>

namespace config
{
   extern int focal;
   extern float sceneSize;
   extern int maxIntensity;
   extern int blockSize;
   extern int athConst;
}