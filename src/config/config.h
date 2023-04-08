#pragma once

/* window settings */
#define WIDTH 1920 // width of the application window
#define HEIGHT 1080 // height of the application window
#define USE_WEBCAM 1 // 1: use webcam | 0: use resourcers/marker.mp4 (pre-recorded game scene video)
#define FPS_DROP 10 // delay per frame in ms

/* marker settings */
#define MARKER_SIZE 0.04846 // scale of the markers
#define CODE_1 90 // Scene marker
#define CODE_2 7236 // Character controller marker
#define CODE_3 1680 // Action button marker

/* camera calibration + adaptive threshold parameters */
/* detail about the adaptive threshold parameters: https://docs.opencv.org/4.x/d7/d1b/group__imgproc__misc.html#ga72b913f352e4a1b1b397736707afcde3 */
// #define PARAMETER_MODE // enable camera calibration and threshold setup window
#define MAX_INTENSITY 255 
#define ATH_CONST 10          
#define ATH_TYPE 1   

/* game settings */
#define PLAYER_SIZE 0.051f // scale of the player platform
#define PLAYER_HEALTH 3 // player health
#define PLAYER_SPEED_CONST 2.0f // player movement speed
#define BALL_SPEED 0.0035f // ball speed
#define BUTTON_PRESS 60 // min button hold duration in frames (it is fps dependent)
#define FLOOR_OFFSET 0.008f // floor y offset
#define WALL_HEIGHT 0.15f // height of the walls
#define WALL_LENGTH 0.15f // length of the walls
#define WALL_WIDTH 0.01f // width of the walls
#define BALL_SCALE 0.005f // scale of the ball
#define BRICK_SCALE 0.0115f // brick scale
#define BRICK_HEALTH 2 // brick health (required hit count to destroy)

#include <glm/glm.hpp>
namespace config
{
   extern int focal;
   extern float sceneSize;
   extern int maxIntensity;
   extern int blockSize;
   extern int athConst;
}