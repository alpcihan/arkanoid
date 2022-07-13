#include "pose/pose.h"
#include "utils/utils.h"
#include "game/Game.h"
#include <opencv2/calib3d.hpp>

#define path(x) path::resource(x)

std::unique_ptr<pose::Video> video;
std::unique_ptr<arcanoid::Game> game;
pose::MarkerDetector markerDetector;

// debug
#ifdef DEBUG_MODE
pose::Window debugWindow;
int maxFocal = 1000;
#endif

glm::mat4 getMatrices(cv::Mat frame);

void focalLengthCallback(int val, void*)
{
    global::focal = val;
    std::cout << "Focal length: " << val << std::endl;
}

void onUpdate()
{
    // get the frame
    pose::Image frame;
    video->getFrame(&frame);

    // detect the markers
	const pose::Vec<pose::Marker> &markers = markerDetector.getMarkersFromImage(frame);
    for(const auto& marker : markers)
        if(marker.getCode() == 90)
            game->transformation = marker.getTransformation(); 

    #ifdef DEBUG_MODE
    pose::drawContourVec(markerDetector.subDivideCP.getLastResult(), &frame);
    pose::drawPoints(markerDetector.preciseEdgesCP.getLastResult(), &frame);
    debugWindow.display(markerDetector.adaptiveTHIP.getLastResult());
    #endif

    cv::waitKey(10);
    // set the game background image
    game -> bgTexture = help::getBGTexture(frame);
}

int main() 
{   
    #ifdef DEBUG_MODE
    debugWindow.createTrackbar("Focal len", &global::focal, maxFocal, &focalLengthCallback);
    #endif

    // init the capture
    video = std::make_unique<pose::Video>(0);

    // game
    game = std::make_unique<arcanoid::Game>();
    game -> init(&onUpdate);
    game -> run();

    return 0;
}