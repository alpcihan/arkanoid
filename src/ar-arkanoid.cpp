#include <opencv2/calib3d.hpp>
#include "helpers/help.h"
#include "arkanoid.h"

#define path(x) help::toResourcePath(x)

std::unique_ptr<pose::Video> video;
std::unique_ptr<game::Game> arkanoid;
pose::MarkerDetector markerDetector;

cv::Mat imgGrayScale, imgAdaptive;
int maxFocal = 1000;

// debug
#ifdef PARAMETER_MODE
pose::Window parameterWindow;
#endif

// callbacks
void focalLengthCallback(int val, void *) {
    config::focal = val;
}
void intensityCallback(int val, void *) {
    config::maxIntensity = val;
}
void blockSizeCallback(int val, void *) {
    config::blockSize = val;
}
void athConstCallback(int val, void *) {
    config::athConst = val;
}

void onUpdate() {
    pose::Image frame;
    video->getFrame(&frame);

    arkanoid->isMarkerDetected = false, arkanoid->isPlayerMarkerDetected = false, arkanoid->isButtonMarkerDetected = false;
    pose::getMatrices(frame,
                arkanoid->extrinsicMat, arkanoid->extrinsicMatPlayer, arkanoid->extrinsicMatButton,
                arkanoid->isMarkerDetected, arkanoid->isPlayerMarkerDetected, arkanoid->isButtonMarkerDetected);

#ifdef PARAMETER_MODE
    parameterWindow.display(imgAdaptive);
#endif

    cv::waitKey(FPS_DROP);
    arkanoid->bgTexture = help::getBGTexture(frame);
}

int main() {
#ifdef PARAMETER_MODE
    parameterWindow.createTrackbar("Focal len", &config::focal, maxFocal, &focalLengthCallback);
    parameterWindow.createTrackbar("Max intensity", &config::maxIntensity, 255, &intensityCallback);
    parameterWindow.createTrackbar("Block size", &config::blockSize, 200, &blockSizeCallback);
    parameterWindow.createTrackbar("Const", &config::athConst, 70, &athConstCallback);
#endif

    // init the capture
    if (USE_WEBCAM)
        video = std::make_unique<pose::Video>(0);
    else
        video = std::make_unique<pose::Video>(path(INPUT_VIDEO));

    // game
    arkanoid = std::make_unique<game::Game>();
    arkanoid->init(&onUpdate);
    arkanoid->run();

    return 0;
}