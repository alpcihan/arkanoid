# ar-arkanoid

<img width="680" alt="task3_task7_PNG" src="https://user-images.githubusercontent.com/37274614/179298434-02b56a55-1340-4f03-b65f-9db5c14db26b.png">

Marker-based AR implementation of the nostalgic [Arkanoid](https://en.wikipedia.org/wiki/Arkanoid) game with OpenGL, OpenCV and C++.

Video: https://youtu.be/__8tg8aaAow

## Table of Contents

* [Getting Started](#getting-started)
* [License](#license)

## Getting Started
### Download
- Clone the repository recursively `git clone --recursive https://github.com/alpcihan/ar-arcanoid`
- Use `git submodule update --init` if the repository was cloned without the submodules previously.
- The project automatically builds the third-party libraries except for OpenCV. 

  To set up the OpenCV:
  
  ```
  git clone https://github.com/opencv/opencv
  cd opencv
  mkdir build
  cd build
  cmake ..
  sudo make install
  ```
- The project should be ready to build.
### Markers
- The Application uses AR markers to estimate the pose of the game objects. Hence, three unique markers are required to play the game (game scene, character controller, action button).
- Default marker codes can be found/changed at [config.h](./src/config/config.h):
```
#define CODE_1 90 // Scene marker
#define CODE_2 7236 // Character controller marker
#define CODE_3 1680 // Action button marker
```
### Parameters
<img width="600" alt="task3_task7_PNG" src="https://user-images.githubusercontent.com/37274614/179316592-74a924f4-9d3f-4bf9-8878-24385471f77c.png">

- Accuracy of the marker detection and scene rendering may vary depending on various factors such as lighting, camera, marker color, resolution, etc.
- There are 4 parameters to control. `Block size`, `Constant` and `Max intensity` are the [Adaptive Threshold](https://docs.opencv.org/4.x/d7/d1b/group__imgproc__misc.html#ga72b913f352e4a1b1b397736707afcde3) parameters. 
  `Focal length` is the field of view value of the projection matrix to calibrate the camera. 
  
- To control the parameters define, ```#define PARAMETER_MODE``` at [config.h](./src/config/config.h).

## Contact

alpcihan.ac@gmail.com

## License

[MIT](./LICENSE)