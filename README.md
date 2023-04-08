# ar-arkanoid

<img width="680" alt="task3_task7_PNG" src="https://user-images.githubusercontent.com/37274614/179810967-151b7dd1-6f5a-4313-be69-b0b84df71504.png">

Marker-based AR implementation of the nostalgic [Arkanoid](https://en.wikipedia.org/wiki/Arkanoid) game with OpenGL, OpenCV and C++.

Video: https://youtu.be/FkXOo16MfIE

## Getting Started
### Installation & Setup
1) Clone the repository recursively `git clone --recursive https://github.com/alpcihan/ar-arcanoid`

2) Use `git submodule update --init` if the repository was cloned without the submodules previously.

3) The project automatically builds third-party libraries except for OpenCV. 

4) Install OpenCV:
  
    ```
    git clone https://github.com/opencv/opencv
    cd opencv
    mkdir build
    cd build
    cmake ..
    sudo make install
    ```

### Parameters
See [src/config/config.h](./src/config/config.h) for details about the parameters related to the game settings, camera calibration and marker detection.

### Markers
The Application uses ArUco markers to estimate the pose of the game objects. Three unique markers are required to play the game (game scene, character controller, and action button).

Default marker codes can be found/changed at [src/config/config.h](./src/config/config.h):

  ```
  ...

  #define CODE_1 90 // Scene marker
  #define CODE_2 7236 // Character controller marker
  #define CODE_3 1680 // Action button marker

  ...
  ```
### Parameters
<img width="600" alt="task3_task7_PNG" src="https://user-images.githubusercontent.com/37274614/179316592-74a924f4-9d3f-4bf9-8878-24385471f77c.png">

- Accuracy of the marker detection and scene rendering may vary depending on various factors such as lighting, camera, marker color, resolution, etc.
- There are 4 parameters to control. `Block size`, `Constant` and `Max intensity` are the [Adaptive Threshold](https://docs.opencv.org/4.x/d7/d1b/group__imgproc__misc.html#ga72b913f352e4a1b1b397736707afcde3) parameters. 
  `Focal length` is the field of view value of the projection matrix to calibrate the camera. 
  
- To control the parameters define, ```#define PARAMETER_MODE``` at [config.h](./src/config/config.h).

## License

[MIT](./LICENSE)
