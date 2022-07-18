#include "pose/pose.h"
#include "helpers/help.h"
#include "game/Game.h"
#include <opencv2/calib3d.hpp>
#include "config/config.h"

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

void getMatrices(cv::Mat frame, glm::mat4 &extrinsicMat, glm::mat4 &extrinsicMatPlayer, glm::mat4 &extrinsicMatButton, bool &isMarkerDetected, bool &isPlayerMarkerDetected, bool &isButtonMarkerDetected);

// callbacks
void focalLengthCallback(int val, void *)
{
	config::focal = val;
	std::cout << "Focal length: " << val << std::endl;
}
void intensityCallback(int val, void *)
{
	config::maxIntensity= val;
	std::cout << "Max intensity: " << val << std::endl;
}
void blockSizeCallback(int val, void *)
{
	config::blockSize = val;
	std::cout << "Block size: " << val << std::endl;
}
void athConstCallback(int val, void *)
{
	config::athConst = val;
	std::cout << "Const: " << val << std::endl;
}

void onUpdate()
{
	// get the frame
	pose::Image frame;
	video->getFrame(&frame);

	arkanoid->isMarkerDetected = false, arkanoid->isPlayerMarkerDetected = false, arkanoid->isButtonMarkerDetected = false;
	getMatrices(frame,
				arkanoid->extrinsicMat, arkanoid->extrinsicMatPlayer, arkanoid->extrinsicMatButton,
				arkanoid->isMarkerDetected, arkanoid->isPlayerMarkerDetected, arkanoid->isButtonMarkerDetected
	);

#ifdef PARAMETER_MODE
	parameterWindow.display(imgAdaptive);
#endif

	cv::waitKey(FPS_DROP);
	// set the game background image
	arkanoid->bgTexture = help::getBGTexture(frame);
}

int main()
{
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
		video = std::make_unique<pose::Video>(path("marker.mp4"));

	// game
	arkanoid = std::make_unique<game::Game>();
	arkanoid->init(&onUpdate);
	arkanoid->run();

	return 0;
}

#include <opencv2/opencv.hpp>
#include <iostream>
#include "pose/math/pose-estimation.h"
#include "pose/pose.h"
#include "game/Game.h"
using namespace cv;
using namespace std;
#define EX5 1
#define EX5_RAW 0
#define DRAW_CONTOUR 0
#define DRAW_RECTANGLE 0
#define THICKNESS_VALUE 4
struct MyStrip
{
	int stripeLength;
	int nStop;
	int nStart;
	Point2f stripeVecX;
	Point2f stripeVecY;
};
typedef vector<Point> contour_t;
typedef vector<contour_t> contour_vector_t;
int bw_thresh = 55;

int subpixSampleSafe(const Mat &pSrc, const Point2f &p)
{
	int fx = int(floorf(p.x));
	int fy = int(floorf(p.y));
	if (fx < 0 || fx >= pSrc.cols - 1 ||
		fy < 0 || fy >= pSrc.rows - 1)
		return 127;
	int px = int(256 * (p.x - floorf(p.x)));
	int py = int(256 * (p.y - floorf(p.y)));
	unsigned char *i = (unsigned char *)((pSrc.data + fy * pSrc.step) + fx);
	int a = i[0] + ((px * (i[1] - i[0])) >> 8);
	i += pSrc.step;
	int b = i[0] + ((px * (i[1] - i[0])) >> 8);
	return a + ((py * (b - a)) >> 8);
}
Mat calculate_Stripe(double dx, double dy, MyStrip &st)
{
	double diffLength = sqrt(dx * dx + dy * dy);
	st.stripeLength = (int)(0.8 * diffLength);
	if (st.stripeLength < 5)
		st.stripeLength = 5;
	st.stripeLength |= 1;
	st.nStop = st.stripeLength >> 1;
	st.nStart = -st.nStop;
	Size stripeSize;
	stripeSize.width = 3;
	stripeSize.height = st.stripeLength;
	st.stripeVecX.x = dx / diffLength;
	st.stripeVecX.y = dy / diffLength;
	st.stripeVecY.x = st.stripeVecX.y;
	st.stripeVecY.y = -st.stripeVecX.x;
	return Mat(stripeSize, CV_8UC1);
}

void getMatrices(Mat frame, glm::mat4 &extrinsicMat, glm::mat4 &extrinsicMatPlayer, glm::mat4 &extrinsicMatButton, bool &isMarkerDetected, bool &isPlayerMarkerDetected, bool &isButtonMarkerDetected)
{
	Mat imgFiltered;
	imgFiltered = frame.clone();
	cvtColor(imgFiltered, imgGrayScale, COLOR_BGR2GRAY);
	adaptiveThreshold(imgGrayScale, imgAdaptive, config::maxIntensity, ATH_TYPE, cv::THRESH_BINARY, config::blockSize, config::athConst);
	contour_vector_t contours;
	findContours(imgAdaptive, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
	for (size_t k = 0; k < contours.size(); k++)
	{
		contour_t approx_contour;
		approxPolyDP(contours[k], approx_contour, arcLength(contours[k], true) * 0.02, true);
		Scalar QUADRILATERAL_COLOR(0, 0, 255);
		Scalar colour;
		Rect r = boundingRect(approx_contour);
		if (approx_contour.size() == 4)
		{
			colour = QUADRILATERAL_COLOR;
		}
		else
		{
			continue;
		}
		if (r.height < 20 || r.width < 20 || r.width > imgFiltered.cols - 10 || r.height > imgFiltered.rows - 10)
		{
			continue;
		}
		polylines(imgFiltered, approx_contour, true, colour, THICKNESS_VALUE);
		float lineParams[16];
		Mat lineParamsMat(Size(4, 4), CV_32F, lineParams);
		for (size_t i = 0; i < approx_contour.size(); ++i)
		{
			double dx = ((double)approx_contour[(i + 1) % 4].x - (double)approx_contour[i].x) / 7.0;
			double dy = ((double)approx_contour[(i + 1) % 4].y - (double)approx_contour[i].y) / 7.0;
			MyStrip strip;
			Mat imagePixelStripe = calculate_Stripe(dx, dy, strip);
			Point2f edgePointCenters[6];
			for (int j = 1; j < 7; ++j)
			{
				double px = (double)approx_contour[i].x + (double)j * dx;
				double py = (double)approx_contour[i].y + (double)j * dy;

				Point p;
				p.x = (int)px;
				p.y = (int)py;
				for (int m = -1; m <= 1; ++m)
				{
					for (int n = strip.nStart; n <= strip.nStop; ++n)
					{
						Point2f subPixel;
						subPixel.x = (double)p.x + ((double)m * strip.stripeVecX.x) + ((double)n * strip.stripeVecY.x);
						subPixel.y = (double)p.y + ((double)m * strip.stripeVecX.y) + ((double)n * strip.stripeVecY.y);
						Point p2;
						p2.x = (int)subPixel.x;
						p2.y = (int)subPixel.y;
						int pixelIntensity = subpixSampleSafe(imgAdaptive, subPixel);
						int w = m + 1;
						int h = n + (strip.stripeLength >> 1);
						imagePixelStripe.at<uchar>(h, w) = (uchar)pixelIntensity;
					}
				}
				vector<double> sobelValues(strip.stripeLength - 2.);
				for (int n = 1; n < (strip.stripeLength - 1); n++)
				{
					unsigned char *stripePtr = &(imagePixelStripe.at<uchar>(n - 1, 0));
					double r1 = -stripePtr[0] - 2. * stripePtr[1] - stripePtr[2];
					stripePtr += 2 * imagePixelStripe.step;
					double r3 = stripePtr[0] + 2. * stripePtr[1] + stripePtr[2];
					unsigned int ti = n - 1;
					sobelValues[ti] = r1 + r3;
				}

				double maxIntensity = -1;
				int maxIntensityIndex = 0;
				for (int n = 0; n < strip.stripeLength - 2; ++n)
				{
					if (sobelValues[n] > maxIntensity)
					{
						maxIntensity = sobelValues[n];
						maxIntensityIndex = n;
					}
				}
				double y0, y1, y2;
				unsigned int max1 = maxIntensityIndex - 1, max2 = maxIntensityIndex + 1;
				y0 = (maxIntensityIndex <= 0) ? 0 : sobelValues[max1];
				y1 = sobelValues[maxIntensityIndex];
				y2 = (maxIntensityIndex >= strip.stripeLength - 3) ? 0 : sobelValues[max2];
				double pos = (y2 - y0) / (4 * y1 - 2 * y0 - 2 * y2);
				if (isnan(pos))
				{
					continue;
				}
				Point2d edgeCenter;
				int maxIndexShift = maxIntensityIndex - (strip.stripeLength >> 1);
				edgeCenter.x = (double)p.x + (((double)maxIndexShift + pos) * strip.stripeVecY.x);
				edgeCenter.y = (double)p.y + (((double)maxIndexShift + pos) * strip.stripeVecY.y);
				edgePointCenters[j - 1].x = edgeCenter.x;
				edgePointCenters[j - 1].y = edgeCenter.y;
			}
			Mat highIntensityPoints(Size(1, 6), CV_32FC2, edgePointCenters);
			fitLine(highIntensityPoints, lineParamsMat.col(i), CV_DIST_L2, 0, 0.01, 0.01);
			Point p1;
			p1.x = (int)lineParams[8 + i] - (int)(50.0 * lineParams[i]);
			p1.y = (int)lineParams[12 + i] - (int)(50.0 * lineParams[4 + i]);
			Point p2;
			p2.x = (int)lineParams[8 + i] + (int)(50.0 * lineParams[i]);
			p2.y = (int)lineParams[12 + i] + (int)(50.0 * lineParams[4 + i]);
		}
		Point2f corners[4];
		for (int i = 0; i < 4; ++i)
		{
			int j = (i + 1) % 4;
			double x0, x1, y0, y1, u0, u1, v0, v1;
			x0 = lineParams[i + 8];
			y0 = lineParams[i + 12];
			x1 = lineParams[j + 8];
			y1 = lineParams[j + 12];
			u0 = lineParams[i];
			v0 = lineParams[i + 4];
			u1 = lineParams[j];
			v1 = lineParams[j + 4];
			double a = x1 * u0 * v1 - y1 * u0 * u1 - x0 * u1 * v0 + y0 * u0 * u1;
			double b = -x0 * v0 * v1 + y0 * u0 * v1 + x1 * v0 * v1 - y1 * v0 * u1;
			double c = v1 * u0 - v0 * u1;
			if (fabs(c) < 0.001)
			{
				continue;
			}
			a /= c;
			b /= c;
			corners[i].x = a;
			corners[i].y = b;
		}
		Point2f targetCorners[4];
		targetCorners[0].x = -0.5;
		targetCorners[0].y = -0.5;
		targetCorners[1].x = 5.5;
		targetCorners[1].y = -0.5;
		targetCorners[2].x = 5.5;
		targetCorners[2].y = 5.5;
		targetCorners[3].x = -0.5;
		targetCorners[3].y = 5.5;
		Mat homographyMatrix(Size(3, 3), CV_32FC1);
		homographyMatrix = getPerspectiveTransform(corners, targetCorners);
		Mat imageMarker(Size(6, 6), CV_8UC1);
		warpPerspective(imgAdaptive, imageMarker, homographyMatrix, Size(6, 6));
		threshold(imageMarker, imageMarker, bw_thresh, 255, CV_THRESH_BINARY);
		int code = 0;
		for (int i = 0; i < 6; ++i)
		{
			// Check if border is black
			int pixel1 = imageMarker.at<uchar>(0, i); // top
			int pixel2 = imageMarker.at<uchar>(5, i); // bottom
			int pixel3 = imageMarker.at<uchar>(i, 0); // left
			int pixel4 = imageMarker.at<uchar>(i, 5); // right
			if ((pixel1 > 0) || (pixel2 > 0) || (pixel3 > 0) || (pixel4 > 0))
			{
				code = -1;
				break;
			}
		}
		if (code < 0)
		{
			continue;
		}
		int cP[4][4];
		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				cP[i][j] = imageMarker.at<uchar>(i + 1, j + 1);
				cP[i][j] = (cP[i][j] == 0) ? 1 : 0;
			}
		}
		int codes[4];
		codes[0] = codes[1] = codes[2] = codes[3] = 0;
		for (int i = 0; i < 16; i++)
		{
			int row = i >> 2;
			int col = i % 4;
			codes[0] <<= 1;
			codes[0] |= cP[row][col]; // 0�
			codes[1] <<= 1;
			codes[1] |= cP[3 - col][row]; // 90�
			codes[2] <<= 1;
			codes[2] |= cP[3 - row][3 - col]; // 180�
			codes[3] <<= 1;
			codes[3] |= cP[col][3 - row]; // 270�
		}
		if ((codes[0] == 0) || (codes[0] == 0xffff))
		{
			continue;
		}
		int angle = 0;
		code = codes[0];
		for (int i = 1; i < 4; ++i)
		{
			if (codes[i] < code)
			{
				code = codes[i];
				angle = i;
			}
		}
		if (angle != 0)
		{
			Point2f corrected_corners[4];
			for (int i = 0; i < 4; i++)
				corrected_corners[(i + angle) % 4] = corners[i];
			for (int i = 0; i < 4; i++)
				corners[i] = corrected_corners[i];
		}
		for (int i = 0; i < 4; i++)
		{
			corners[i].x -= 320;
			corners[i].y = -corners[i].y + 180;
		}
		float resultMatrix[16];
		pose::estimateSquarePose(resultMatrix, (Point2f *)corners, MARKER_SIZE * 2);

		if (code == CODE_1)
		{
			float transposed[16];

			for (int x = 0; x < 4; ++x)
				for (int y = 0; y < 4; ++y)
					transposed[x * 4 + y] = resultMatrix[y * 4 + x];

			memcpy(glm::value_ptr(extrinsicMat), transposed, sizeof(transposed));
			isMarkerDetected = true;
		}
		else if (code == CODE_2)
		{
			float transposed[16];

			for (int x = 0; x < 4; ++x)
				for (int y = 0; y < 4; ++y)
					transposed[x * 4 + y] = resultMatrix[y * 4 + x];

			memcpy(glm::value_ptr(extrinsicMatPlayer), transposed, sizeof(transposed));
			isPlayerMarkerDetected = true;
		}
		else if (code == CODE_3)
		{
			float transposed[16];

			for (int x = 0; x < 4; ++x)
				for (int y = 0; y < 4; ++y)
					transposed[x * 4 + y] = resultMatrix[y * 4 + x];

			memcpy(glm::value_ptr(extrinsicMatButton), transposed, sizeof(transposed));
			isButtonMarkerDetected = true;
		}
	}
}