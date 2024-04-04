#define _USE_MATH_DEFINES

#include "pose-estimation.h"
#include <math.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include "config/config.h"
#include "helpers/help.h"

using namespace std;

namespace {
//! @brief Use only this constant for accessing a quaternion's X component
const unsigned char QX = 0;

//! @brief Use only this constant for accessing a quaternion's Y component
const unsigned char QY = 1;

//! @brief Use only this constant for accessing a quaternion's Z component
const unsigned char QZ = 2;

//! @brief Use only this constant for accessing a quaternion's scalar component
const unsigned char QW = 3;
}  // namespace

namespace pose {
/**
 * normalizes a quaternion (makes it a unit quaternion)
 */
float *normalizeQuaternion(float *q) {
    float norm = 0.0f;
    for (int i = 0; i < 4; i++)
        norm += q[i] * q[i];
    norm = sqrtf(1.0f / norm);

    for (int i = 0; i < 4; i++)
        q[i] *= norm;

    return q;
}

/**
 * implementation based on proposal by Horn, idea:
 * first determine the largest entry of unit quaternion,
 * then use this to get other entries
 * adapted from dwarfutil.cpp
 */
float *matrixToQuaternion(const CvMat *pMat, float *q) {
    // shortcuts to the matrix data
    const float *m0 = (float *)(pMat->data.ptr);
    const float *m1 = (float *)(pMat->data.ptr + pMat->step);
    const float *m2 = (float *)(pMat->data.ptr + 2 * pMat->step);

    // get entry of q with largest absolute value
    // note: we compute here 4 * q[..]^2 - 1

    float tmp[4];

    tmp[QW] = m0[0] + m1[1] + m2[2];
    tmp[QX] = m0[0] - m1[1] - m2[2];
    tmp[QY] = -m0[0] + m1[1] - m2[2];
    tmp[QZ] = -m0[0] - m1[1] + m2[2];

    int max = QW;

    if (tmp[QX] > tmp[max])
        max = QX;

    if (tmp[QY] > tmp[max])
        max = QY;

    if (tmp[QZ] > tmp[max])
        max = QZ;

    // depending on largest entry compute the other values
    // note: these formulae can be derived very simply from the
    //       matrix representation computed in quaternionToMatrix

    switch (max) {
        case QW:
            q[QW] = sqrtf(tmp[QW] + 1) * 0.5f;
            q[QX] = (m2[1] - m1[2]) / (4 * q[QW]);
            q[QY] = (m0[2] - m2[0]) / (4 * q[QW]);
            q[QZ] = (m1[0] - m0[1]) / (4 * q[QW]);
            break;

        case QX:
            q[QX] = sqrtf(tmp[QX] + 1) * 0.5f;
            q[QW] = (m2[1] - m1[2]) / (4 * q[QX]);
            q[QY] = (m1[0] + m0[1]) / (4 * q[QX]);
            q[QZ] = (m0[2] + m2[0]) / (4 * q[QX]);
            break;

        case QY:
            q[QY] = sqrtf(tmp[QY] + 1) * 0.5f;
            q[QW] = (m0[2] - m2[0]) / (4 * q[QY]);
            q[QX] = (m1[0] + m0[1]) / (4 * q[QY]);
            q[QZ] = (m2[1] + m1[2]) / (4 * q[QY]);
            break;

        case QZ:
            q[QZ] = sqrtf(tmp[QZ] + 1) * 0.5f;
            q[QW] = (m1[0] - m0[1]) / (4 * q[QZ]);
            q[QX] = (m0[2] + m2[0]) / (4 * q[QZ]);
            q[QY] = (m2[1] + m1[2]) / (4 * q[QZ]);
            break;
    }

    normalizeQuaternion(q);
    return q;
}

/**
 * rotate a vector around a quaternion
 * implementation based on precomputation, correct mult
 * matrix can be found in Horn, Closed-form solution of
 * absolute orientation using unit quaternions. (1987)
 * adapted from dwarfutil.cpp
 */
float *rotateQuaternion(float *r, const float *q, const float *p) {
    // precomputation of some values
    float xy = q[QX] * q[QY];
    float xz = q[QX] * q[QZ];
    float yz = q[QY] * q[QZ];
    float ww = q[QW] * q[QW];
    float wx = q[QW] * q[QX];
    float wy = q[QW] * q[QY];
    float wz = q[QW] * q[QZ];

    r[0] = p[0] * (2 * (q[QX] * q[QX] + ww) - 1) + p[1] * 2 * (xy - wz) + p[2] * 2 * (wy + xz);
    r[1] = p[0] * 2 * (xy + wz) + p[1] * (2 * (q[QY] * q[QY] + ww) - 1) + p[2] * 2 * (yz - wx);
    r[2] = p[0] * 2 * (xz - wy) + p[1] * 2 * (wx + yz) + p[2] * (2 * (q[QZ] * q[QZ] + ww) - 1);

    return r;
}

/**
 * computes the orientation and translation of a square using homography
 * @param pRot result as quaternion
 * @param pTrans result position
 * @param p2D four input coordinates. the origin is assumed to be at the camera's center of projection
 * @param fMarkerSize side-length of marker. Origin is at marker center.
 * @param f focal length
 */
void getInitialPose(float *pRot, float *pTrans, const CvPoint2D32f *p2D, float fMarkerSize, float f) {
    // compute homography
    float hom[3][3];
    calcHomography(hom[0], p2D);

    // compute rotation matrix by multiplying with inverse of camera matrix and inverse marker scaling:
    // R = C^-1 H S^-1

    float fRotMat[3][3];
    CvMat rotMat = cvMat(3, 3, CV_32F, fRotMat[0]);
    const float fScaleLeft[3] = {1.0f / f, 1.0f / f, -1.0f};
    const float fScaleRight[3] = {1.0f / fMarkerSize, 1.0f / fMarkerSize, 1.0f};

    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            fRotMat[r][c] = hom[r][c] * fScaleLeft[r] * fScaleRight[c];

    // check sign of z-axis translation, multiply matrix with -1 if necessary
    if (fRotMat[2][2] > 0.0f)
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                fRotMat[r][c] *= -1;

    // get shortcuts for columns
    CvMat ColX;
    CvMat ColY;
    CvMat ColZ;

    cvGetCol(&rotMat, &ColX, 0);
    cvGetCol(&rotMat, &ColY, 1);
    cvGetCol(&rotMat, &ColZ, 2);

    // compute length of the first two colums
    float fXLen = static_cast<float>(cvNorm(&ColX));
    float fYLen = static_cast<float>(cvNorm(&ColY));

    // copy & normalize translation
    float fTransScale = 2.0f / (fXLen + fYLen);

    for (int i = 0; i < 3; i++)
        pTrans[i] = fRotMat[i][2] * fTransScale;

    // normalize first two colums
    cvScale(&ColX, &ColX, 1.0f / fXLen);
    cvScale(&ColY, &ColY, 1.0f / fYLen);

    // compute error = cosine of angle between colums
    // float fDotProd = cvDotProduct( &ColX, &ColY );
    // compute third row as vector product
    cvCrossProduct(&ColX, &ColY, &ColZ);

    // normalize cross product
    float fZLen = static_cast<float>(cvNorm(&ColZ));
    cvScale(&ColZ, &ColZ, 1.0f / fZLen);

    // recompute y vector from x and z
    cvCrossProduct(&ColX, &ColZ, &ColY);
    cvScale(&ColY, &ColY, -1.0);

    // compute rotation quaternion from matrix
    matrixToQuaternion(&rotMat, pRot);
}

/**
 * computes the orientation and translation of a square using homography
 * @param pResult result as a 4x4 matrix
 * @param pHomography pointer to homography matrix
 * @param markerSize side-length of marker. Origin is at marker center.
 * @param focalLength focal length
 */
void poseFromHomography(float *pResult, float *pHomography, float markerSize, float focalLength) {
    // convert to float because of better precision
    float fMarkerSize = markerSize;
    float f = focalLength;

    // scale first two rows with 1/f
    // scale first two columns with 1/fMarkerSize

    float m[3][3];

    for (int r = 0; r < 2; r++)
        for (int c = 0; c < 2; c++)
            m[r][c] = pHomography[3 * r + c] / (f * fMarkerSize);

    m[0][2] = pHomography[2] / f;
    m[1][2] = pHomography[5] / f;
    m[2][0] = pHomography[6] / fMarkerSize;
    m[2][1] = pHomography[7] / fMarkerSize;
    m[2][2] = pHomography[8];

    // compute length of the first two colums
    float fXLen = 0.0f;
    float fYLen = 0.0f;

    for (int i = 0; i < 3; i++) {
        fXLen += m[i][0] * m[i][0];
        fYLen += m[i][1] * m[i][1];
    }

    fXLen = sqrtf(fXLen);
    fYLen = sqrtf(fYLen);

    // copy & normalize translation
    float fTransScale = 2.0f / (fXLen + fYLen);

    for (int i = 0; i < 3; i++)
        pResult[4 * i + 3] = m[i][2] * fTransScale;

    // normalize first two colums
    for (int i = 0; i < 3; i++) {
        m[i][0] /= fXLen;
        m[i][1] /= fYLen;
    }

    // compute error = cosine of angle between colums
    // float fDotProd = cvDotProduct( &ColX, &ColY );
    // todo: port from here on
    /*

    // compute third row as vector product
    cvCrossProduct( &ColX, &ColY, &ColZ );

    // normalize cross product
    float fZLen = static_cast<float>( cvNorm( &ColZ ) );
    cvScale( &ColZ, &ColZ, 1.0f / fZLen );

    // recompute y vector from x and z
    cvCrossProduct( &ColX, &ColZ, &ColY );
    cvScale( &ColY, &ColY, -1.0 );

    // compute rotation quaternion from matrix
    matrixToQuaternion( pRotMat, pRot );

    // free up stuff
    cvReleaseMat( &pRotMat );
    cvReleaseMat( &pHomography ); */
}

/**
 * computes where a point should be on the screen, given a pose
 * @param p2D output 2d vector
 * @param p3d input 3D vector
 * @param pRotation rotation as quaternione
 * @param pTranslation 3-element translation
 * @param f focal length
 */
void projectPoint(CvPoint2D32f &p2D, CvPoint3D32f p3D, const float *pRotation, const float *pTranslation, float f) {
    float point[3];
    float point3D[3];

    point3D[0] = p3D.x;
    point3D[1] = p3D.y;
    point3D[2] = p3D.z;

    // rotate
    rotateQuaternion(point, pRotation, point3D);

    // translate

    for (int i = 0; i < 3; i++)
        point[i] += pTranslation[i];

    // project
    // TODO: check for division by zero
    p2D.x = f * point[0] / -point[2];
    p2D.y = f * point[1] / -point[2];
}

/**
 * factors a non-unit-length of a quaternion into the translation
 */
void normalizePose(float *pRot, float *pTrans) {
    // compute length of quaternion
    float fQuatLenSq = 0.0f;

    for (int i = 0; i < 4; i++)
        fQuatLenSq += pRot[i] * pRot[i];

    float fQuatLen = sqrtf(fQuatLenSq);

    // normalize quaternion
    for (int i = 0; i < 4; i++)
        pRot[i] /= fQuatLen;

    // scale translation
    for (int i = 0; i < 3; i++)
        pTrans[i] /= fQuatLenSq;
}

/**
 * computes the Jacobian for optimizing the pose
 * @param pResult 2x7 matrix
 * @param pParam rotation parameters: 4 * quaternion rotation + 3 * translation
 * @param p3D the 3d input vector
 * @param f focal length
 */
void computeJacobian(float *pResult, float *pParam, const CvPoint3D32f &p3D, float f) {
    // TODO: check for division by zero
    // maple-generated code
    float t4 = pParam[0] * p3D.x + pParam[1] * p3D.y + pParam[2] * p3D.z;
    float t10 = pParam[3] * p3D.x + pParam[1] * p3D.z - pParam[2] * p3D.y;
    float t15 = pParam[3] * p3D.y - pParam[0] * p3D.z + pParam[2] * p3D.x;
    float t20 = pParam[3] * p3D.z + pParam[0] * p3D.y - pParam[1] * p3D.x;
    float t22 = -t4 * pParam[2] + t10 * pParam[1] - t15 * pParam[0] - t20 * pParam[3] - pParam[6];
    float t23 = 1.0f / t22;
    float t24 = 2.0f * f * t4 * t23;
    float t30 = f * (t4 * pParam[0] + t10 * pParam[3] - t15 * pParam[2] + t20 * pParam[1] + pParam[4]);
    float t31 = t22 * t22;
    float t32 = 1.0f / t31;
    float t33 = -2.0f * t32 * t15;
    float t38 = 2.0f * t32 * t10;
    float t43 = -2.0f * t32 * t4;
    float t47 = 2.0f * f * t10 * t23;
    float t48 = -2.0f * t32 * t20;
    float t51 = f * t23;
    float t60 = f * (t4 * pParam[1] + t10 * pParam[2] + t15 * pParam[3] - t20 * pParam[0] + pParam[5]);

    pResult[0] = t24 - t30 * t33;
    pResult[1] = 2.0f * f * t20 * t23 - t30 * t38;
    pResult[2] = -2.0f * f * t15 * t23 - t30 * t43;
    pResult[3] = t47 - t30 * t48;
    pResult[4] = t51;
    pResult[5] = 0.0f;
    pResult[6] = t30 * t32;
    pResult[7 + 0] = -2.0f * f * t20 * t23 - t60 * t33;
    pResult[7 + 1] = t24 - t60 * t38;
    pResult[7 + 2] = t47 - t60 * t43;
    pResult[7 + 3] = 2.0f * f * t15 * t23 - t60 * t48;
    pResult[7 + 4] = 0.0f;
    pResult[7 + 5] = t51;
    pResult[7 + 6] = t60 * t32;
}

/**
 * computes the reprojection error for each point
 * @param pError output: two entries (x,y) for each input point = (measured - reprojected)
 * @param p3D 3D coordinates of the points
 * @param p2D measured 2D coordinates
 * @param nPoints number of input points
 * @param pRot rotation
 * @param pTrans translation
 * @param f focal length
 * @returns absolute squared error
 */
float computeReprojectionError(float *pError, const CvPoint3D32f *p3D, const CvPoint2D32f *p2D, int nPoints,
                               const float *pRot, const float *pTrans, float f) {
    float fAbsErrSq = 0.0f;

    for (int i = 0; i < nPoints; i++) {
        // reproject
        CvPoint2D32f projected;
        projectPoint(projected, p3D[i], pRot, pTrans, f);

        // compute deviation
        pError[2 * i] = p2D[i].x - projected.x;
        pError[2 * i + 1] = p2D[i].y - projected.y;

        // update absolute error
        fAbsErrSq += pError[2 * i] * pError[2 * i] + pError[2 * i + 1] * pError[2 * i + 1];
    }

    return fAbsErrSq;
}

/**
 * optimize a pose with levenberg-marquardt
 * @param pRotation rotation as quaternion, both used as output and initial value
 * @param pTranslation 3-element translation, both used as output and initial value
 * @param nPoints number of correspondences
 * @param p2D pointer to camera coordinates
 * @param p3D pointer to object coordinates
 * @param f focal length
 */
void optimizePose(float *pRotation, float *pTranslation, int nPoints, const CvPoint2D32f *p2D, const CvPoint3D32f *p3D, float f) {
    using std::vector;

    float params[7];

    // copy rot & trans to vector
    for (int i = 0; i < 4; i++)
        params[i] = pRotation[i];

    for (int i = 0; i < 3; i++)
        params[i + 4] = pTranslation[i];

    float paramDiff[7];

    vector<CvPoint2D32f> estMeasurements(nPoints);
    vector<float> jacobian(nPoints * 2 * 7);
    vector<float> measurementDiffPrev(nPoints * 2);
    vector<float> measurementDiffNew(nPoints * 2);

    float MDiff2[7];
    float jacobiSquare[7 * 7];
    float fLambda = 1.0f;  // levenberg-marquardt-lambda

    CvMat matJacobian;
    cvInitMatHeader(&matJacobian, nPoints * 2, 7, CV_32F, &jacobian[0]);

    CvMat matJacobiSquare;
    cvInitMatHeader(&matJacobiSquare, 7, 7, CV_32F, &jacobiSquare[0]);

    CvMat matMeasurementDiffPrev;
    cvInitMatHeader(&matMeasurementDiffPrev, nPoints * 2, 1, CV_32F, &measurementDiffPrev[0]);

    CvMat matMeasurementDiffNew;
    cvInitMatHeader(&matMeasurementDiffNew, nPoints * 2, 1, CV_32F, &measurementDiffNew[0]);

    CvMat matParamDiff;
    cvInitMatHeader(&matParamDiff, 7, 1, CV_32F, &paramDiff[0]);

    CvMat matMDiff2;
    cvInitMatHeader(&matMDiff2, 7, 1, CV_32F, &MDiff2[0]);

    // compute initial error
    float fPreviousErr = computeReprojectionError(&measurementDiffPrev[0], p3D, p2D, nPoints, params, params + 4, f);

#ifdef PRINT_OPTIMIZATION

    // debugging

    std::cout << "initial error: " << fPreviousErr;

#endif

    // iterate (levenberg-marquardt)
    const int nMaxIterations = 3;

    for (int iIteration = 0; iIteration < nMaxIterations; iIteration++) {
        // create jacobian
        for (int i = 0; i < nPoints; i++)
            computeJacobian(&jacobian[i * 2 * 7], params, p3D[i], f);

        // multiply both sides with J^T
        cvMulTransposed(&matJacobian, &matJacobiSquare, 1);
        cvGEMM(&matJacobian, &matMeasurementDiffPrev, 1.0, NULL, 1.0, &matMDiff2, CV_GEMM_A_T);

        // add lambda to diagonal
        for (int i = 0; i < 7; i++)
            jacobiSquare[i * 7 + i] += fLambda;

        // do least squares
        // performace improvement: use cholesky decomp
        cvSolve(&matJacobiSquare, &matMDiff2, &matParamDiff, CV_LU);

        // update parameters
        float paramsNew[7];

        for (int i = 0; i < 7; i++)
            paramsNew[i] = params[i] + paramDiff[i];

        // factor the quaternion length into the translation
        normalizePose(paramsNew, paramsNew + 4);

        // compute new error
        float fErr = computeReprojectionError(&measurementDiffNew[0], p3D, p2D, nPoints, paramsNew, paramsNew + 4, f);

        if (fErr >= fPreviousErr)
            fLambda *= 10.0f;

        else {
            fLambda /= 10.0f;

            // update parameters
            for (int i = 0; i < 7; i++)
                params[i] = paramsNew[i];

            // copy measurement error
            for (int i = 0; i < nPoints * 2; i++)
                measurementDiffPrev[i] = measurementDiffNew[i];

            fPreviousErr = fErr;
        }

#ifdef PRINT_OPTIMIZATION
        // more debugging
        std::cout << ", it" << iIteration << ": fErr=" << fErr << " lambda=" << fLambda;
#endif
    }

#ifdef PRINT_OPTIMIZATION
    std::cout << std::endl;
#endif

    // copy back rot & trans fromvector
    for (int i = 0; i < 4; i++)
        pRotation[i] = params[i];

    for (int i = 0; i < 3; i++)
        pTranslation[i] = params[i + 4];
}

void estimateSquarePose(float *result, const cv::Point2f *p2D_, float markerSize) {
    CvPoint2D32f *p2D = new CvPoint2D32f[4];

    for (size_t i = 0; i < 4; i++) {
        p2D[i].x = p2D_[i].x;
        p2D[i].y = p2D_[i].y;
    }

    estimateSquarePose_(result, p2D, markerSize);
    delete p2D;
};

/**
 * @param mat result as 4x4 matrix in row-major format
 * @param p2D coordinates of the four corners in counter-clock-wise order.
 *        the origin is assumed to be at the camera's center of projection
 * @param markerSize side-length of marker. Origin is at marker center.
 */
void estimateSquarePose_(float *mat, CvPoint2D32f *p2D, float markerSize) {
    // approximate focal length for logitech quickcam 4000 at 320*240 resolution

    // approx for laptop internal camera with resolution 640x480
    const float fFocalLength = config::focal;

    // compute initial pose
    float rot[4], trans[3];
    getInitialPose(rot, trans, p2D, markerSize, fFocalLength);

    // corner 3D coordinates
    float fCp = (markerSize / 2);

#ifdef _WIN32
    // define something for Windows (32-bit and 64-bit, this part is common)

#ifdef _WIN64
    // define something for Windows (64-bit only)
    CvPoint3D32f points3D[4] =

        {{-fCp, fCp, 0.0f}, {-fCp, -fCp, 0.0f}, {fCp, -fCp, 0.0f}, {fCp, fCp, 0.0f}};  // counter-clock-wise

#else
    // define something for Windows (32-bit only)
#endif
#elif __APPLE__
#include "TargetConditionals.h"
#if TARGET_IPHONE_SIMULATOR
    // iOS Simulator
#elif TARGET_OS_IPHONE
    // iOS device
#elif TARGET_OS_MAC
    // Other kinds of Mac OS
    CvPoint3D32f points3D[4];
    points3D[0] = cvPoint3D32f(-fCp, fCp, 0.0f);
    points3D[1] = cvPoint3D32f(-fCp, -fCp, 0.0f);
    points3D[2] = cvPoint3D32f(fCp, -fCp, 0.0f);
    points3D[3] = cvPoint3D32f(fCp, fCp, 0.0f);  // counter-clock-wise

#else
#error "Unknown Apple platform"
#endif
#elif __linux__
    // linux
    // Other kinds of Mac OS
    CvPoint3D32f points3D[4];
    points3D[0] = cvPoint3D32f(-fCp, fCp, 0.0f);
    points3D[1] = cvPoint3D32f(-fCp, -fCp, 0.0f);
    points3D[2] = cvPoint3D32f(fCp, -fCp, 0.0f);
    points3D[3] = cvPoint3D32f(fCp, fCp, 0.0f);  // counter-clock-wise
#elif __unix__  // all unices not caught above
                // Other kinds of Mac OS
    CvPoint3D32f points3D[4];
    points3D[0] = cvPoint3D32f(-fCp, fCp, 0.0f);
    points3D[1] = cvPoint3D32f(-fCp, -fCp, 0.0f);
    points3D[2] = cvPoint3D32f(fCp, -fCp, 0.0f);
    points3D[3] = cvPoint3D32f(fCp, fCp, 0.0f);  // counter-clock-wise
                                                 // Unix
#elif defined(_POSIX_VERSION)
    // POSIX
    // Other kinds of Mac OS
    CvPoint3D32f points3D[4];
    points3D[0] = cvPoint3D32f(-fCp, fCp, 0.0f);
    points3D[1] = cvPoint3D32f(-fCp, -fCp, 0.0f);
    points3D[2] = cvPoint3D32f(fCp, -fCp, 0.0f);
    points3D[3] = cvPoint3D32f(fCp, fCp, 0.0f);  // counter-clock-wise
#else
#error "Unknown compiler"
#endif

    // refine pose using nonlinear optimization
    CvPoint2D32f points[4];

    for (int i = 0; i < 4; i++) {
        points[i].x = p2D[i].x;
        points[i].y = p2D[i].y;
    }

    optimizePose(rot, trans, 4, points, points3D, fFocalLength);

    // convert quaternion to matrix
    float X = -rot[0];
    float Y = -rot[1];
    float Z = -rot[2];
    float W = rot[3];

    float xx = X * X;
    float xy = X * Y;
    float xz = X * Z;
    float xw = X * W;
    float yy = Y * Y;
    float yz = Y * Z;
    float yw = Y * W;
    float zz = Z * Z;
    float zw = Z * W;

    mat[0] = 1 - 2 * (yy + zz);
    mat[1] = 2 * (xy + zw);
    mat[2] = 2 * (xz - yw);
    mat[4] = 2 * (xy - zw);
    mat[5] = 1 - 2 * (xx + zz);
    mat[6] = 2 * (yz + xw);
    mat[8] = 2 * (xz + yw);
    mat[9] = 2 * (yz - xw);
    mat[10] = 1 - 2 * (xx + yy);
    mat[3] = trans[0];
    mat[7] = trans[1];
    mat[11] = trans[2];
    mat[12] = mat[13] = mat[14] = 0;
    mat[15] = 1;

#ifdef PRINT_OPTIMIZATION
    printf("rot: %5.3f %5.3f %5.3f %5.3f\n", (double)rot[0], (double)rot[1], (double)rot[2], (double)rot[3]);
    printf("tra: %5.3f %5.3f %5.3f\n", (double)trans[0], (double)trans[1], (double)trans[2]);
#endif
}

// Returns Matrix in Row-major format
void calcHomography(float *pResult, const CvPoint2D32f *pQuad) {
    // homography computation Ela Harker & O'Leary, simplified for squares
    // subtract mean from points
    CvPoint2D32f c[4];
    CvPoint2D32f mean;

    mean.x = 0;
    mean.y = 0;
    for (int i = 0; i < 4; i++) {
        mean.x = mean.x + pQuad[i].x;
        mean.y = mean.y + pQuad[i].y;
    }

    mean.x = mean.x / 4;
    mean.y = mean.y / 4;

    for (int i = 0; i < 4; i++) {
        c[i].x = pQuad[i].x - mean.x;
        c[i].y = pQuad[i].y - mean.y;
    }

    // build simplified matrix A
    float fMatA[4][3];

    fMatA[0][0] = (c[0].x - c[1].x - c[2].x + c[3].x);
    fMatA[0][1] = (-c[0].x - c[1].x + c[2].x + c[3].x);
    fMatA[0][2] = (-2 * (c[0].x + c[2].x));
    fMatA[1][0] = -fMatA[0][0];
    fMatA[1][1] = -fMatA[0][1];
    fMatA[1][2] = (-2 * (c[1].x + c[3].x));
    fMatA[2][0] = (c[0].y - c[1].y - c[2].y + c[3].y);
    fMatA[2][1] = (-c[0].y - c[1].y + c[2].y + c[3].y);
    fMatA[2][2] = (-2 * (c[0].y + c[2].y));
    fMatA[3][0] = -fMatA[2][0];
    fMatA[3][1] = -fMatA[2][1];
    fMatA[3][2] = (-2 * (c[1].y + c[3].y));

    CvMat matA = cvMat(4, 3, CV_32F, fMatA[0]);

    // compute SVD

    // Idea: replace the whole thing with an analytical solution of the line at infinity
    float W[3];
    CvMat matW = cvMat(3, 1, CV_32F, W);
    float V[3][3];
    CvMat matV = cvMat(3, 3, CV_32F, V[0]);
    cvSVD(&matA, &matW, NULL, &matV, CV_SVD_MODIFY_A | CV_SVD_V_T);

    // copy bottom line of homography
    pResult[6] = V[2][0];
    pResult[7] = V[2][1];
    pResult[8] = V[2][2];

    // compute entries 1,1 and 1,2, multiply by 2 to compensate scaling
    pResult[0] = ((c[0].x + c[1].x + c[2].x + c[3].x) * pResult[6] +
                  (-c[0].x + c[1].x - c[2].x + c[3].x) * pResult[7] +
                  (-c[0].x - c[1].x + c[2].x + c[3].x) * pResult[8]) /
                 2;

    pResult[1] = ((-c[0].x + c[1].x - c[2].x + c[3].x) * pResult[6] +
                  (c[0].x + c[1].x + c[2].x + c[3].x) * pResult[7] +
                  (c[0].x - c[1].x - c[2].x + c[3].x) * pResult[8]) /
                 2;

    // compute entries 2,1 and 2,2, multiply by 2 to compensate scaling
    pResult[3] = ((c[0].y + c[1].y + c[2].y + c[3].y) * pResult[6] +
                  (-c[0].y + c[1].y - c[2].y + c[3].y) * pResult[7] +
                  (-c[0].y - c[1].y + c[2].y + c[3].y) * pResult[8]) /
                 2;

    pResult[4] = ((-c[0].y + c[1].y - c[2].y + c[3].y) * pResult[6] +
                  (c[0].y + c[1].y + c[2].y + c[3].y) * pResult[7] +
                  (c[0].y - c[1].y - c[2].y + c[3].y) * pResult[8]) /
                 2;

    // compute entries 1,3 and 2,3
    pResult[2] = ((c[0].x + c[1].x - c[2].x - c[3].x) * pResult[6] +
                  (-c[0].x + c[1].x + c[2].x - c[3].x) * pResult[7]) /
                 -4;

    pResult[5] = ((c[0].y + c[1].y - c[2].y - c[3].y) * pResult[6] +
                  (-c[0].y + c[1].y + c[2].y - c[3].y) * pResult[7]) /
                 -4;

    // now multiply last row with factor 2 to compensate scaling
    pResult[6] = V[2][0] * 2;
    pResult[7] = V[2][1] * 2;

    // multiply with shift to compensate mean subtraction
    for (int i = 0; i < 3; i++) {
        pResult[i] = pResult[i] + pResult[6 + i] * mean.x;
        pResult[3 + i] = pResult[3 + i] + pResult[6 + i] * mean.y;
    }
}

#include <iostream>
#include <opencv2/opencv.hpp>
#include "../../game/Game.h"
#include "../pose/math/pose-estimation.h"
#include "../pose/pose.h"
using namespace cv;
using namespace std;
#define EX5 1
#define EX5_RAW 0
#define DRAW_CONTOUR 0
#define DRAW_RECTANGLE 0
#define THICKNESS_VALUE 4
struct MyStrip {
    int stripeLength;
    int nStop;
    int nStart;
    Point2f stripeVecX;
    Point2f stripeVecY;
};
typedef vector<Point> contour_t;
typedef vector<contour_t> contour_vector_t;
int bw_thresh = 55;

int subpixSampleSafe(const Mat &pSrc, const Point2f &p) {
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
Mat calculate_Stripe(double dx, double dy, MyStrip &st) {
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

void getMatrices(Mat frame, glm::mat4 &extrinsicMat, glm::mat4 &extrinsicMatPlayer, glm::mat4 &extrinsicMatButton, bool &isMarkerDetected, bool &isPlayerMarkerDetected, bool &isButtonMarkerDetected) {
    Mat imgFiltered;
    imgFiltered = frame.clone();
    cvtColor(imgFiltered, imgGrayScale, COLOR_BGR2GRAY);
    adaptiveThreshold(imgGrayScale, imgAdaptive, config::maxIntensity, ATH_TYPE, cv::THRESH_BINARY, config::blockSize, config::athConst);
    contour_vector_t contours;
    findContours(imgAdaptive, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    for (size_t k = 0; k < contours.size(); k++) {
        contour_t approx_contour;
        approxPolyDP(contours[k], approx_contour, arcLength(contours[k], true) * 0.02, true);
        Scalar QUADRILATERAL_COLOR(0, 0, 255);
        Scalar colour;
        Rect r = boundingRect(approx_contour);
        if (approx_contour.size() == 4) {
            colour = QUADRILATERAL_COLOR;
        } else {
            continue;
        }
        if (r.height < 20 || r.width < 20 || r.width > imgFiltered.cols - 10 || r.height > imgFiltered.rows - 10) {
            continue;
        }
        polylines(imgFiltered, approx_contour, true, colour, THICKNESS_VALUE);
        float lineParams[16];
        Mat lineParamsMat(Size(4, 4), CV_32F, lineParams);
        for (size_t i = 0; i < approx_contour.size(); ++i) {
            double dx = ((double)approx_contour[(i + 1) % 4].x - (double)approx_contour[i].x) / 7.0;
            double dy = ((double)approx_contour[(i + 1) % 4].y - (double)approx_contour[i].y) / 7.0;
            MyStrip strip;
            Mat imagePixelStripe = calculate_Stripe(dx, dy, strip);
            Point2f edgePointCenters[6];
            for (int j = 1; j < 7; ++j) {
                double px = (double)approx_contour[i].x + (double)j * dx;
                double py = (double)approx_contour[i].y + (double)j * dy;

                Point p;
                p.x = (int)px;
                p.y = (int)py;
                for (int m = -1; m <= 1; ++m) {
                    for (int n = strip.nStart; n <= strip.nStop; ++n) {
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
                for (int n = 1; n < (strip.stripeLength - 1); n++) {
                    unsigned char *stripePtr = &(imagePixelStripe.at<uchar>(n - 1, 0));
                    double r1 = -stripePtr[0] - 2. * stripePtr[1] - stripePtr[2];
                    stripePtr += 2 * imagePixelStripe.step;
                    double r3 = stripePtr[0] + 2. * stripePtr[1] + stripePtr[2];
                    unsigned int ti = n - 1;
                    sobelValues[ti] = r1 + r3;
                }

                double maxIntensity = -1;
                int maxIntensityIndex = 0;
                for (int n = 0; n < strip.stripeLength - 2; ++n) {
                    if (sobelValues[n] > maxIntensity) {
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
                if (isnan(pos)) {
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
        for (int i = 0; i < 4; ++i) {
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
            if (fabs(c) < 0.001) {
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
        for (int i = 0; i < 6; ++i) {
            // Check if border is black
            int pixel1 = imageMarker.at<uchar>(0, i);  // top
            int pixel2 = imageMarker.at<uchar>(5, i);  // bottom
            int pixel3 = imageMarker.at<uchar>(i, 0);  // left
            int pixel4 = imageMarker.at<uchar>(i, 5);  // right
            if ((pixel1 > 0) || (pixel2 > 0) || (pixel3 > 0) || (pixel4 > 0)) {
                code = -1;
                break;
            }
        }
        if (code < 0) {
            continue;
        }
        int cP[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                cP[i][j] = imageMarker.at<uchar>(i + 1, j + 1);
                cP[i][j] = (cP[i][j] == 0) ? 1 : 0;
            }
        }
        int codes[4];
        codes[0] = codes[1] = codes[2] = codes[3] = 0;
        for (int i = 0; i < 16; i++) {
            int row = i >> 2;
            int col = i % 4;
            codes[0] <<= 1;
            codes[0] |= cP[row][col];  // 0�
            codes[1] <<= 1;
            codes[1] |= cP[3 - col][row];  // 90�
            codes[2] <<= 1;
            codes[2] |= cP[3 - row][3 - col];  // 180�
            codes[3] <<= 1;
            codes[3] |= cP[col][3 - row];  // 270�
        }
        if ((codes[0] == 0) || (codes[0] == 0xffff)) {
            continue;
        }
        int angle = 0;
        code = codes[0];
        for (int i = 1; i < 4; ++i) {
            if (codes[i] < code) {
                code = codes[i];
                angle = i;
            }
        }
        if (angle != 0) {
            Point2f corrected_corners[4];
            for (int i = 0; i < 4; i++)
                corrected_corners[(i + angle) % 4] = corners[i];
            for (int i = 0; i < 4; i++)
                corners[i] = corrected_corners[i];
        }
        for (int i = 0; i < 4; i++) {
            corners[i].x -= 320;
            corners[i].y = -corners[i].y + 180;
        }
        float resultMatrix[16];
        pose::estimateSquarePose(resultMatrix, (Point2f *)corners, MARKER_SIZE * 2);

        if (code == CODE_1) {
            float transposed[16];

            for (int x = 0; x < 4; ++x)
                for (int y = 0; y < 4; ++y)
                    transposed[x * 4 + y] = resultMatrix[y * 4 + x];

            memcpy(glm::value_ptr(extrinsicMat), transposed, sizeof(transposed));
            isMarkerDetected = true;
        } else if (code == CODE_2) {
            float transposed[16];

            for (int x = 0; x < 4; ++x)
                for (int y = 0; y < 4; ++y)
                    transposed[x * 4 + y] = resultMatrix[y * 4 + x];

            memcpy(glm::value_ptr(extrinsicMatPlayer), transposed, sizeof(transposed));
            isPlayerMarkerDetected = true;
        } else if (code == CODE_3) {
            float transposed[16];

            for (int x = 0; x < 4; ++x)
                for (int y = 0; y < 4; ++y)
                    transposed[x * 4 + y] = resultMatrix[y * 4 + x];

            memcpy(glm::value_ptr(extrinsicMatButton), transposed, sizeof(transposed));
            isButtonMarkerDetected = true;
        }
    }
}
}  // namespace pose