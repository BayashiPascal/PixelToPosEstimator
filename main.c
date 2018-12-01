#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <tgmath.h>
#include "genalg.h"

typedef struct PixelToPosEstimator {
  // Camera position
  VecFloat3D _cameraPos;
  // Dimension of the image
  VecFloat2D _imgSize;
  // Projection parameters
  // (Px, Py, Sx, Sy, Upx, Upy, Upz)
  VecFloat* _param;
} PixelToPosEstimator;

#define Px(that) VecGet(that->_param, 0)
#define Py(that) VecGet(that->_param, 1)
#define Sx(that) VecGet(that->_param, 2)
#define Sy(that) VecGet(that->_param, 3)
#define Upx(that) VecGet(that->_param, 4)
#define Upy(that) VecGet(that->_param, 5)
#define Upz(that) VecGet(that->_param, 6)

#define NBPARAM 7

PixelToPosEstimator PixelToPosEstimatorCreateStatic(
  VecFloat3D* posCamera, const VecFloat2D* const imgSize) {
  // Declare the new estimator
  PixelToPosEstimator estimator;
  // Init the estimator
  estimator._cameraPos = *posCamera;
  estimator._imgSize = *imgSize;
  estimator._param = VecFloatCreate(NBPARAM);
  // Return the new estimator
  return estimator;
}

void PixelToPosEstimatorFreeStatic(PixelToPosEstimator* const that) {
  VecFree(&(that->_param));
}

VecFloat3D PixelToPosEstimatorCameraPolarToRealPos(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const polarPos) {
  // Declare a variable to memorize the result
  VecFloat3D res = VecFloatCreateStatic3D();
  // Calculate the real coordinates

  // POV
  VecFloat3D P = VecFloatCreateStatic3D();
  VecSet(&P, 0, Px(that));
  VecSet(&P, 1, Py(that));
  VecSet(&P, 2, 0.0);
  // Normalized vector Camera->POV
  VecFloat3D CP = VecGetOp(&P, 1.0, &(that->_cameraPos), -1.0);
  VecNormalise(&CP);
  // Normalized up vector
  VecFloat3D Up = VecFloatCreateStatic3D();
  VecSet(&Up, 0, Upx(that));
  VecSet(&Up, 1, Upy(that));
  VecSet(&Up, 2, Upz(that));
  VecNormalise(&Up);
  // Normalized right vector
  VecFloat3D Right = VecCrossProd(&CP, &Up);
  VecNormalise(&Right);
  // Rotation according to up and right
  VecFloat3D Rx = CP;
  VecRotAxis(&Rx, &Up, Sx(that) * VecGet(polarPos, 0));
  Rx = VecGetOp(&Rx, 1.0, &CP, -1.0);
  VecFloat3D Ry = CP;
  VecRotAxis(&Ry, &Right, Sy(that) * VecGet(polarPos, 1));
  Ry = VecGetOp(&Ry, 1.0, &CP, -1.0);
  // 3d vector from camera corresponding to the polar pos pixel
  VecFloat3D V = VecGetOp(&CP, 1.0, &Rx, 1.0);
  V = VecGetOp(&V, 1.0, &Ry, 1.0);
  VecNormalise(&V);
  // Projection to ground plane
  float a = VecGet(&(that->_cameraPos), 1) / VecGet(&V, 1);
  VecSet(&res, 0, VecGet(&(that->_cameraPos), 0) - a * VecGet(&V, 0));
  VecSet(&res, 1, 0.0);
  VecSet(&res, 2, VecGet(&(that->_cameraPos), 2) - a * VecGet(&V, 2));

  // Return the result
  return res;
}

VecFloat2D PixelToPosEstimatorGetPosPxToPolar(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const screenPos) {
  // Declare a variable to memorize the result
  VecFloat2D res = VecFloatCreateStatic2D();
  // Calculate the polar coordinates
  float w = VecGet(&(that->_imgSize), 0);
  VecSet(&res, 0, (VecGet(screenPos, 0) - 0.5 * w) / (0.5 * w));
  float h = VecGet(&(that->_imgSize), 1);
  VecSet(&res, 1, (VecGet(screenPos, 1) - 0.5 * h) / (0.5 * h));
  // Return the result
  return res;
}

void PixelToPosEstimatorInit(PixelToPosEstimator* const that,
  const GSet* const posMeter, const GSet* const posPixel) {
  // Calculate the projection parameters of the estimator

  srandom(time(NULL));
  int lengthAdnF = NBPARAM;
  int lengthAdnI = 0;
  GenAlg* ga = GenAlgCreate(GENALG_NBENTITIES, GENALG_NBELITES, 
    lengthAdnF, lengthAdnI);
  VecFloat2D boundsF = VecFloatCreateStatic2D();
  VecSet(&boundsF, 0, 0.0); VecSet(&boundsF, 1, 10.0);
  GASetBoundsAdnFloat(ga, 0, &boundsF); // Px
  VecSet(&boundsF, 0, 0.0); VecSet(&boundsF, 1, 20.0);
  GASetBoundsAdnFloat(ga, 1, &boundsF); // Py
  VecSet(&boundsF, 0, -PBMATH_HALFPI); 
  VecSet(&boundsF, 1, PBMATH_HALFPI);
  GASetBoundsAdnFloat(ga, 2, &boundsF); // Sx
  GASetBoundsAdnFloat(ga, 3, &boundsF); // Sy
  VecSet(&boundsF, 0, -1.0); VecSet(&boundsF, 1, 1.0);
  GASetBoundsAdnFloat(ga, 4, &boundsF); // Upx
  VecSet(&boundsF, 0, 0.0); VecSet(&boundsF, 1, 1.0);
  GASetBoundsAdnFloat(ga, 5, &boundsF); // Upy
  VecSet(&boundsF, 0, -1.0); VecSet(&boundsF, 1, 1.0);
  GASetBoundsAdnFloat(ga, 6, &boundsF); // Upz

  GAInit(ga);
  float best = 10000.0;
  do {
    //printf("epoch %ld avg err %fm     \r", 
    //  GAGetCurEpoch(ga), best / (float)GSetNbElem(posMeter));
    fflush(stdout);
    float ev = 0.0;
    for (int iEnt = 0; iEnt < GAGetNbAdns(ga); ++iEnt) {
      VecCopy(that->_param, GAAdnAdnF(GAAdn(ga, iEnt)));
      ev = 0.0;
      GSetIterForward iterMeter = GSetIterForwardCreateStatic(posMeter);
      GSetIterForward iterPixel = GSetIterForwardCreateStatic(posPixel);
      do {
        VecFloat3D* pMeter = GSetIterGet(&iterMeter);
        VecFloat2D* pPixel = GSetIterGet(&iterPixel);
        VecFloat2D polarPos = 
          PixelToPosEstimatorGetPosPxToPolar(that, pPixel);
        VecFloat3D pEstim = 
          PixelToPosEstimatorCameraPolarToRealPos(that, &polarPos);
        ev += VecDist(pMeter, &pEstim);
      } while (GSetIterStep(&iterMeter) && GSetIterStep(&iterPixel));
      ev /=  (float)GSetNbElem(posMeter);
      GASetAdnValue(ga, GAAdn(ga, iEnt), -1.0 * ev);
      if (ev < best - PBMATH_EPSILON) {
        best = ev;
        //printf("%lu %f ", GAGetCurEpoch(ga), best);
        //VecFloatPrint(that->_param, stdout, 6);
        //printf("        \n"); fflush(stdout);
      }
    }
    GAStep(ga);
  } while (GAGetCurEpoch(ga) < 100000 && best > 0.01);
  VecCopy(that->_param, GAAdnAdnF(GABestAdn(ga)));

  GenAlgFree(&ga);
}

VecFloat3D PixelToPosEstimatorGetPosPxToMeter(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const screenPos) {
  // Declare a variable to memorize the result
  VecFloat3D res = VecFloatCreateStatic3D();
  // Calculate the real coordinates
  VecFloat2D polarPos = 
    PixelToPosEstimatorGetPosPxToPolar(that, screenPos);
  res = PixelToPosEstimatorCameraPolarToRealPos(that, &polarPos);
  // Return the result
  return res;
}

int main(int argc, char** argv) {
  (void)argc; (void)argv;

  // Example of use: estimating positions on a baseball field kowing
  // the bases' position
  
  // Size of one side of the ground
  float groundSize = 27.431;
  // Variables to memorize the position of the bases in meter and pixel
  #define NBINPUT 4
  #define NBDATA 8
  VecFloat3D basesPosMeter[NBDATA];
  VecFloat2D basesPosPixel[NBDATA];
  for (int iBase = 0; iBase < NBDATA; ++iBase) {
    basesPosMeter[iBase] = VecFloatCreateStatic3D();
    basesPosPixel[iBase] = VecFloatCreateStatic2D();
  }
  float d = sqrt(fastpow(groundSize, 2) / 2.0);
  VecSet(basesPosMeter + 1, 0, d);
  VecSet(basesPosMeter + 1, 2, d);
  VecSet(basesPosMeter + 2, 2, 2.0 * d);
  VecSet(basesPosMeter + 3, 0, -d);
  VecSet(basesPosMeter + 3, 2, d);
  VecSet(basesPosMeter + 4, 0, -5);
  VecSet(basesPosMeter + 4, 2, -20);
  VecSet(basesPosMeter + 5, 0, -25);
  VecSet(basesPosMeter + 5, 2, -10);
  VecSet(basesPosMeter + 6, 0, 35);
  VecSet(basesPosMeter + 6, 2, 50);
  VecSet(basesPosMeter + 7, 0, -5);
  VecSet(basesPosMeter + 7, 2, 12);
  VecSet(basesPosPixel, 0, 614);
  VecSet(basesPosPixel, 1, 492);
  VecSet(basesPosPixel + 1, 0, 824);
  VecSet(basesPosPixel + 1, 1, 438);
  VecSet(basesPosPixel + 2, 0, 631);
  VecSet(basesPosPixel + 2, 1, 405);
  VecSet(basesPosPixel + 3, 0, 427);
  VecSet(basesPosPixel + 3, 1, 436);
  VecSet(basesPosPixel + 4, 0, 473);
  VecSet(basesPosPixel + 4, 1, 620);
  VecSet(basesPosPixel + 5, 0, 170);
  VecSet(basesPosPixel + 5, 1, 538);
  VecSet(basesPosPixel + 6, 0, 884);
  VecSet(basesPosPixel + 6, 1, 394);
  VecSet(basesPosPixel + 7, 0, 564);
  VecSet(basesPosPixel + 7, 1, 454);
  // Variable to memorize the position of the camera
  VecFloat3D posCamera = VecFloatCreateStatic3D();
  VecSet(&posCamera, 0, 2.75);
  VecSet(&posCamera, 1, 14.13);
  VecSet(&posCamera, 2, -49.8);
  // Variable to memorize the dimensions of the image
  VecFloat2D imgSize = VecFloatCreateStatic2D();
  VecSet(&imgSize, 0, 1280);
  VecSet(&imgSize, 1, 720);
  // Variable to memorize the name of the bases
  const char* baseName[NBDATA] = 
    {"Home base", "1st base", "2nd base", "3rd base",
      "Test1", "Test2", "Test3", "Test4"};
  // Create the estimator
  PixelToPosEstimator estimator = 
    PixelToPosEstimatorCreateStatic(&posCamera, &imgSize);

  // Calculate the projection parameters
  printf("Calculate the projection param...\n");
  GSet posMeter = GSetCreateStatic();
  GSet posPixel = GSetCreateStatic();
  for (int iBase = 0; iBase < NBINPUT; ++iBase) {
    GSetAppend(&posMeter, basesPosMeter + iBase);
    GSetAppend(&posPixel, basesPosPixel + iBase);
  }
  PixelToPosEstimatorInit(&estimator, &posMeter, &posPixel);
  GSetFlush(&posMeter);
  GSetFlush(&posPixel);
  
  /*printf("Reuse the projection param...\n");
  float param[7] = {1.891078,9.284310,0.868436,-0.488767,-0.005342,0.973494,0.204840};
  for (int iParam = NBPARAM; iParam--;) {
    VecSet(estimator._param, iParam, param[iParam]);
  }*/
  
  printf("\n");
  printf("Projection param: ");
  VecPrint(estimator._param, stdout);printf("\n");
  printf("\n");
  
  float avgErr = 0.0;
  float maxErr = 0.0;
  for (int iBase = 0; iBase < NBDATA; ++iBase) {
    printf("%s (m): ", baseName[iBase]); 
    VecPrint(basesPosMeter + iBase, stdout); 
    printf(" (px): "); 
    VecPrint(basesPosPixel + iBase, stdout); 
    printf("\n");
    printf(" (polar): "); 
    VecFloat2D polarPos = PixelToPosEstimatorGetPosPxToPolar(
      &estimator, basesPosPixel + iBase);
    VecPrint(&polarPos, stdout); 
    printf("\n");
    VecFloat3D realPos = PixelToPosEstimatorCameraPolarToRealPos(
      &estimator, &polarPos);
    printf(" (real): "); 
    VecPrint(&realPos, stdout); 
    float errorReal = VecDist(basesPosMeter + iBase, &realPos);
    printf(" (error): %fm", errorReal); 
    printf("\n");
    printf(" (screen->real): "); 
    VecFloat3D estimPos = PixelToPosEstimatorGetPosPxToMeter(
      &estimator, basesPosPixel + iBase);
    VecPrint(&estimPos, stdout);
    float error = VecDist(&estimPos, basesPosMeter + iBase);
    printf(" (error): %fm", error); 
    printf("\n\n");
    avgErr += error;
    if (maxErr < error)
      maxErr = error;
  }
  avgErr /= (float)NBDATA;
  printf("Average error: %fm\n", avgErr); 
  printf("Max error: %fm\n", maxErr); 
  
  // Free memory
  PixelToPosEstimatorFreeStatic(&estimator);
  
  // Return success code
  return 0;
}

