#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <tgmath.h>
#include "genalg.h"

typedef struct PixelToPosEstimator {
  // Camera position
  VecFloat3D _cameraPos;
  // Camera ground position
  VecFloat3D _cameraGroundPos;
  // Dimension of the image
  VecFloat2D _imgSize;
  // Estimated position of the POV
  VecFloat3D _povPos;
  // Projection parameters
  // (theta, f, Sx, Sy, Ox, Oy)
  VecFloat* _param;
  
} PixelToPosEstimator;

PixelToPosEstimator PixelToPosEstimatorCreateStatic(
  VecFloat3D* posCamera, const VecFloat2D* const imgSize,
  const VecFloat3D* const povPos) {
  // Declare the new estimator
  PixelToPosEstimator estimator;
  // Init the estimator
  estimator._cameraPos = *posCamera;
  estimator._cameraGroundPos = *posCamera;
  VecSet(&(estimator._cameraGroundPos), 1, 0.0);
  estimator._imgSize = *imgSize;
  estimator._povPos = *povPos;
  estimator._param = VecFloatCreate(6);
  
  // Return the new estimator
  return estimator;
}

void PixelToPosEstimatorFreeStatic(PixelToPosEstimator* const that) {
  VecFree(&(that->_param));
}

// Return [theta, dist]
// theta in [-PI,PI], dist in meter
VecFloat2D PixelToPosEstimatorRealPosToCameraPolar(
  const PixelToPosEstimator* const that, 
  const VecFloat3D* const realPos) {
  // Declare a variable to memorize the result
  VecFloat2D res = VecFloatCreateStatic2D();
  // Calculate the polar coordinates
  VecFloat3D CP = 
    VecGetOp(realPos, 1.0, &(that->_cameraGroundPos), -1.0);
  VecFloat2D CP2D = VecFloatCreateStatic2D();
  VecSet(&CP2D, 0, VecGet(&CP, 0));
  VecSet(&CP2D, 1, VecGet(&CP, 2));
  VecFloat3D CPOV = 
    VecGetOp(&(that->_povPos), 1.0, &(that->_cameraGroundPos), -1.0);
  VecFloat2D CPOV2D = VecFloatCreateStatic2D();
  VecSet(&CPOV2D, 0, VecGet(&CPOV, 0));
  VecSet(&CPOV2D, 1, VecGet(&CPOV, 2));
  VecSet(&res, 1, VecNorm(&CP2D));
  VecNormalise(&CP2D);
  VecNormalise(&CPOV2D);
  VecSet(&res, 0, VecAngleTo(&CP2D, &CPOV2D));
  // Return the result
  return res;
}

// 'polarPos': [theta, dist]
// theta in [-PI,PI], dist in meter
VecFloat3D PixelToPosEstimatorCameraPolarToRealPos(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const polarPos) {
  // Declare a variable to memorize the result
  VecFloat3D res = VecFloatCreateStatic3D();
  // Calculate the real coordinates
  VecFloat3D CPOV = 
    VecGetOp(&(that->_povPos), 1.0, &(that->_cameraGroundPos), -1.0);
  VecNormalise(&CPOV);
  res = VecGetScale(&CPOV, VecGet(polarPos, 1));
  VecRotY(&res, VecGet(polarPos, 0));
  VecOp(&res, 1.0, &(that->_cameraGroundPos), 1.0);
  // Return the result
  return res;
}

// 'polarPos': [theta, dist]
// theta in [-PI,PI], dist in meter
VecFloat2D PixelToPosEstimatorCameraPolarToScreenPos(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const polarPos) {
  // Declare a variable to memorize the result
  VecFloat2D res = VecFloatCreateStatic2D();
  // Calculate the screen coordinates
  long double phi = PBMATH_HALFPI - VecGet(polarPos, 0);

  long double Rx = VecGet(polarPos, 1);
  long double Ry = VecGet(that->_param, 1) * 
    (tanl(VecGet(that->_param, 0)) - 
    tanl(VecGet(that->_param, 0) - 
    atanl((long double)VecGet(polarPos, 1) / 
    (long double)VecGet(&(that->_cameraPos), 1))));

  long double x = VecGet(that->_param, 4) + 
    VecGet(that->_param, 2) * Rx * cos(phi);
  VecSet(&res, 0, x);
  long double y = VecGet(that->_param, 5) - 
    VecGet(that->_param, 3) * Ry * sin(phi);
  VecSet(&res, 1, y);

  // Return the result
  return res;
}

void PixelToPosEstimatorInit(PixelToPosEstimator* const that,
  const GSet* const posMeter, const GSet* const posPixel) {
  // Calculate the projection parameters of the estimator

  srandom(time(NULL));
  int lengthAdnF = 6;
  int lengthAdnI = 0;
  GenAlg* ga = GenAlgCreate(GENALG_NBENTITIES, GENALG_NBELITES, 
    lengthAdnF, lengthAdnI);
  VecFloat2D boundsF = VecFloatCreateStatic2D();
  VecSet(&boundsF, 0, PBMATH_QUARTERPI); 
  VecSet(&boundsF, 1, 3.0 * PBMATH_QUARTERPI);
  GASetBoundsAdnFloat(ga, 0, &boundsF); // theta
  VecSet(&boundsF, 0, 10.0); VecSet(&boundsF, 1, 100.0);
  GASetBoundsAdnFloat(ga, 1, &boundsF); // f
  VecSet(&boundsF, 0, 0.0); VecSet(&boundsF, 1, 10000.0);
  GASetBoundsAdnFloat(ga, 2, &boundsF); // Sx
  VecSet(&boundsF, 0, 0.0); VecSet(&boundsF, 1, 10000.0);
  GASetBoundsAdnFloat(ga, 3, &boundsF); // Sy
  VecSet(&boundsF, 0, 0.3 * (float)VecGet(&(that->_imgSize), 0)); 
  VecSet(&boundsF, 1, 0.6 * (float)VecGet(&(that->_imgSize), 0));
  GASetBoundsAdnFloat(ga, 4, &boundsF); // Ox
  VecSet(&boundsF, 0, (float)VecGet(&(that->_imgSize), 1));
  VecSet(&boundsF, 1, 5.0 * (float)VecGet(&(that->_imgSize), 1));
  GASetBoundsAdnFloat(ga, 5, &boundsF); // Oy

  GAInit(ga);
  float best = 10000.0;
  do {
    printf("epoch %ld %fpx     \r", GAGetCurEpoch(ga), best);
    fflush(stdout);
    float ev = 0.0;
    for (int iEnt = 0; iEnt < GAGetNbAdns(ga); ++iEnt) {
      VecCopy(that->_param, GAAdnAdnF(GAAdn(ga, iEnt)));
      ev = 0.0;
      GSetIterForward iterMeter = GSetIterForwardCreateStatic(posMeter);
      GSetIterForward iterPixel = GSetIterForwardCreateStatic(posPixel);
      do {
        VecFloat3D* pMeter = GSetIterGet(&iterMeter);
        VecFloat2D polarPos = 
          PixelToPosEstimatorRealPosToCameraPolar(that, pMeter);
        VecFloat2D* pPixel = GSetIterGet(&iterPixel);
        VecFloat2D pEstim = 
          PixelToPosEstimatorCameraPolarToScreenPos(that, &polarPos);
        ev += sqrt(
          fastpow(VecGet(&pEstim, 0) - VecGet(pPixel, 0), 2) + 
          fastpow(VecGet(&pEstim, 1) - VecGet(pPixel, 1), 2));
      } while (GSetIterStep(&iterMeter) && GSetIterStep(&iterPixel));
      GASetAdnValue(ga, GAAdn(ga, iEnt), -1.0 * ev);
      if (ev < best - PBMATH_EPSILON) {
        best = ev;
        printf("%lu %f ", GAGetCurEpoch(ga), 
          best / (float)GSetNbElem(posMeter));
        VecFloatPrint(that->_param, stdout, 6);
        printf("        \n"); fflush(stdout);
      }
    }
    GAStep(ga);
  } while (GAGetCurEpoch(ga) < 1000000 && best > PBMATH_EPSILON);
  VecCopy(that->_param, GAAdnAdnF(GABestAdn(ga)));

  GenAlgFree(&ga);
}

VecFloat3D PixelToPosEstimatorGetPosPxToMeter(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const screenPos) {
  // Declare a variable to memorize the result
  VecFloat3D res = VecFloatCreateStatic3D();
  // Calculate the real coordinates

  srandom(time(NULL));
  int lengthAdnF = 2;
  int lengthAdnI = 0;
  GenAlg* ga = GenAlgCreate(GENALG_NBENTITIES, GENALG_NBELITES, 
    lengthAdnF, lengthAdnI);
  VecFloat2D boundsF = VecFloatCreateStatic2D();
  VecSet(&boundsF, 0, -PBMATH_HALFPI); 
  VecSet(&boundsF, 1, PBMATH_HALFPI);
  GASetBoundsAdnFloat(ga, 0, &boundsF); // theta
  VecSet(&boundsF, 0, 10.0); VecSet(&boundsF, 1, 500.0);
  GASetBoundsAdnFloat(ga, 1, &boundsF); // dist

  GAInit(ga);
  float best = 10000.0;
  do {
    //printf("epoch %ld   \r", GAGetCurEpoch(ga));fflush(stdout);
    float ev = 0.0;
    for (int iEnt = 0; iEnt < GAGetNbAdns(ga); ++iEnt) {
      VecFloat2D pEstim = PixelToPosEstimatorCameraPolarToScreenPos(
        that, (VecFloat2D*)GAAdnAdnF(GAAdn(ga, iEnt)));
      ev = sqrt(
        fastpow(VecGet(&pEstim, 0) - VecGet(screenPos, 0), 2) + 
        fastpow(VecGet(&pEstim, 1) - VecGet(screenPos, 1), 2));
      GASetAdnValue(ga, GAAdn(ga, iEnt), -1.0 * ev);
      if (ev < best - PBMATH_EPSILON) {
        best = ev;
        /*printf("%lu %f ", GAGetCurEpoch(ga), best);
        VecFloatPrint(GAAdnAdnF(GAAdn(ga, iEnt)), stdout, 6);
        printf("         \n"); fflush(stdout);*/
      }
    }
    GAStep(ga);
  } while (GAGetCurEpoch(ga) < 1000 && best > PBMATH_EPSILON);
  
  /*printf("\n"); fflush(stdout);
  VecFloatPrint(GAAdnAdnF(GABestAdn(ga)), stdout, 6);
  printf(" %f\n", best); fflush(stdout);*/
  res = PixelToPosEstimatorCameraPolarToRealPos(that, (VecFloat2D*)GAAdnAdnF(GABestAdn(ga)));

  GenAlgFree(&ga);

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
  const int nbBase = 4;
  #define NBDATA 7
  VecFloat3D basesPosMeter[NBDATA];
  VecFloat2D basesPosPixel[NBDATA];
  for (int iBase = nbBase; iBase--;) {
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
      "Test1", "Test2", "Test3"};
  // Approximated position of the POV
  float a = (VecGet(&imgSize, 0) * 0.5 - VecGet(basesPosPixel + 3, 0)) /
    (VecGet(basesPosPixel + 1, 0) - VecGet(basesPosPixel + 3, 0));
  VecFloat3D DB = 
    VecGetOp(basesPosMeter + 1, 1.0, basesPosMeter + 3, -1.0);
  VecFloat3D povPos = VecGetOp(basesPosMeter + 3, 1.0, &DB, a);
  // Create the estimator
  PixelToPosEstimator estimator = 
    PixelToPosEstimatorCreateStatic(&posCamera, &imgSize, &povPos);
  // Display the input values
  printf("Camera(m): "); VecPrint(&posCamera, stdout); printf("\n");
  for (int iBase = 0; iBase < nbBase; ++iBase) {
    printf("%s (m): ", baseName[iBase]); 
    VecPrint(basesPosMeter + iBase, stdout); 
    printf(" (px): "); 
    VecPrint(basesPosPixel + iBase, stdout); 
    printf("\n");
  }
  printf("POV(m): "); VecPrint(&povPos, stdout); printf("\n");
  // Calculate the projection parameters
  printf("Calculate the projection param...\n");

  /*GSet posMeter = GSetCreateStatic();
  GSet posPixel = GSetCreateStatic();
  for (int iBase = NBDATA; iBase--;) {
    GSetAppend(&posMeter, basesPosMeter + iBase);
    GSetAppend(&posPixel, basesPosPixel + iBase);
  }
  PixelToPosEstimatorInit(&estimator, &posMeter, &posPixel);*/
  
  float param[6] = {0.785406,80.543961,10.155653,6.049213,624.139771,1233.761353};
  for (int iParam = 6; iParam--;) {
    VecSet(estimator._param, iParam, param[iParam]);
  }
  
  printf("Projection param: ");
  VecPrint(estimator._param, stdout);printf("\n");
  
  for (int iBase = 0; iBase < nbBase; ++iBase) {
    printf("%s (polar): ", baseName[iBase]); 
    VecFloat2D polarPos = PixelToPosEstimatorRealPosToCameraPolar(
      &estimator, basesPosMeter + iBase);
    VecPrint(&polarPos, stdout); 
    printf("\n");
    VecFloat3D realPos = PixelToPosEstimatorCameraPolarToRealPos(
      &estimator, &polarPos);
    printf(" (real): "); 
    VecPrint(&realPos, stdout); 
    float errorReal = VecDist(basesPosMeter + iBase, &realPos);
    printf(" (error): %fm", errorReal); 
    printf("\n");
    VecFloat2D screenPos = PixelToPosEstimatorCameraPolarToScreenPos(
      &estimator, &polarPos);
    printf(" (screen): "); 
    VecPrint(&screenPos, stdout); 
    float errorScreenDist = 
      VecDist(basesPosPixel + iBase, &screenPos);
    printf(" (error): %fpx", errorScreenDist); 
    printf("\n");
  }
  for (int iBase = 0; iBase < nbBase; ++iBase) {
    printf("%s (screen->real): ", baseName[iBase]); 
    VecFloat3D estimPos = PixelToPosEstimatorGetPosPxToMeter(
      &estimator, basesPosPixel + iBase);
    VecPrint(&estimPos, stdout);
    float error = VecDist(&estimPos, basesPosMeter + iBase);
    printf(" (error): %fm", error); 
    printf("\n");
  }

  /*VecFloat3D v = VecFloatCreateStatic3D();
  for (int z = -10; z <= 30; z += 1) {
    VecSet(&v, 2, z);
    VecFloat2D polarPos = PixelToPosEstimatorRealPosToCameraPolar(
      &estimator, &v);
    VecFloat3D estimPos = PixelToPosEstimatorGetPosPxToMeter(
      &estimator, );
    float error = VecDist(&estimPos, v);
    printf("%f %f\n", z, error);
  }*/
  
  // Free memory
  PixelToPosEstimatorFreeStatic(&estimator);
  
  // Return success code
  return 0;
}

