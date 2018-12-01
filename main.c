#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <tgmath.h>
#include "pixeltoposestimator.h"


int main(int argc, char** argv) {
  (void)argc; (void)argv;

  srandom(time(NULL));

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
  // Variable to memorize the name of the positions
  const char* baseName[NBDATA] = 
    {"Home base", "1st base", "2nd base", "3rd base",
      "Test1", "Test2", "Test3", "Test4"};
  // Create the estimator
  PixelToPosEstimator estimator = 
    PixelToPosEstimatorCreateStatic(&posCamera, &imgSize);

  // Calculate the projection parameters
  FILE* fileParam = fopen("./param.txt", "r");
  if (fileParam == NULL) {
    printf("Calculate the projection param...\n");
    GSet posMeter = GSetCreateStatic();
    GSet posPixel = GSetCreateStatic();
    for (int iBase = 0; iBase < NBINPUT; ++iBase) {
      GSetAppend(&posMeter, basesPosMeter + iBase);
      GSetAppend(&posPixel, basesPosPixel + iBase);
    }
    unsigned int nbEpoch = 100000;
    float prec = 0.01;
    PTPEInit(&estimator, &posMeter, &posPixel, nbEpoch, prec);
    GSetFlush(&posMeter);
    GSetFlush(&posPixel);
    fileParam = fopen("./param.txt", "w");
    if (!VecSave(estimator._param, fileParam, true)) {
      fprintf(stderr, "Failed to save the parameters\n");
      exit(0);
    }
  } else {
    printf("Reuse the projection param...\n");
    if (!VecLoad(&(estimator._param), fileParam)) {
      fprintf(stderr, "Failed to load the parameters\n");
      exit(0);
    }
  }
  fclose(fileParam);
  
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
    VecFloat2D polarPos = PTPEGetPxToPolar(
      &estimator, basesPosPixel + iBase);
    VecPrint(&polarPos, stdout); 
    printf("\n");
    VecFloat3D realPos = PTPEGetPolarToMeter(
      &estimator, &polarPos);
    printf(" (real): "); 
    VecPrint(&realPos, stdout); 
    float errorReal = VecDist(basesPosMeter + iBase, &realPos);
    printf(" (error): %fm", errorReal); 
    printf("\n");
    printf(" (screen->real): "); 
    VecFloat3D estimPos = PTPEGetPxToMeter(
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

