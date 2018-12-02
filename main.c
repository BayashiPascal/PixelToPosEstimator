#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <tgmath.h>
#include "pixeltoposestimator.h"


int main(int argc, char** argv) {
  (void)argc; (void)argv;

  srandom(time(NULL));

  // Read the data from the file in argument
  if (argc != 2) {
    fprintf(stderr, "Usage: main <input file>\n");
    exit(0);
  }
  FILE* inputFile = fopen(argv[1], "r");
  if (inputFile == NULL) {
    fprintf(stderr, "Can't open %s\n", argv[0]);
    exit(0);
  }
  VecFloat* posCamera = NULL;
  VecFloat* imgSize = NULL;
  VecLoad(&posCamera, inputFile);
  VecLoad(&imgSize, inputFile);
  VecFloat* POVmin = NULL;
  VecFloat* POVmax = NULL;
  VecLoad(&POVmin, inputFile);
  VecLoad(&POVmax, inputFile);
  int nbInput = 0;
  int ret = fscanf(inputFile, "%d", &nbInput);
  GSet inputMeter = GSetCreateStatic();
  GSet inputPixel = GSetCreateStatic();
  for (int iInput = 0; iInput < nbInput; ++iInput) {
    VecFloat* posMeter = NULL;
    VecFloat* posPixel = NULL;
    VecLoad(&posMeter, inputFile);
    VecLoad(&posPixel, inputFile);
    GSetAppend(&inputMeter, posMeter);
    GSetAppend(&inputPixel, posPixel);
  }
  int nbTest = 0;
  ret = fscanf(inputFile, "%d", &nbTest);
  (void)ret;
  GSet testMeter = GSetCreateStatic();
  GSet testPixel = GSetCreateStatic();
  for (int iInput = 0; iInput < nbTest; ++iInput) {
    VecFloat* posMeter = NULL;
    VecFloat* posPixel = NULL;
    VecLoad(&posMeter, inputFile);
    VecLoad(&posPixel, inputFile);
    GSetAppend(&testMeter, posMeter);
    GSetAppend(&testPixel, posPixel);
  }
  fclose(inputFile);

  // Create the estimator
  PixelToPosEstimator estimator = PixelToPosEstimatorCreateStatic(
    (VecFloat3D*)posCamera, (VecFloat2D*)imgSize);

  // Calculate the projection parameters
  FILE* fileParam = fopen("./param.txt", "r");
  if (fileParam == NULL) {
    printf("Calculate the projection param...\n");
    unsigned int nbEpoch = 500000;
    float prec = 0.001;
    PTPEInit(&estimator, &inputMeter, &inputPixel, 
      nbEpoch, prec, (VecFloat3D*)POVmin, (VecFloat3D*)POVmax);
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

  printf("Input data:\n\n");
  float avgErr = 0.0;
  float maxErr = 0.0;
  for (int iInput = 0; iInput < nbInput; ++iInput) {
    VecFloat3D* posMeter = (VecFloat3D*)GSetGet(&inputMeter, iInput);
    VecFloat2D* posPixel = (VecFloat2D*)GSetGet(&inputPixel, iInput);
    printf("input #%d (m): ", iInput); 
    VecPrint(posMeter, stdout); 
    printf(" (px): "); 
    VecPrint(posPixel, stdout); 
    printf("\n");
    printf(" (screen->real): "); 
    VecFloat3D estimPos = PTPEGetPxToMeter(&estimator, posPixel);
    VecPrint(&estimPos, stdout);
    float error = VecDist(&estimPos, posMeter);
    printf(" (error): %fm", error); 
    printf("\n\n");
    avgErr += error;
    if (maxErr < error)
      maxErr = error;
  }
  avgErr /= (float)nbInput;
  printf("Average error: %fm\n", avgErr); 
  printf("Max error: %fm\n\n", maxErr); 
  
  printf("Test data:\n\n");
  avgErr = 0.0;
  maxErr = 0.0;
  for (int iInput = 0; iInput < nbTest; ++iInput) {
    VecFloat3D* posMeter = (VecFloat3D*)GSetGet(&testMeter, iInput);
    VecFloat2D* posPixel = (VecFloat2D*)GSetGet(&testPixel, iInput);
    printf("input #%d (m): ", iInput); 
    VecPrint(posMeter, stdout); 
    printf(" (px): "); 
    VecPrint(posPixel, stdout); 
    printf("\n");
    printf(" (screen->real): "); 
    VecFloat3D estimPos = PTPEGetPxToMeter(&estimator, posPixel);
    VecPrint(&estimPos, stdout);
    float error = VecDist(&estimPos, posMeter);
    printf(" (error): %fm", error); 
    printf("\n\n");
    avgErr += error;
    if (maxErr < error)
      maxErr = error;
  }
  avgErr /= (float)nbTest;
  printf("Average error: %fm\n", avgErr); 
  printf("Max error: %fm\n", maxErr); 
  
  // Free memory
  PixelToPosEstimatorFreeStatic(&estimator);
  while (GSetNbElem(&inputMeter) > 0) {
    VecFloat* v = GSetPop(&inputMeter);
    VecFree(&v);
  }
  while (GSetNbElem(&inputPixel) > 0) {
    VecFloat* v = GSetPop(&inputPixel);
    VecFree(&v);
  }
  while (GSetNbElem(&testMeter) > 0) {
    VecFloat* v = GSetPop(&testMeter);
    VecFree(&v);
  }
  while (GSetNbElem(&testPixel) > 0) {
    VecFloat* v = GSetPop(&testPixel);
    VecFree(&v);
  }
  
  // Return success code
  return 0;
}

