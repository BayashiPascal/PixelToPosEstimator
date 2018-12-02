#include "pixeltoposestimator.h"

// Create a new PixelToPosEstimator
PixelToPosEstimator PixelToPosEstimatorCreateStatic(
  VecFloat3D* posCamera, const VecFloat2D* const imgSize) {
#if BUILDMODE == 0
  if (posCamera == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'posCamera' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
  if (imgSize == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'imgSize' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
#endif
  // Declare the new estimator
  PixelToPosEstimator estimator;
  // Init the estimator
  estimator._cameraPos = *posCamera;
  estimator._imgSize = *imgSize;
  estimator._param = VecFloatCreate(PTPE_NBPARAM);
  // Return the new estimator
  return estimator;
}

// Free memory used by the PixelToPosEstimator 'that'
void PixelToPosEstimatorFreeStatic(PixelToPosEstimator* const that) {
  if (that == NULL)
    return;
  VecFree(&(that->_param));
}

// Convert the polar position to a real position
VecFloat3D PTPEGetPolarToMeter(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const polarPos) {
#if BUILDMODE == 0
  if (that == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'that' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
  if (polarPos == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'polarPos' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
#endif
  // Declare a variable to memorize the result
  VecFloat3D res = VecFloatCreateStatic3D();
  // Calculate the real coordinates

  // POV
  VecFloat3D P = VecFloatCreateStatic3D();
  VecSet(&P, 0, PTPE_Px(that));
  VecSet(&P, 1, PTPE_Py(that));
  VecSet(&P, 2, PTPE_Pz(that));
  // Normalized vector Camera->POV
  VecFloat3D CP = VecGetOp(&P, 1.0, &(that->_cameraPos), -1.0);
  VecNormalise(&CP);
  // Normalized up vector
  VecFloat3D Up = VecFloatCreateStatic3D();
  VecSet(&Up, 0, PTPE_Upx(that));
  VecSet(&Up, 1, PTPE_Upy(that));
  VecSet(&Up, 2, PTPE_Upz(that));
  VecNormalise(&Up);
  // Normalized right vector
  VecFloat3D Right = VecCrossProd(&CP, &Up);
  VecNormalise(&Right);
  // Rotation according to up and right
  VecFloat3D Rx = CP;
  VecRotAxis(&Rx, &Up, PTPE_Sx(that) * VecGet(polarPos, 0));
  Rx = VecGetOp(&Rx, 1.0, &CP, -1.0);
  VecFloat3D Ry = CP;
  VecRotAxis(&Ry, &Right, PTPE_Sy(that) * VecGet(polarPos, 1));
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

// Convert the screen position to a polar position
VecFloat2D PTPEGetPxToPolar(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const screenPos) {
#if BUILDMODE == 0
  if (that == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'that' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
  if (screenPos == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'screenPos' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
#endif
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

// Calculate the projection parameter using genetic algorithm for
// 'nbEpoch' epochs or until the average error gets below 'prec'
// Search for the parameters Px, Py, Pz in the bounding box defined
// by POVmin-POVmax
// the random generator must be initialized before calling this function
void PTPEInit(PixelToPosEstimator* const that,
  const GSet* const posMeter, const GSet* const posPixel, 
  const unsigned int nbEpoch, const float prec,
  const VecFloat3D* const POVmin, const VecFloat3D* const POVmax) {
#if BUILDMODE == 0
  if (that == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'that' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
  if (posMeter == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'posMeter' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
  if (posPixel == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'posPixel' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
  if (GSetNbElem(posPixel) != GSetNbElem(posMeter)) {
    ELORankErr->_type = PBErrTypeInvalidArg;
    sprintf(ELORankErr->_msg, 
      "'posPixel' and 'posMeter' don't have same sizes (%ld==%ld)",
      GSetNbElem(posPixel), GSetNbElem(posMeter));
    PBErrCatch(PixelToPosEstimatorErr);
  }
  if (POVmin == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'POVmin' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
  if (POVmax == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'POVmax' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
#endif
  // Create the GenAlg
  int lengthAdnF = PTPE_NBPARAM;
  int lengthAdnI = 0;
  GenAlg* ga = GenAlgCreate(GENALG_NBENTITIES, GENALG_NBELITES, 
    lengthAdnF, lengthAdnI);
  // Set the boundaries for the parameters
  VecFloat2D boundsF = VecFloatCreateStatic2D();
  VecSet(&boundsF, 0, VecGet(POVmin, 0)); 
  VecSet(&boundsF, 1, VecGet(POVmax, 0));
  GASetBoundsAdnFloat(ga, 0, &boundsF); // Px
  VecSet(&boundsF, 0, VecGet(POVmin, 1)); 
  VecSet(&boundsF, 1, VecGet(POVmax, 1));
  GASetBoundsAdnFloat(ga, 1, &boundsF); // Py
  VecSet(&boundsF, 0, VecGet(POVmin, 2)); 
  VecSet(&boundsF, 1, VecGet(POVmax, 2));
  GASetBoundsAdnFloat(ga, 2, &boundsF); // Pz
  VecSet(&boundsF, 0, -PBMATH_HALFPI); 
  VecSet(&boundsF, 1, PBMATH_HALFPI);
  GASetBoundsAdnFloat(ga, 3, &boundsF); // Sx
  GASetBoundsAdnFloat(ga, 4, &boundsF); // Sy
  VecSet(&boundsF, 0, -1.0); VecSet(&boundsF, 1, 1.0);
  GASetBoundsAdnFloat(ga, 5, &boundsF); // Upx
  VecSet(&boundsF, 0, 0.0); VecSet(&boundsF, 1, 1.0);
  GASetBoundsAdnFloat(ga, 6, &boundsF); // Upy
  VecSet(&boundsF, 0, -1.0); VecSet(&boundsF, 1, 1.0);
  GASetBoundsAdnFloat(ga, 7, &boundsF); // Upz
  // Init the GenAlg
  GAInit(ga);
  // Variable to memorize the current best adn value
  float best = 10000.0;
  // Loop on epochs
  do {
    //printf("epoch %ld avg err %fm     \r", 
    //  GAGetCurEpoch(ga), best / (float)GSetNbElem(posMeter));
    //fflush(stdout);
    // Variable to memorize the evaluation of one base
    float ev = 0.0;
    // Loop on adns
    for (int iEnt = 0; iEnt < GAGetNbAdns(ga); ++iEnt) {
      // Copy the adn into the estimator's parameters
      VecCopy(that->_param, GAAdnAdnF(GAAdn(ga, iEnt)));
      // Reset the evaluation variable
      ev = 0.0;
      // Loop on both sets
      GSetIterForward iterMeter = GSetIterForwardCreateStatic(posMeter);
      GSetIterForward iterPixel = GSetIterForwardCreateStatic(posPixel);
      do {
        // Get the screen position
        VecFloat2D* pPixel = GSetIterGet(&iterPixel);
        // Convert to polar position
        VecFloat2D polarPos = PTPEGetPxToPolar(that, pPixel);
        // Convert to real position
        VecFloat3D pEstim = PTPEGetPolarToMeter(that, &polarPos);
        // Get the correct real position
        VecFloat3D* pMeter = GSetIterGet(&iterMeter);
        // Calculate the error
        ev += VecDist(pMeter, &pEstim);
      } while (GSetIterStep(&iterMeter) && GSetIterStep(&iterPixel));
      // Calculate the average error
      ev /=  (float)GSetNbElem(posMeter);
      // Update the value of this adn
      GASetAdnValue(ga, GAAdn(ga, iEnt), -1.0 * ev);
      // Update the best value if necessary
      if (ev < best - PBMATH_EPSILON) {
        best = ev;
        printf("%lu %f ", GAGetCurEpoch(ga), best);
        VecFloatPrint(that->_param, stdout, 6);
        printf("        \n"); fflush(stdout);
      }
    }
    // Step the GenAlg
    GAStep(ga);
  } while (GAGetCurEpoch(ga) < nbEpoch && best > prec);
  // Copy the final best adn into the estimator's parameters
  VecCopy(that->_param, GAAdnAdnF(GABestAdn(ga)));
  // Free memory
  GenAlgFree(&ga);
}

// Convert the screen position to a real position
VecFloat3D PTPEGetPxToMeter(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const screenPos) {
#if BUILDMODE == 0
  if (that == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'that' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
  if (screenPos == NULL) {
    ELORankErr->_type = PBErrTypeNullPointer;
    sprintf(ELORankErr->_msg, "'screenPos' is null");
    PBErrCatch(PixelToPosEstimatorErr);
  }
#endif
  // Declare a variable to memorize the result
  VecFloat3D res = VecFloatCreateStatic3D();
  // Calculate the real coordinates
  VecFloat2D polarPos = PTPEGetPxToPolar(that, screenPos);
  res = PTPEGetPolarToMeter(that, &polarPos);
  // Return the result
  return res;
}

