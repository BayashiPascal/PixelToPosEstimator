// ============ PIXELTOPOSESTIMATOR.H ================

#ifndef PIXELTOPOSESTIMATOR_H
#define PIXELTOPOSESTIMATOR_H

// ================= Include =================

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "pberr.h"
#include "pbmath.h"
#include "gset.h"
#include "genalg.h"

// ================= Define ==================

#define PTPE_Px(that) VecGet(that->_param, 0)
#define PTPE_Py(that) VecGet(that->_param, 1)
#define PTPE_Sx(that) VecGet(that->_param, 2)
#define PTPE_Sy(that) VecGet(that->_param, 3)
#define PTPE_Upx(that) VecGet(that->_param, 4)
#define PTPE_Upy(that) VecGet(that->_param, 5)
#define PTPE_Upz(that) VecGet(that->_param, 6)

#define PTPE_NBPARAM 7

// ------------- PixelToPosEstimator

// ================= Data structure ===================

typedef struct PixelToPosEstimator {
  // Camera position
  VecFloat3D _cameraPos;
  // Dimension of the image
  VecFloat2D _imgSize;
  // Projection parameters
  // (Px, Py, Sx, Sy, Upx, Upy, Upz)
  VecFloat* _param;
} PixelToPosEstimator;

// ================ Functions declaration ====================

// Create a new PixelToPosEstimator
PixelToPosEstimator PixelToPosEstimatorCreateStatic(
  VecFloat3D* posCamera, const VecFloat2D* const imgSize);

// Free memory used by the PixelToPosEstimator 'that'
void PixelToPosEstimatorFreeStatic(PixelToPosEstimator* that);

// Convert the screen position to a polar position
VecFloat2D PTPEGetPxToPolar(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const screenPos);

// Calculate the projection parameter using genetic algorithm for
// 'nbEpoch' eochs or until the average error gets below 'prec'
// the random generator must be initialized before calling this function
void PTPEInit(PixelToPosEstimator* const that,
  const GSet* const posMeter, const GSet* const posPixel, 
  const unsigned int nbEpoch, const float prec);

// Convert the screen position to a real position
VecFloat3D PTPEGetPxToMeter(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const screenPos);

// Convert the polar position to a real position
VecFloat3D PTPEGetPolarToMeter(
  const PixelToPosEstimator* const that, 
  const VecFloat2D* const polarPos);
  
#endif
