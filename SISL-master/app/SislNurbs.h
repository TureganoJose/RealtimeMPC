// MathLibrary.h - Contains declarations of math functions
#pragma once

#include "sisl.h"


//#ifdef SISLNURBS_EXPORTS
 
//#else
//#define SISLNURBS_API __declspec(dllimport)
//#endif

// The Fibonacci recurrence relation describes a sequence F
// where F(n) is { n = 0, a
//               { n = 1, b
//               { n > 1, F(n-2) + F(n-1)
// for some initial integral values a and b.
// If the sequence is initialized F(0) = 1, F(1) = 1,
// then this relation produces the well-known Fibonacci
// sequence: 1, 1, 2, 3, 5, 8, 13, 21, 34, ...

// Initialize a Fibonacci relation sequence
// such that F(0) = a, F(1) = b.
// This function must be called before any other function.
#ifdef __cplusplus
extern "C" {
#endif
	__declspec(dllexport) SISLCurve* createNURBS(double epoint[], int nptyp[], int num_points);
#ifdef __cplusplus
}
#endif


// Produce the next value in the sequence.
// Returns true on success and updates current value and index;
// false on overflow, leaves current value and index unchanged.
#ifdef __cplusplus
extern "C" {
#endif
	__declspec(dllexport) double* interrogateNURBS(SISLCurve *Curve, double param, double position[4]);
#ifdef __cplusplus
}
#endif
// Get the current value in the sequence.
#ifdef __cplusplus
extern "C" {
#endif
	__declspec(dllexport) double* closestpoint(SISLCurve *Curve, double point[2], double output[2]);
#ifdef __cplusplus
}
#endif
// Get the position of the current value in the sequence.
#ifdef __cplusplus
extern "C" {
#endif
	__declspec(dllexport) double CalculateDerivate(SISLCurve *Curve, double param);
#ifdef __cplusplus
}
#endif

// Get the position of the current value in the sequence.
#ifdef __cplusplus
extern "C" {
#endif
	__declspec(dllexport) double CalculateCurvature(SISLCurve *Curve, double param);
#ifdef __cplusplus
}
#endif


// Get the position of the current value in the sequence.
#ifdef __cplusplus
extern "C" {
#endif
	__declspec(dllexport) void freeNURBS(SISLCurve *Curve);
#ifdef __cplusplus
}
#endif