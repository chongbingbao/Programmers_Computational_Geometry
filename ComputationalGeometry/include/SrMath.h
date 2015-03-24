/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/11/22
****************************************************************************/
#ifndef SR_FOUNDATION_MATH_H_
#define SR_FOUNDATION_MATH_H_

/** \addtogroup foundation
  @{
*/

#include <math.h>
#include <float.h>
#include <stdlib.h>
#include "SrSimpleTypes.h"

#ifdef log2
#undef log2
#endif 
#ifdef min
#undef min
#endif 
#ifdef max
#undef max
#endif 
//constants
static const SrF64 SrPiF64		= 3.141592653589793;
static const SrF64 SrHalfPiF64	= 1.57079632679489661923;
static const SrF64 SrTwoPiF64	= 6.28318530717958647692;
static const SrF64 SrInvPiF64	= 0.31830988618379067154;
//we can get bad range checks if we use double prec consts to check single prec results.
static const SrF32 SrPiF32		= 3.141592653589793f;
static const SrF32 SrHalfPiF32	= 1.57079632679489661923f;
static const SrF32 SrTwoPiF32	= 6.28318530717958647692f;
static const SrF32 SrInvPiF32	= 0.31830988618379067154f;

/**
\brief Static class with stateless scalar math routines.
*/
class SrMath
	{
	public:

// Type conversion and rounding
		/**
		\brief Returns true if the two numbers are within eps of each other.
		*/
		SR_INLINE static bool equals(SrF32,SrF32,SrF32 eps);
		/**
		\brief Returns true if the two numbers are within eps of each other.
		*/
		SR_INLINE static bool equals(SrF64,SrF64,SrF64 eps);
		/**
		\brief The floor function returns a floating-point value representing the largest integer that is less than or equal to x.
		*/
		SR_INLINE static SrF32 floor(SrF32);
		/**
		\brief The floor function returns a floating-point value representing the largest integer that is less than or equal to x.
		*/
		SR_INLINE static SrF64 floor(SrF64);


		/**
		\brief The ceil function returns a single value representing the smallest integer that is greater than or equal to x. 
		*/
		SR_INLINE static SrF32 ceil(SrF32);
		/**
		\brief The ceil function returns a double value representing the smallest integer that is greater than or equal to x. 
		*/
		SR_INLINE static SrF64 ceil(SrF64);

		/**
		\brief Truncates the float to an integer.
		*/
		SR_INLINE static SrI32 trunc(SrF32);
		/**
		\brief Truncates the double precision float to an integer.
		*/
		SR_INLINE static SrI32 trunc(SrF64);
		/**
		\brief abs returns the absolute value of its argument. 
		*/
		SR_INLINE static SrF32 abs(SrF32);
		/**
		\brief abs returns the absolute value of its argument. 
		*/
		SR_INLINE static SrF64 abs(SrF64);
		/**
		\brief abs returns the absolute value of its argument. 
		*/
		SR_INLINE static SrI32 abs(SrI32);
		/**
		\brief sign returns the sign of its argument. The sign of zero is undefined.
		*/
		SR_INLINE static SrF32 sign(SrF32);
		/**
		\brief sign returns the sign of its argument. The sign of zero is undefined.
		*/
		SR_INLINE static SrF64 sign(SrF64);
		/**
		\brief sign returns the sign of its argument. The sign of zero is undefined.
		*/
		SR_INLINE static SrI32 sign(SrI32);
		/**
		\brief The return value is the greater of the two specified values. 
		*/
		SR_INLINE static SrF32 max(SrF32,SrF32);
		/**
		\brief The return value is the greater of the two specified values. 
		*/
		SR_INLINE static SrF64 max(SrF64,SrF64);
		/**
		\brief The return value is the greater of the two specified values. 
		*/
		SR_INLINE static SrI32 max(SrI32,SrI32);
		/**
		\brief The return value is the greater of the two specified values. 
		*/
		SR_INLINE static SrU32 max(SrU32,SrU32);
		/**
		\brief The return value is the greater of the two specified values. 
		*/
		SR_INLINE static SrU16 max(SrU16,SrU16);
		/**
		\brief The return value is the greater of the two specified values. 
		*/
		SR_INLINE static SrI16 max(SrI16,SrI16);
		/**
		\brief The return value is the lesser of the two specified values. 
		*/
		SR_INLINE static SrF32 min(SrF32,SrF32);
		/**
		\brief The return value is the lesser of the two specified values. 
		*/
		SR_INLINE static SrF64 min(SrF64,SrF64);
		/**
		\brief The return value is the lesser of the two specified values. 
		*/
		SR_INLINE static SrI32 min(SrI32,SrI32);
		/**
		\brief The return value is the lesser of the two specified values. 
		*/
		SR_INLINE static SrU32 min(SrU32,SrU32);
		/**
		\brief The return value is the lesser of the two specified values. 
		*/
		SR_INLINE static SrU16 min(SrU16,SrU16);
		/**
		\brief The return value is the lesser of the two specified values. 
		*/
		SR_INLINE static SrI16 min(SrI16,SrI16);
		
		/**
		\brief mod returns the floating-point remainder of x / y. 
		
		If the value of y is 0.0, mod returns a quiet NaN.
		*/
		SR_INLINE static SrF32 mod(SrF32 x, SrF32 y);
		/**
		\brief mod returns the floating-point remainder of x / y. 
		
		If the value of y is 0.0, mod returns a quiet NaN.
		*/
		SR_INLINE static SrF64 mod(SrF64 x, SrF64 y);

		/**
		\brief Clamps v to the range [hi,lo]
		*/
		SR_INLINE static SrF32 clamp(SrF32 v, SrF32 hi, SrF32 low);
		/**
		\brief Clamps v to the range [hi,lo]
		*/
		SR_INLINE static SrF64 clamp(SrF64 v, SrF64 hi, SrF64 low);
		/**
		\brief Clamps v to the range [hi,lo]
		*/
		SR_INLINE static SrU32 clamp(SrU32 v, SrU32 hi, SrU32 low);
		/**
		\brief Clamps v to the range [hi,lo]
		*/
		SR_INLINE static SrI32 clamp(SrI32 v, SrI32 hi, SrI32 low);

		//!powers
		/**
		\brief Square root.
		*/
		SR_INLINE static SrF32 sqrt(SrF32);
		/**
		\brief Square root.
		*/
		SR_INLINE static SrF64 sqrt(SrF64);
		
		/**
		\brief reciprocal square root.
		*/
		SR_INLINE static SrF32 recipSqrt(SrF32);
		/**
		\brief reciprocal square root.
		*/
		SR_INLINE static SrF64 recipSqrt(SrF64);
		
		/**
		\brief Calculates x raised to the power of y.
		*/
		SR_INLINE static SrF32 pow(SrF32 x, SrF32 y);
		/**
		\brief Calculates x raised to the power of y.
		*/
		SR_INLINE static SrF64 pow(SrF64 x, SrF64 y);
		
		
		/**
		\brief Calculates e^n
		*/
		SR_INLINE static SrF32 exp(SrF32);
		/**
		\brief Calculates e^n
		*/
		SR_INLINE static SrF64 exp(SrF64);
		
		/**
		\brief Calculates logarithms.
		*/
		SR_INLINE static SrF32 logE(SrF32);
		/**
		\brief Calculates logarithms.
		*/
		SR_INLINE static SrF64 logE(SrF64);
		/**
		\brief Calculates logarithms.
		*/
		SR_INLINE static SrF32 log2(SrF32);
		/**
		\brief Calculates logarithms.
		*/
		SR_INLINE static SrF64 log2(SrF64);
		/**
		\brief Calculates logarithms.
		*/
		SR_INLINE static SrF32 log10(SrF32);
		/**
		\brief Calculates logarithms.
		*/
		SR_INLINE static SrF64 log10(SrF64);

		//!trigonometry -- all angles are in radians.
		
		/**
		\brief Converts degrees to radians.
		*/
		SR_INLINE static SrF32 degToRad(SrF32);
		/**
		\brief Converts degrees to radians.
		*/
		SR_INLINE static SrF64 degToRad(SrF64);

		/**
		\brief Converts radians to degrees.
		*/
		SR_INLINE static SrF32 radToDeg(SrF32);
		/**
		\brief Converts radians to degrees.
		*/
		SR_INLINE static SrF64 radToDeg(SrF64);

		/**
		\brief Sine of an angle.

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF32 sin(SrF32);
		/**
		\brief Sine of an angle.

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF64 sin(SrF64);
		
		/**
		\brief Cosine of an angle.

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF32 cos(SrF32);
		/**
		\brief Cosine of an angle.

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF64 cos(SrF64);

		/**
		\brief Computes both the sin and cos.

		<b>Unit:</b> Radians
		*/
		SR_INLINE static void sinCos(SrF32, SrF32 & sin, SrF32 & cos);

		/**
		\brief Computes both the sin and cos.

		<b>Unit:</b> Radians
		*/
		SR_INLINE static void sinCos(SrF64, SrF64 & sin, SrF64 & cos);


		/**
		\brief Tangent of an angle.

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF32 tan(SrF32);
		/**
		\brief Tangent of an angle.
		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF64 tan(SrF64);
		
		/**
		\brief Arcsine.
		
		Returns angle between -PI/2 and PI/2 in radians

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF32 asin(SrF32);
		/**
		\brief Arcsine.
		
		Returns angle between -PI/2 and PI/2 in radians

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF64 asin(SrF64);
		
		/**
		\brief Arccosine.
		
		Returns angle between 0 and PI in radians

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF32 acos(SrF32);
		
		/**
		\brief Arccosine.
		
		Returns angle between 0 and PI in radians

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF64 acos(SrF64);
		
		/**
		\brief ArcTangent.
		
		Returns angle between -PI/2 and PI/2 in radians

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF32 atan(SrF32);
		/**
		\brief ArcTangent.
		
		Returns angle between -PI/2 and PI/2 in radians

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF64 atan(SrF64);

		/**
		\brief Arctangent of (x/y) with correct sign.
		
		Returns angle between -PI and PI in radians

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF32 atan2(SrF32 x, SrF32 y);
		/**
		\brief Arctangent of (x/y) with correct sign.
		
		Returns angle between -PI and PI in radians

		<b>Unit:</b> Radians
		*/
		SR_INLINE static SrF64 atan2(SrF64 x, SrF64 y);

		//random numbers
		
		/**
		\brief uniform random number in [a,b]
		*/
		SR_INLINE static SrF32 rand(SrF32 a,SrF32 b);
		
		/**
		\brief uniform random number in [a,b]
		*/
		SR_INLINE static SrI32 rand(SrI32 a,SrI32 b);

		/**
		\brief hashing: hashes an array of n 32 bit values	to a 32 bit value.
		
		Because the output bits are uniformly distributed, the caller may mask
		off some of the bits to index into a hash table	smaller than 2^32.
		*/
		SR_INLINE static SrU32 hash(const SrU32 * array, SrU32 n);

		/**
		\brief hash32
		*/
		SR_INLINE static int hash32(int);

		/**
		\brief returns true if the passed number is a finite floating point number as opposed to INF, NAN, etc.
		*/
		SR_INLINE static bool isFinite(SrF32 x);
		
		/**
		\brief returns true if the passed number is a finite floating point number as opposed to INF, NAN, etc.
		*/
		SR_INLINE static bool isFinite(SrF64 x);
	};

/*
Many of these are just implemented as calls to the C lib right now,
but later we could replace some of them with some approximations or more
clever stuff.
*/
SR_INLINE bool SrMath::equals(SrF32 a,SrF32 b,SrF32 eps)
	{
	const SrF32 diff = SrMath::abs(a - b);
	return (diff < eps);
	}

SR_INLINE bool SrMath::equals(SrF64 a,SrF64 b,SrF64 eps)
	{
	const SrF64 diff = SrMath::abs(a - b);
	return (diff < eps);
	}

SR_INLINE SrF32 SrMath::floor(SrF32 a)
	{
	return ::floorf(a);
	}

SR_INLINE SrF64 SrMath::floor(SrF64 a)
	{
	return ::floor(a);
	}

SR_INLINE SrF32 SrMath::ceil(SrF32 a)
	{
	return ::ceilf(a);
	}

SR_INLINE SrF64 SrMath::ceil(SrF64 a)
	{
	return ::ceil(a);
	}

SR_INLINE SrI32 SrMath::trunc(SrF32 a)
	{
	return (SrI32) a;	// ### PT: this actually depends on FPU settings
	}

SR_INLINE SrI32 SrMath::trunc(SrF64 a)
	{
	return (SrI32) a;	// ### PT: this actually depends on FPU settings
	}

SR_INLINE SrF32 SrMath::abs(SrF32 a)
	{
	return ::fabsf(a);
	}

SR_INLINE SrF64 SrMath::abs(SrF64 a)
	{
	return ::fabs(a);
	}

SrI32 SrMath::abs(SrI32 a)
	{
	return ::abs(a);
	}

SR_INLINE SrF32 SrMath::sign(SrF32 a)
	{
	return abs(a)<SR_EPS_F32?0:((a > 0.0f) ? 1.0f : -1.0f);
	}

SR_INLINE SrF64 SrMath::sign(SrF64 a)
	{
		return abs(a)<SR_EPS_F64?0.0:((a > 0.0) ? 1.0 : -1.0);
	}

SR_INLINE SrI32 SrMath::sign(SrI32 a)
	{
		return a==0?0:((a >= 0) ? 1 : -1);
	}

SR_INLINE bool SrMath::isFinite(SrF32 f)
	{
		return (0 == ((_FPCLASS_SNAN | _FPCLASS_QNAN | _FPCLASS_NINF | _FPCLASS_PINF) & _fpclass(f) ));
	}
		
SR_INLINE bool SrMath::isFinite(SrF64 f)
	{
		return (0 == ((_FPCLASS_SNAN | _FPCLASS_QNAN | _FPCLASS_NINF | _FPCLASS_PINF) & _fpclass(f) ));
	}
SR_INLINE SrF32 SrMath::max(SrF32 a,SrF32 b)
{
	return (a < b) ? b : a;
}
SR_INLINE SrF64 SrMath::max(SrF64 a,SrF64 b)
{
	return (a < b) ? b : a;
}
SR_INLINE SrI32 SrMath::max(SrI32 a,SrI32 b)
	{
	return (a < b) ? b : a;
	}

SR_INLINE SrU32 SrMath::max(SrU32 a,SrU32 b)
	{
	return (a < b) ? b : a;
	}
SR_INLINE SrI16 SrMath::max(SrI16 a,SrI16 b)
{
	return (a < b) ? b : a;
}
SR_INLINE SrU16 SrMath::max(SrU16 a,SrU16 b)
	{
	return (a < b) ? b : a;
	}

SR_INLINE SrF32 SrMath::min(SrF32 a,SrF32 b)
{
	return (a < b) ? a : b;
}

SR_INLINE SrF64 SrMath::min(SrF64 a,SrF64 b)
{
	return (a < b) ? a : b;
}
SR_INLINE SrI32 SrMath::min(SrI32 a,SrI32 b)
{
	return (a < b) ? a : b;
}
SR_INLINE SrU32 SrMath::min(SrU32 a,SrU32 b)
	{
	return (a < b) ? a : b;
	}

SR_INLINE SrI16 SrMath::min(SrI16 a,SrI16 b)
	{
	return (a < b) ? a : b;
	}
SR_INLINE SrU16 SrMath::min(SrU16 a,SrU16 b)
{
	return (a < b) ? a : b;
}

SR_INLINE SrF32 SrMath::mod(SrF32 x, SrF32 y)
	{
	return (SrF32)::fmod(x,y);
	}

SR_INLINE SrF64 SrMath::mod(SrF64 x, SrF64 y)
	{
	return ::fmod(x,y);
	}

SR_INLINE SrF32 SrMath::clamp(SrF32 v, SrF32 hi, SrF32 low)
	{
	if (v > hi) 
		return hi;
	else if (v < low) 
		return low;
	else
		return v;
	}

SR_INLINE SrF64 SrMath::clamp(SrF64 v, SrF64 hi, SrF64 low)
	{
	if (v > hi) 
		return hi;
	else if (v < low) 
		return low;
	else
		return v;
	}

SR_INLINE SrU32 SrMath::clamp(SrU32 v, SrU32 hi, SrU32 low)
	{
	if (v > hi) 
		return hi;
	else if (v < low) 
		return low;
	else
		return v;
	}

SR_INLINE SrI32 SrMath::clamp(SrI32 v, SrI32 hi, SrI32 low)
	{
	if (v > hi) 
		return hi;
	else if (v < low) 
		return low;
	else
		return v;
	}

SR_INLINE SrF32 SrMath::recipSqrt(SrF32 a)
	{
	return 1.0f/::sqrtf(a);
	}

SR_INLINE SrF64 SrMath::recipSqrt(SrF64 a)
	{
	return 1.0/::sqrt(a);
	}


SR_INLINE SrF32 SrMath::pow(SrF32 x, SrF32 y)
	{
	return ::powf(x,y);
	}

SrF64 SrMath::pow(SrF64 x, SrF64 y)
	{
	return ::pow(x,y);
	}

SR_INLINE SrF32 SrMath::exp(SrF32 a)
	{
	return ::expf(a);
	}

SR_INLINE SrF64 SrMath::exp(SrF64 a)
	{
	return ::exp(a);
	}

SR_INLINE SrF32 SrMath::logE(SrF32 a)
	{
	return ::logf(a);
	}

SR_INLINE SrF64 SrMath::logE(SrF64 a)
	{
	return ::log(a);
	}

SR_INLINE SrF32 SrMath::log2(SrF32 a)
	{
	const SrF32 ln2 = (SrF32)0.693147180559945309417;
    return ::logf(a) / ln2;
	}

SR_INLINE SrF64 SrMath::log2(SrF64 a)
	{
	const SrF64 ln2 = (SrF64)0.693147180559945309417;
    return ::log(a) / ln2;
	}

SR_INLINE SrF32 SrMath::log10(SrF32 a)
	{
	return (SrF32)::log10(a);
	}

SR_INLINE SrF64 SrMath::log10(SrF64 a)
	{
	return ::log10(a);
	}

SR_INLINE SrF32 SrMath::degToRad(SrF32 a)
	{
	return (SrF32)0.01745329251994329547 * a;
	}

SR_INLINE SrF64 SrMath::degToRad(SrF64 a)
	{
	return (SrF64)0.01745329251994329547 * a;
	}

SR_INLINE SrF32 SrMath::radToDeg(SrF32 a)
	{
	return (SrF32)57.29577951308232286465 * a;
	}

SR_INLINE SrF64 SrMath::radToDeg(SrF64 a)
	{
	return (SrF64)57.29577951308232286465 * a;
	}

SR_INLINE SrF32 SrMath::sin(SrF32 a)
	{
	return ::sinf(a);
	}

SR_INLINE SrF64 SrMath::sin(SrF64 a)
	{
	return ::sin(a);
	}

SR_INLINE SrF32 SrMath::cos(SrF32 a)
	{
	return ::cosf(a);
	}

SR_INLINE SrF64 SrMath::cos(SrF64 a)
	{
	return ::cos(a);
	}

// Calling fsincos instead of fsin+fcos
SR_INLINE void SrMath::sinCos(SrF32 f, SrF32& s, SrF32& c)
	{
#if defined(WIN32) && !defined(_WIN64)
		SrF32 localCos, localSin;
		SrF32 local = f;
		_asm	fld		local
		_asm	fsincos
		_asm	fstp	localCos
		_asm	fstp	localSin
		c = localCos;
		s = localSin;
#else
		c = cosf(f);
		s = sinf(f);
#endif
	}

SR_INLINE void SrMath::sinCos(SrF64 a, SrF64 & s, SrF64 & c)
	{
	s = ::sin(a);
	c = ::cos(a);
	}

SR_INLINE SrF32 SrMath::tan(SrF32 a)
	{
	return ::tanf(a);
	}

SR_INLINE SrF64 SrMath::tan(SrF64 a)
	{
	return ::tan(a);
	}

SR_INLINE SrF32 SrMath::asin(SrF32 f)
	{
	// Take care of FPU inaccuracies
	if(f>=1.0f)	return (SrF32)SrHalfPiF32;
	if(f<=-1.0f)return -(SrF32)SrHalfPiF32;
				return ::asinf(f);
	}

SR_INLINE SrF64 SrMath::asin(SrF64 f)
	{
	// Take care of FPU inaccuracies
	if(f>=1.0)	return (SrF32)SrHalfPiF64;
	if(f<=-1.0)	return -(SrF32)SrHalfPiF64;
				return ::asin(f);
	}

SR_INLINE SrF32 SrMath::acos(SrF32 f)
	{
	// Take care of FPU inaccuracies
	if(f>=1.0f)	return 0.0f;
	if(f<=-1.0f)return (SrF32)SrPiF32;
				return ::acosf(f);
	}

SR_INLINE SrF64 SrMath::acos(SrF64 f)
	{
	// Take care of FPU inaccuracies
	if(f>=1.0)	return 0.0;
	if(f<=-1.0)	return (SrF64)SrPiF64;
				return ::acos(f);
	}

SR_INLINE SrF32 SrMath::atan(SrF32 a)
	{
	return ::atanf(a);
	}

SR_INLINE SrF64 SrMath::atan(SrF64 a)
	{
	return ::atan(a);
	}

SR_INLINE SrF32 SrMath::atan2(SrF32 x, SrF32 y)
	{
	return ::atan2f(x,y);
	}

SR_INLINE SrF64 SrMath::atan2(SrF64 x, SrF64 y)
	{
	return ::atan2(x,y);
	}

SR_INLINE SrF32 SrMath::rand(SrF32 a,SrF32 b)
	{
	const SrF32 r = (SrF32)::rand()/((SrF32)RAND_MAX+1);
	return r*(b-a) + a;
	}

SR_INLINE SrI32 SrMath::rand(SrI32 a,SrI32 b)
	{
	return a + (SrI32)(::rand()%(b-a));
	}

/*
--------------------------------------------------------------------
lookup2.c, by Bob Jenkins, December 1996, Public Domain.
hash(), hash2(), hash3, and mix() are externally useful functions.
Routines to test the hash are included if SELF_TEST is defined.
You can use this free for any purpose.  It has no warranty.
--------------------------------------------------------------------
--------------------------------------------------------------------
mix -- mix 3 32-bit values reversibly.
For every delta with one or two bit set, and the deltas of all three
  high bits or all three low bits, whether the original value of a,b,c
  is almost all zero or is uniformly distributed,
* If mix() is run forward or backward, at least 32 bits in a,b,c
  have at least 1/4 probability of changing.
* If mix() is run forward, every bit of c will change between 1/3 and
  2/3 of the time.  (Well, 22/100 and 78/100 for some 2-bit deltas.)
mix() was built out of 36 single-cycle latency instructions in a 
  structure that could supported 2x parallelism, like so:
      a -= b; 
      a -= c; x = (c>>13);
      b -= c; a ^= x;
      b -= a; x = (a<<8);
      c -= a; b ^= x;
      c -= b; x = (b>>13);
      ...
  Unfortunately, superscalar Pentiums and Sparcs can't take advantage 
  of that parallelism.  They've also turned some of those single-cycle
  latency instructions into multi-cycle latency instructions.  Still,
  this is the fastest good hash I could find.  There were about 2^^68
  to choose from.  I only looked at a billion or so.
--------------------------------------------------------------------
*/
#define SR_HASH_MIX(a,b,c) \
{ \
  a -= b; a -= c; a ^= (c>>13); \
  b -= c; b -= a; b ^= (a<<8); \
  c -= a; c -= b; c ^= (b>>13); \
  a -= b; a -= c; a ^= (c>>12);  \
  b -= c; b -= a; b ^= (a<<16); \
  c -= a; c -= b; c ^= (b>>5); \
  a -= b; a -= c; a ^= (c>>3);  \
  b -= c; b -= a; b ^= (a<<10); \
  c -= a; c -= b; c ^= (b>>15); \
}

/*
--------------------------------------------------------------------
 This works on all machines.  hash2() is identical to hash() on 
 little-endian machines, except that the length has to be measured
 in ub4s instead of bytes.  It is much faster than hash().  It 
 requires
 -- that the key be an array of ub4's, and
 -- that all your machines have the same endianness, and
 -- that the length be the number of ub4's in the key
--------------------------------------------------------------------
*/
SR_INLINE SrU32 SrMath::hash(const SrU32 *k, SrU32 length)
//register ub4 *k;        /* the key */
//register ub4  length;   /* the length of the key, in ub4s */
	{
	SrU32 a,b,c,len;

	/* Set up the internal state */
	len = length;
	a = b = 0x9e3779b9;  /* the golden ratio; an arbitrary value */
	c = 0;           /* the previous hash value */

	/*---------------------------------------- handle most of the key */
	while (len >= 3)
	{
	  a += k[0];
	  b += k[1];
	  c += k[2];
	  SR_HASH_MIX(a,b,c);
	  k += 3; len -= 3;
	}

	/*-------------------------------------- handle the last 2 ub4's */
	c += length;
	switch(len)              /* all the case statements fall through */
	{
	 /* c is reserved for the length */
	case 2 : b+=k[1];
	case 1 : a+=k[0];
	 /* case 0: nothing left to add */
	}
	SR_HASH_MIX(a,b,c);
	/*-------------------------------------------- report the result */
	return c;
	}
#undef SR_HASH_MIX

SR_INLINE int SrMath::hash32(int key)
	{
	key += ~(key << 15);
	key ^=  (key >> 10);
	key +=  (key << 3);
	key ^=  (key >> 6);
	key += ~(key << 11);
	key ^=  (key >> 16);
	return key;
	}

SR_INLINE SrF32 SrMath::sqrt(SrF32 a)
	{
		return (SrF32)::sqrt(a);
	}
SR_INLINE SrF64 SrMath::sqrt(SrF64 a)
	{
		return (SrF64)::sqrt(a);
	}


/** @} */
#endif

