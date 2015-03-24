/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/11/22
****************************************************************************/
#ifndef	SR_FOUNDATION_VECTOR2_H_
#define SR_FOUNDATION_VECTOR2_H_
/** \addtogroup foundation
  @{
*/

#include <assert.h>
#include "SrSimpleTypes.h"
#include "SrMath.h"

#ifndef SR_ASSERT
#ifdef _DEBUG
#define SR_ASSERT(x) assert(x)
#else
#define SR_ASSERT(x) {}
#endif
#endif


/**
\brief 2 Element vector class.

This is a vector class with public data members.
This is not nice but it has become such a standard that hiding the xy data members
makes it difficult to reuse external code that assumes that these are public in the library.
The vector class can be made to use float or double precision by appropriately defining float.
This has been chosen as a cleaner alternative to a template class.
*/
class SrVector2
	{
	public:
	//!Constructors

	/**
	\brief default constructor leaves data uninitialized.
	*/
	SR_INLINE SrVector2();

	/**
	\brief Assigns scalar parameter to all elements.
	
	Useful to initialize to zero or one.

	\param[in] a Value to assign to elements.
	*/
	SR_INLINE explicit SrVector2(SrReal a);

	/**
	\brief Initializes from 2 scalar parameters.

	\param[in] nx Value to initialize X component.
	\param[in] ny Value to initialize Y component.
	*/
	SR_INLINE SrVector2(SrReal nx, SrReal ny);
	
	/**
	\brief Initializes from an array of scalar parameters.

	\param[in] v Value to initialize with.
	*/
	SR_INLINE SrVector2(const SrReal v[]);

	/**
	\brief Copy constructor.
	*/
	SR_INLINE SrVector2(const SrVector2& v);

	/**
	\brief Assignment operator.
	*/
	SR_INLINE const SrVector2& operator=(const SrVector2&);

	/**
	\brief Access the data as an array.

	\return Array of 2 floats.
	*/
	SR_INLINE const SrReal *get() const;
	/**
	\brief writes out the 2 values to dest.

	\param[out] dest Array to write elements to.
	*/
	SR_INLINE void get(SrF32 * dest) const;

	/**
	\brief writes out the 2 values to dest.
	*/
	SR_INLINE void get(SrF64 * dest) const;

	/**
	\brief Access the data as an array.

	\return Array of 2 floats.
	*/
	SR_INLINE SrReal* get();

	SR_INLINE SrReal& operator[](int index);
	SR_INLINE SrReal  operator[](int index) const;

	//Operators
	/**
	\brief true if all the members are smaller.
	*/
	SR_INLINE bool operator< (const SrVector2&) const;
	/**
	\brief returns true if the two vectors are exactly equal.

	use equal() to test with a tolerance.
	*/
	SR_INLINE bool operator==(const SrVector2&) const;
	/**
	\brief returns true if the two vectors are exactly unequal.

	use !equal() to test with a tolerance.
	*/
	SR_INLINE bool operator!=(const SrVector2&) const;

//Methods
	
	/**
	\brief reads 2 consecutive values from the ptr passed
	*/
	SR_INLINE void  set(const SrF32 *);

	/**
	\brief reads 2 consecutive values from the ptr passed
	*/
	SR_INLINE void  set(const SrF64 *);
	SR_INLINE void  set(const SrVector2 &);

//legacy methods:
	SR_INLINE void setx(const SrReal & d);
	SR_INLINE void sety(const SrReal & d);

	/**
	\brief this = -a
	*/
	SR_INLINE void  setNegative(const SrVector2 &a);

	/**
	\brief this = -this
	*/
	SR_INLINE void  setNegative();

	/**
	\brief reads 2 consecutive values from the ptr passed
	*/
	SR_INLINE void  set(SrReal, SrReal);
	SR_INLINE void  set(SrReal);

	SR_INLINE void  zero();
	
	/**
	\brief tests for exact zero vector
	*/
	SR_INLINE bool isZero()	const
		{
		if((x != 0.0f) || (y != 0.0f))	return false;
		return true;
		}

	SR_INLINE void  setPlusInfinity();
	SR_INLINE void  setMinusInfinity();

	/**
	\brief this = element wise min(this,other)
	*/
	SR_INLINE void min(const SrVector2 &);
	/**
	\brief this = element wise max(this,other)
	*/
	SR_INLINE void max(const SrVector2 &);

	/**
	\brief this = a + b
	*/
	SR_INLINE void  add(const SrVector2 & a, const SrVector2 & b);
	/**
	\brief this = a - b
	*/
	SR_INLINE void  subtract(const SrVector2 &a, const SrVector2 &b);
	/**
	\brief this = s * a;
	*/
	SR_INLINE void  multiply(SrReal s,  const SrVector2 & a);

	/**
	\brief this[i] = a[i] * b[i], for all i.
	*/
	SR_INLINE void  arrayMultiply(const SrVector2 &a, const SrVector2 &b);


	/**
	\brief this = s * a + b;
	*/
	SR_INLINE void  multiplyAdd(SrReal s, const SrVector2 & a, const SrVector2 & b);

	/**
	\brief normalizes the vector
	*/
	SR_INLINE SrReal normalize();

	/**
	\brief sets the vector's magnitude
	*/
	SR_INLINE void	setMagnitude(SrReal);

	/**
	\brief snaps to closest axis
	*/
	SR_INLINE SrU32	closestAxis()	const;

	/**
	\brief returns true if all 2 elems of the vector are finite (not NAN or INF, etc.)
	*/
	SR_INLINE bool isFinite() const;

	/**
	\brief returns the scalar product of this and other.
	*/
	SR_INLINE SrReal dot(const SrVector2 &other) const;
	/**
	\brief returns the scalar cross product of this and other.
	*/
	SR_INLINE SrReal cross(const SrVector2 &other) const;

	/**
	\brief compares orientations (more readable, user-friendly function)
	*/
	SR_INLINE bool sameDirection(const SrVector2 &) const;

	/**
	\brief returns the magnitude
	*/
	SR_INLINE SrReal magnitude() const;

	/**
	\brief returns the squared magnitude
	
	Avoids calling sqrt()!
	*/
	SR_INLINE SrReal magnitudeSquared() const;

	/**
	\brief returns (this - other).magnitude();
	*/
	SR_INLINE SrReal distance(const SrVector2 &) const;

	/**
	\brief returns (this - other).magnitudeSquared();
	*/
	SR_INLINE SrReal distanceSquared(const SrVector2 &v) const;

	/**
	\brief Stuff magic values in the point, marking it as explicitly not used.
	*/
	SR_INLINE void setNotUsed();

	/**
	\brief Checks the point is marked as not used
	*/
	SR_INLINE bool isNotUsed() const;

	/**
	\brief returns true if this and arg's elems are within epsilon of each other.
	*/
	SR_INLINE bool equals(const SrVector2 &, SrReal epsilon) const;

	/**
	\brief negation
	*/
	SR_INLINE SrVector2 operator -() const;
	/**
	\brief vector addition
	*/
	SR_INLINE SrVector2 operator +(const SrVector2 & v) const;
	/**
	\brief vector difference
	*/
	SR_INLINE SrVector2 operator -(const SrVector2 & v) const;
	/**
	\brief scalar post-multiplication
	*/
	SR_INLINE SrVector2 operator *(SrReal f) const;
	/**
	\brief scalar division
	*/
	SR_INLINE SrVector2 operator /(SrReal f) const;
	/**
	\brief vector addition
	*/
	SR_INLINE SrVector2&operator +=(const SrVector2& v);
	/**
	\brief vector difference
	*/
	SR_INLINE SrVector2&operator -=(const SrVector2& v);
	/**
	\brief scalar multiplication
	*/
	SR_INLINE SrVector2&operator *=(SrReal f);
	/**
	\brief scalar division
	*/
	SR_INLINE SrVector2&operator /=(SrReal f);
	/**
	\brief dot product
	*/
	SR_INLINE SrReal      operator|(const SrVector2& v) const;

	SrReal x,y;
	};


/** \endcond */

//implementations:
SrVector2::SrVector2(SrReal v) : x(v), y(v)
	{
	}

SR_INLINE SrVector2::SrVector2(SrReal _x, SrReal _y) : x(_x), y(_y)
	{
	}


SR_INLINE SrVector2::SrVector2(const SrReal v[]) : x(v[0]), y(v[1])
	{
	}


SR_INLINE SrVector2::SrVector2(const SrVector2 &v) : x(v.x), y(v.y)
	{
	}


SR_INLINE SrVector2::SrVector2()
	{
	//default constructor leaves data uninitialized.
	}


SR_INLINE const SrVector2& SrVector2::operator=(const SrVector2& v)
	{
	x = v.x;
	y = v.y;
	return *this;
	}


// Access the data as an array.

SR_INLINE const SrReal* SrVector2::get() const
	{
	return &x;
	}


SR_INLINE SrReal* SrVector2::get()
	{
	return &x;
	}

 
SR_INLINE void  SrVector2::get(SrF32 * v) const
	{
	v[0] = (SrF32)x;
	v[1] = (SrF32)y;
	}

 
SR_INLINE void  SrVector2::get(SrF64 * v) const
	{
	v[0] = (SrF64)x;
	v[1] = (SrF64)y;
	}


SR_INLINE SrReal& SrVector2::operator[](int index)
	{
	SR_ASSERT(index>=0 && index<=1);
	return (&x)[index];
	}


SR_INLINE SrReal  SrVector2::operator[](int index) const
	{
	SR_ASSERT(index>=0 && index<=1);
	return (&x)[index];
	}

 
SR_INLINE void SrVector2::setx(const SrReal & d) 
	{ 
	x = d; 
	}

 
SR_INLINE void SrVector2::sety(const SrReal & d) 
	{ 
	y = d; 
	}

//Operators
 
SR_INLINE bool SrVector2::operator < (const SrVector2&v) const
	{
		if( x < v.x)	return true;
		if( x > v.x)	return false;
		if( y < v.y)	return true;
		return false;
	}

 
SR_INLINE bool SrVector2::operator==(const SrVector2& v) const
	{
	return ((x == v.x)&&(y == v.y));
	}

 
SR_INLINE bool SrVector2::operator!=(const SrVector2& v) const
	{
	return ((x != v.x)||(y != v.y));
	}

//Methods
 
SR_INLINE void  SrVector2::set(const SrVector2 & v)
	{
	x = v.x;
	y = v.y;
	}

 
SR_INLINE void  SrVector2::setNegative(const SrVector2 & v)
	{
	x = -v.x;
	y = -v.y;
	}

 
SR_INLINE void  SrVector2::setNegative()
	{
	x = -x;
	y = -y;
	}


 
SR_INLINE void  SrVector2::set(const SrF32 * v)
	{
	x = (SrReal)v[0];
	y = (SrReal)v[1];
	}

 
SR_INLINE void  SrVector2::set(const SrF64 * v)
	{
	x = (SrReal)v[0];
	y = (SrReal)v[1];
	}


 
SR_INLINE void  SrVector2::set(SrReal _x, SrReal _y)
	{
	this->x = _x;
	this->y = _y;
	}

 
SR_INLINE void SrVector2::set(SrReal v)
	{
	x = v;
	y = v;
	}

 
SR_INLINE void  SrVector2::zero()
	{
	x = y = 0.0;
	}

 
SR_INLINE void  SrVector2::setPlusInfinity()
	{
	x = y = SR_MAX_F32; //TODO: this may be double too, but here we can't tell!
	}

 
SR_INLINE void  SrVector2::setMinusInfinity()
	{
	x = y = SR_MIN_F32; //TODO: this may be double too, but here we can't tell!
	}

 
SR_INLINE void SrVector2::max(const SrVector2 & v)
	{
		x = x > v.x?x:v.x;
		y = y > v.y?y:y;
	}

 
SR_INLINE void SrVector2::min(const SrVector2 & v)
	{
		x = x > v.x?v.x:x;
		y = y > v.y?v.y:y;
	}




SR_INLINE void  SrVector2::add(const SrVector2 & a, const SrVector2 & b)
	{
	x = a.x + b.x;
	y = a.y + b.y;
	}


SR_INLINE void  SrVector2::subtract(const SrVector2 &a, const SrVector2 &b)
	{
	x = a.x - b.x;
	y = a.y - b.y;
	}


SR_INLINE void  SrVector2::arrayMultiply(const SrVector2 &a, const SrVector2 &b)
	{
	x = a.x * b.x;
	y = a.y * b.y;
	}


SR_INLINE void  SrVector2::multiply(SrReal s,  const SrVector2 & a)
	{
	x = a.x * s;
	y = a.y * s;
	}


SR_INLINE void  SrVector2::multiplyAdd(SrReal s, const SrVector2 & a, const SrVector2 & b)
	{
	x = s * a.x + b.x;
	y = s * a.y + b.y;
	}

 
SR_INLINE SrReal SrVector2::normalize()
	{
	SrReal m = magnitude();
	if (m)
		{
		const SrReal il =  SrReal(1.0) / m;
		x *= il;
		y *= il;
		}
	return m;
	}

 
SR_INLINE void SrVector2::setMagnitude(SrReal length)
	{
	SrReal m = magnitude();
	if(m)
		{
		SrReal newLength = length / m;
		x *= newLength;
		y *= newLength;
		}
	}
SR_INLINE bool SrVector2::isFinite() const
	{
		return SrMath::isFinite(x) && SrMath::isFinite(y);
	}
SR_INLINE SrReal SrVector2::dot(const SrVector2 &v) const
	{
		return x * v.x + y * v.y;
	}
SR_INLINE SrReal SrVector2::cross(const SrVector2 &other) const
	{
		return x*other.y-y*other.x;
	}
 
SR_INLINE bool SrVector2::sameDirection(const SrVector2 &v) const
	{
	return x*v.x + y*v.y >= 0.0f;
	}

 
SR_INLINE SrReal SrVector2::magnitude() const
	{
	return SrMath::sqrt(x * x + y * y);
	}

 
SR_INLINE SrReal SrVector2::magnitudeSquared() const
	{
	return x * x + y * y;
	}

 
SR_INLINE SrReal SrVector2::distance(const SrVector2 & v) const
	{
	SrReal dx = x - v.x;
	SrReal dy = y - v.y;
	return SrMath::sqrt(dx * dx + dy * dy);
	}

 
SR_INLINE SrReal SrVector2::distanceSquared(const SrVector2 &v) const
	{
	SrReal dx = x - v.x;
	SrReal dy = y - v.y;
	return dx * dx + dy * dy;
	}


 
SR_INLINE bool SrVector2::equals(const SrVector2 & v, SrReal epsilon) const
	{
	return 
		SrMath::equals(x, v.x, epsilon) &&
		SrMath::equals(y, v.y, epsilon);
	}


 
SR_INLINE SrVector2 SrVector2::operator -() const
	{
	return SrVector2(-x, -y);
	}

 
SR_INLINE SrVector2 SrVector2::operator +(const SrVector2 & v) const
	{
	return SrVector2(x + v.x, y + v.y);	// RVO version
	}

 
SR_INLINE SrVector2 SrVector2::operator -(const SrVector2 & v) const
	{
	return SrVector2(x - v.x, y - v.y);	// RVO version
	}



SR_INLINE SrVector2 SrVector2::operator *(SrReal f) const
	{
	return SrVector2(x * f, y * f);	// RVO version
	}


SR_INLINE SrVector2 SrVector2::operator /(SrReal f) const
	{
		f = SrReal(1.0) / f; return SrVector2(x * f, y * f);
	}


SR_INLINE SrVector2& SrVector2::operator +=(const SrVector2& v)
	{
	x += v.x;
	y += v.y;
	return *this;
	}


SR_INLINE SrVector2& SrVector2::operator -=(const SrVector2& v)
	{
	x -= v.x;
	y -= v.y;
	return *this;
	}


SR_INLINE SrVector2& SrVector2::operator *=(SrReal f)
	{
	x *= f;
	y *= f;
	
	return *this;
	}


SR_INLINE SrVector2& SrVector2::operator /=(SrReal f)
	{
	f = 1.0f/f;
	x *= f;
	y *= f;
	
	return *this;
	}


SR_INLINE SrReal SrVector2::operator|(const SrVector2& v) const
	{
	return x * v.x + y * v.y;
	}

/**
scalar pre-multiplication
*/

SR_INLINE SrVector2 operator *(SrReal f, const SrVector2& v)
	{
	return SrVector2(f * v.x, f * v.y);
	}
/** @} */
#endif
