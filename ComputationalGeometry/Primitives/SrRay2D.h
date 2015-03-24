/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#ifndef SR_PRIMITIVE_RAY2D_H
#define SR_PRIMITIVE_RAY2D_H

/** \addtogroup primitive
  @{
*/
#include "SrPrimitive.h"
/**
\brief 2D ray class.

This is a 2D ray class with public data members.
The ray is parameterized as X(t) = P+t*d,in which P is the 'base' data,d is the 'direction' data,t>=0.
the 'direction' is unit length.
*/
class SrRay2D
{
public:
	/**
	\brief Default constructor, base and direction is set to (0,0).
	*/
	SrRay2D();
	/**
	\brief The line is initialized by two points.The point p1 is the endpoint of the ray.
	*/
	SrRay2D(const SrPoint2D& p1,const SrPoint2D& p2);
	/**
	\brief Destructor. Do nothing.
	*/
	~SrRay2D();
	/**
	\brief  The ray is valid if the direction of the line is unit.
	*/
	bool isValid()const;
	/**
	\brief  Compute the squared distance from point p to the ray.

	Avoids calling sqrt()!
	*/
	SrReal	toPointDistanceSquared(const SrPoint2D& p)const;
	/**
	\brief  Compute the distance from point p to the ray. 
	*/
	SrReal	toPointDistance(const SrPoint2D& p)const;
	/**
	\brief  Rotate the ray about the origin through an angle in the anticlockwise direction.
	*/
	void rotate(SrReal);
	/**
	\brief  Translate the ray.
	*/
	void translate(const SrVector2D&);
	/*
	*\brief  Determine the relation of the line and the ray.
	*\return SR_OVERLAPPING		if they are overlapping.
			 SR_INTERSECTING	if intersecting. 
			 SR_PARALLEL		if parallel
			 SR_DISJOINT		if not intersecting.
	*/
	int intersectLine(const SrLine2D& ,SrPoint2D&)const;
	/*
	*\brief  Determine the relation of the two rays.
	*\return SR_OVERLAPPING		if they are overlapping.
			 SR_INTERSECTING	if intersecting. 
			 SR_PARALLEL		if parallel
			 SR_DISJOINT		if not intersecting.
	*/
	int intersectRay(const SrRay2D&, SrPoint2D&)const;
	/*
	*\brief  Determine the relation of the ray and the segment.
	*\return SR_OVERLAPPING		if they are overlapping.
			 SR_PARALLEL		if parallel.
			 SR_INTERSECTING	if intersecting.
			 SR_DISJOINT		if not intersecting.
	*/
	int intersectSegment(const SrSegment2D&, SrPoint2D&)const;
	/*
	*\brief  Determine the relation of the point and the ray.
	*\return SR_LINEAR_ON			if on the ray.
			 SR_LINEAR_ABSENT		or else.
	*/
	int pointLocation(const SrPoint2D&)const;
public:
	SrPoint2D	mBase;
	SrVector2D	mDirection;
};

/** @} */
#endif