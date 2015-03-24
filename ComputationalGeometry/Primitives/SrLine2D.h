/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#ifndef SR_PRIMITIVE_LINE2D_H
#define SR_PRIMITIVE_LINE2D_H

/** \addtogroup primitive
  @{
*/

#include "SrPrimitive.h"
/**
\brief 2D line class.

This is a 2D line class with public data members.
The line is parameterized as X(t) = P+t*d,in which P is the 'base' data,d is the 'direction' data.
the 'direction' is unit length.
*/

class SrLine2D
{
public:
	/**
	\brief Default constructor, base and direction is set to (0,0).
	*/
	SrLine2D();
	/**
	\brief The line is initialized by two points.
	*/
	SrLine2D(const SrPoint2D& p1,const SrPoint2D& p2);
	/**
	\brief Destructor. Do nothing.
	*/
	~SrLine2D();
	/**
	\brief  The line is valid if the direction of the line is unit.
	*/
	bool isValid()const;
	/**
	\brief  Compute the squared Euclidean distance from the point to the line.

	Avoids calling sqrt()!
	*/
	SrReal toPointDistanceSquared(const SrPoint2D&)const;
	/**
	\brief  Compute the Euclidean distance from point p to the line. 
	*/
	SrReal toPointDistance(const SrPoint2D&)const;
	/**
	\brief  Rotate the line about the origin through an angle in the anticlockwise direction.
	*/
	void rotate(SrReal);
	/**
	\brief  Translate the line.
	*/
	void translate(const SrVector2D&);
	/*
	*\brief  Determine the relation of the two lines.
	*\return SR_OVERLAPPING		if they are overlapping.
			 SR_INTERSECTING		if intersecting. 
			 SR_PARALLEL		if parallel.
	*/
	int intersectLine(const SrLine2D&,SrPoint2D&)const;
	/*
	*\brief  Determine the relation of the line and the ray.
	*\return SR_OVERLAPPING		if they are overlapping.
			 SR_INTERSECTING	if intersecting. 
			 SR_PARALLEL		if parallel
			 SR_DISJOINT		if not intersecting.
	*/
	int intersectRay(const SrRay2D&,SrPoint2D&)const;
	/*
	*\brief  Determine the relation of the line and the segment.
	*\return SR_OVERLAPPING		if they are overlapping.
			 SR_INTERSECTING	if intersecting. 
			 SR_PARALLEL		if parallel
			 SR_DISJOINT		if not intersecting.
	*/
	int intersectSegment(const SrSegment2D&,SrPoint2D&)const;
	/*
	*\brief  Determine the relation of the point and the line.
	*\return SR_LINEAR_ON			if on the line.
			 SR_LINEAR_ABSENT		or not.
	*/
	int pointLocation(const SrPoint2D&)const;

public:
	SrPoint2D	mBase;
	SrVector2D	mDirection;
};

/** @} */

#endif