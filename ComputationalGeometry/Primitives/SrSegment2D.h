/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#ifndef SR_PRIMITIVE_SEGMENT2D_H
#define SR_PRIMITIVE_SEGMENT2D_H

/** \addtogroup primitive
  @{
*/

#include "SrPrimitive.h"

/**
\brief 2D segment class.

This is a 2D segment class with public data members,two endpoints.
*/


class SrSegment2D
{
public:
	/**
	\brief Default constructor, the two endpoints is set to (0,0).
	*/
	SrSegment2D();
	/**
	\brief The segment is initialized by two points.
	*/
	SrSegment2D(const SrPoint2D& p1,const SrPoint2D& p2);
	/**
	\brief Destructor. Do nothing.
	*/
	~SrSegment2D();
	/**
	\brief  The segment is valid if the two endpoints aren't the same.
	*/
	bool isValid()const;
	/**
	\brief  Compute the squared distance from point p to the segment.

	Avoids calling sqrt()!
	*/
	SrReal toPointDistanceSquared(const SrPoint2D& p)const;
	/**
	\brief  Compute the distance from point p to the segment. 
	*/
	SrReal toPointDistance(const SrPoint2D& p)const;
	/**
	\brief  Rotate the segment about the origin through an angle in the anticlockwise direction.
	*/
	void rotate(SrReal);
	/**
	\brief  Translate the segment.
	*/
	void translate(const SrVector2D&);
	/*
	*\brief  Determine the relation of the line and the segment.
	*\return SR_OVERLAPPING		if they are overlapping.
			 SR_INTERSECTING	if intersecting. 
			 SR_PARALLEL		if parallel
			 SR_DISJOINT		if not intersecting.
	*/
	int intersectLine(const SrLine2D&,SrPoint2D&)const;
	/*
	*\brief  Determine the relation of the ray and the segment.
	*\return SR_OVERLAPPING		if they are overlapping.
			 SR_PARALLEL		if parallel.
			 SR_INTERSECTING	if intersecting.
			 SR_DISJOINT		if not intersecting.
	*/
	int intersectRay(const SrRay2D&,SrPoint2D&)const;
	/*
	*\brief  Determine the relation of the two segments.
	*\return SR_OVERLAPPING		if they are overlapping.
			 SR_PARALLEL		if parallel.
			 SR_INTERSECTING	if intersecting.
			 SR_DISJOINT		if not intersecting.
	*/
	int intersectSegment(const SrSegment2D&,SrPoint2D&)const;
	/*
	*\brief  Determine the relation of the two segments.
	*\return SR_INTERSECTING	if intersecting.
			 SR_DISJOINT		if not intersecting.
	*/
	int intersectSegment(const SrSegment2D&)const;
	/*
	*\brief  Determine the relation of the point and the segment.
	*\return SR_LINEAR_ON			if on the segment.
			 SR_LINEAR_ABSENT		or else.
	*/
	int pointLocation(const SrPoint2D&)const;


public:
	SrPoint2D	mPoint1;
	SrPoint2D	mPoint2;
};

/** @} */
#endif