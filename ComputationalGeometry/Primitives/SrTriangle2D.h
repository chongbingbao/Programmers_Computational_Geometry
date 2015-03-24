/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/04/08
****************************************************************************/
#ifndef SR_PRIMITIVE_TRIANGLE2D_H
#define SR_PRIMITIVE_TRIANGLE2D_H

/** \addtogroup primitive
  @{
*/

#include "SrPrimitive.h"

/**
\brief 2D triangle class.
This is a 2D triangle class with public data members.
*/

class SrTriangle2D
{
public:
	/**
	\brief Default constructor, endpoints is set to (0,0).
	*/
	SrTriangle2D();
	/**
	\brief The line is initialized by three points.
	*/
	SrTriangle2D(const SrPoint2D&,const SrPoint2D&,const SrPoint2D&);
	/**
	\brief Destructor. Do nothing.
	*/
	~SrTriangle2D();
	/**
	\brief  Compute the distance from the point to the triangle.
	*/
	SrReal	toPointDistance(const SrPoint2D&) const;
	/**
	\brief The triangle is valid if the three points are not on a line.
	*/
	bool	isValid() const;
	/**
	\brief  Compute the area.
	*/
	SrReal	area() const;
	/**
	\brief  Judge whether the point is in the triangle or not.
	\return SR_POSITION_ON			if the point is on the edge; 
			SR_POSITION_INSIDE		if inside the triangle; 
			SR_POSITION_OUTSIDE		if outside the triangle.
	*/
	int		pointLocation(const SrPoint2D&) const;
	/**
	\brief  Compute the perimeter.
	*/
	SrReal	perimeter() const;
	/**
	\brief  Rotate the triangle about the origin through an angle in the anticlockwise direction.
	*/
	void	rotate(SrReal);
	/**
	\brief  Translate the triangle.
	*/
	void	translate(const SrVector2D&);
	/**
	\brief  Judge whether the two triangles intersect with each other.
	\return SR_INTERSECTING			if intersecting 
			SR_DISJOINT				if separating; 
	*/
	int		intersectTriangle(const SrTriangle2D&)const;


public:
	SrPoint2D	mPoint[3];

};

/** @} */
#endif