/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/05/09
****************************************************************************/
#ifndef SR_PRIMITIVE_RECTANGLE2D_H
#define SR_PRIMITIVE_RECTANGLE2D_H

/** \addtogroup primitive
  @{
*/

#include "SrDataType.h"
#include "SrRay2D.h"

/**
\brief 2D rectangle class.
This is a 2D rectangle class with public data members.


*/
class SrRectangle2D
{
public:
	/**
	\brief Default constructor, endpoints is set to (0,0).
	*/
	SrRectangle2D();
	/**
	\brief The 2d rectangle is initialized 
	*/
	SrRectangle2D(const SrPoint2D& center,const SrPoint2D* axis,const SrReal* halfLength);
	/**
	\brief Destructor. Do nothing.
	*/
	~SrRectangle2D();
	/**
	\brief  true if the rectangle is valid.
	*/
	bool	isValid() const;
	/**
	\brief  Compute the squared distance from point p to the rectangle.

	Avoids calling sqrt()!
	*/
	SrReal	toPointDistanceSquared(const SrPoint2D& p) const;
	/**
	\brief  Compute the distance from point p to the line.
	*/
	SrReal	toPointDistance(const SrPoint2D& p) const;
	/**
	\brief  Compute the area.
	*/
	SrReal	area() const;
	/**
	\brief  Judge whether the point is in the rectangle or not.
	\return SR_POSITION_ON		if the point is on the edge; 
			SR_POSITION_INSIDE if inside the rectangle; 
			SR_POSITION_OUTSIDE if outside the rectangle.
	*/
	int		pointLocation(const SrPoint2D&) const;
	/**
	\brief  Compute the perimeter.
	*/
	SrReal	perimeter() const;
	/**
	\brief  Rotate the rectangle about the origin through an angle in the anticlockwise direction.
	*/
	void rotate(SrReal);
	/**
	\brief  Translate the rectangle.
	*/
	void translate(const SrVector2D&);
	/**
	\brief  Judge whether the ray hits the rectangle.
	\param[out] result It's used as the intersection point when the ray hits the rectangle.
	\return false,if the ray miss the rectangle.
			true, if the ray hit the rectangle.
			 
	Allocate memory for result and dellocate it when it's useless.
	*/
	bool hitTest(const SrRay2D&,SrPoint2D& /*[OUT]*/ result)const;


public:
	SrPoint2D	mCenter;
	SrPoint2D	mAxis[2];
	SrReal		mHalfLength[2];
};

/** @} */
#endif