/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#ifndef SR_PRIMITIVE_RAY3D_H
#define SR_PRIMITIVE_RAY3D_H

/** \addtogroup primitive
  @{
*/
#include "SrPrimitive.h"
/**
\brief 3D ray class.

This is a 3D ray class with public data members.
The ray is parameterized as X(t) = P+t*d,in which P is the 'base' data,d is the 'direction' data,t>=0.
the 'direction' is unit length.
*/
class SrRay3D
{
public:
	/**
	\brief Default constructor, base and direction is set to (0,0,0).
	*/
	SrRay3D();
	/**
	\brief The ray is initialized by two points.The first point p1 is the endpoint of the ray.
	*/
	SrRay3D(const SrPoint3D& p1,const SrPoint3D& p2);
	/**
	\brief Destructor. Do nothing.
	*/
	~SrRay3D();
	/**
	\brief  true if the ray is valid.
	*/
	bool isValid()const;
	/**
	\brief  Compute the distance from the point to the ray. 
	*/
	SrReal	toPointDistance(const SrPoint3D&)const;
	/**
	\brief Rotate the ray through an angle about a coordinate axis.
		
		For an x-roll,the y-axis rotates to the z-axis.
		For a y-roll,the z-axis rotates to the x-axis.
		For a z-roll,the x-axis rotates to the y-axis.
	*/
	void rotateX(SrReal);
	void rotateY(SrReal);
	void rotateZ(SrReal);
	/**
	\brief  Translate the ray.
	*/
	void translate(const SrVector3D&);
	/**
	\brief Compute the squared distance between a line and a ray.
	*/
	SrReal distanceLineSquared(const SrLine3D&)const;
	/**
	\brief Compute the squared distance between the two rays.
	*/
	SrReal distanceRaySquared(const SrRay3D&)const;
	/**
	\brief Compute the squared distance between a ray and a segment.
	*/
	SrReal distanceSegmentSquared(const SrSegment3D&)const;
public:
	SrPoint3D	mBase;
	SrPoint3D	mDirection;
};

/** @} */
#endif