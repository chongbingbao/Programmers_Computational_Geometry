/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#ifndef SR_PRIMITIVE_SEGMENT3D_H
#define SR_PRIMITIVE_SEGMENT3D_H

/** \addtogroup primitive
  @{
*/

#include "SrPrimitive.h"


/**
\brief 3D segment class.
This is a 3D segment class with public data members,two endpoints.
*/
class SrSegment3D
{
public:
	/**
	\brief Default constructor, the two endpoints is set to (0,0,0).
	*/
	SrSegment3D();
	/**
	\brief The segment is initialized by two points.
	*/
	SrSegment3D(const SrPoint3D& p1,const SrPoint3D& p2);
	/**
	\brief Destructor. Do nothing.
	*/
	~SrSegment3D();
	/**
	\brief  true if the line is valid.
	*/
	bool isValid()const;
	/**
	\brief  Compute the distance from the point to the segment. 
	*/
	SrReal toPointDistance(const SrPoint3D& p)const;
	/**
	\brief Rotate the segment through an angle about a coordinate axis.
		
		For an x-roll,the y-axis rotates to the z-axis.
		For a y-roll,the z-axis rotates to the x-axis.
		For a z-roll,the x-axis rotates to the y-axis.
	*/
	void rotateX(SrReal);
	void rotateY(SrReal);
	void rotateZ(SrReal);
	/**
	\brief  Translate the segment.
	*/
	void translate(const SrVector3D&);
	/**
	\brief Compute the squared distance between a line and a segment.
	*/
	SrReal distanceLineSquared(const SrLine3D&)const;
	/**
	\brief Compute the squared distance between a ray and a segment.
	*/
	SrReal distanceRaySquared(const SrRay3D&)const;
	/**
	\brief Compute the squared distance between the two segments.
	*/
	SrReal distanceSegmentSquared(const SrSegment3D&)const;

public:
	SrPoint3D mPoint1;
	SrPoint3D mPoint2;
};

/** @} */
#endif