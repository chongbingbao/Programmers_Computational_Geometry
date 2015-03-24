/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#ifndef SR_PRIMITIVE_LINE3D_H
#define SR_PRIMITIVE_LINE3D_H

/** \addtogroup primitive
  @{
*/

#include "SrPrimitive.h"
/**
\brief 3D line class.

This is a 3D line class with public data members.
The line is parameterized as X(t) = P+t*d,in which P is the 'base' data,d is the 'direction' data.
the 'direction' is unit length.
*/

class SrLine3D
{
public:
	/**
	\brief Default constructor, base and direction is set to (0,0,0).
	*/
	SrLine3D();
	/**
	\brief The line is initialized by two points.
	*/
	SrLine3D(const SrPoint3D& p1,const SrPoint3D& p2);
	/**
	\brief Destructor. Do nothing.
	*/
	~SrLine3D();
	/**
	\brief  Return true if the line is valid.
	*/
	bool isValid()const;
	/**
	\brief  Compute the Euclidean distance from the point to the line. 
	*/
	SrReal toPointDistance(const SrPoint3D&)const;
	/**
	\brief Rotate the line through an angle about a coordinate axis.
		
		For an x-roll,the y-axis rotates to the z-axis.
		For a y-roll,the z-axis rotates to the x-axis.
		For a z-roll,the x-axis rotates to the y-axis.
	*/
	void rotateX(SrReal);
	void rotateY(SrReal);
	void rotateZ(SrReal);
	/**
	\brief  Translate the line.
	*/
	void translate(const SrVector3D&);
	/**
	\brief  Compute angle between two lines.
	\param[in] lineDirDesired is false if the angle is less or equal to Pi/2; 
			true if the angle is between 0 and Pi because of the direction of the lines.
	*/
	SrReal interAngleLine(const SrLine3D&,bool lineDirDesired=true)const;

	/**
	\brief Compute the squared distance between the two lines.
	*/
	SrReal distanceLineSquared(const SrLine3D&)const;
	/**
	\brief Compute the squared distance between a line and a ray.
	*/
	SrReal distanceRaySquared(const SrRay3D&)const;
	/**
	\brief Compute the squared distance between a line and a segment.
	*/
	SrReal distanceSegmentSquared(const SrSegment3D&)const;

public:
	SrPoint3D mBase;
	SrPoint3D mDirection;
};

/** @} */

#endif