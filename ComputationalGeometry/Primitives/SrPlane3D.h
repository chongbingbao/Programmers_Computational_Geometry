/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/04/09
****************************************************************************/
#ifndef SR_PRIMITIVE_PLANE3D_H
#define SR_PRIMITIVE_PLANE3D_H

/** \addtogroup primitive
  @{
*/
#include "SrPrimitive.h"

/**
\brief 3D plane class.

This is a 3D plane class with public data members.
The line is parameterized as ^n*^X+d=0,in which ^n is the 'normal' data,d is the 'd' data.
The normal isn't normalized.
*/
class SrPlane3D
{
public:
	/**
	\brief Default constructor.
	*/
	SrPlane3D();

	SrPlane3D(const SrVector3D&,SrReal);

	SrPlane3D(const SrVector3D&,const SrPoint3D&);

	SrPlane3D(const SrPoint3D&,const SrPoint3D&,const SrPoint3D&);

	SrPlane3D(const SrPoint3D*,int);
	/**
	\brief Destructor. Do nothing.
	*/
	~SrPlane3D();
	/**
	\brief  The plane is valid if the normal of the plane is not zero.
	*/
	bool	isValid()const;
	/**
	\brief  Compute the distance from point p to the plane. 
	*/
	SrReal	toPointDistance(const SrPoint3D& p)const;
	SrReal	toPointSignedDistance(const SrPoint3D&)const;
	/**
	\brief  Judge whether the point is on the plane or not.
	*/
	bool	isOnPlane(const SrPoint3D&)const;
	/**
	\brief  Projection of a point or a vector onto a plane.
	*/
	const SrPoint3D  projectPoint(const SrPoint3D&) const;
	const SrVector3D projectVector(const SrVector3D&) const;
	/**
	\brief  Compute angle between a line and a plane or between the two planes.
	*/
	SrReal intersectAngleLine(const SrLine3D&) const;
	SrReal intersectAnglePlane(const SrPlane3D&) const; 
	/**
	\brief  Compute the intersection point between a line,or a ray,or a segment and a plane.
	\return SR_INTERSECTING	 if the linear component intersects with the plane.
			SR_OVERLAPPING	 if the linear component is on the plane. 
			SR_PARALLEL		 if the linear component is parallel with the plane. 
			SR_DISJOINT		 if the linear component doesn't intersect with the plane. 
	*/
	int intersectLine(const SrLine3D&,SrPoint3D&) const;
	int intersectRay(const SrRay3D&,SrPoint3D&) const;
	int intersectSegment(const SrSegment3D&,SrPoint3D&) const;
	/**
	\brief  Initialize a plane by giving the parameters, normal and d, of the plane.
	*/
	bool initPlane(const SrVector3D& ,SrReal);
	/**
	\brief  Initialize a plane by three points.The order of the points determine the direction of the normal of plane.
	*/
	bool initPlane(const SrPoint3D&,const SrPoint3D&,const SrPoint3D&);
	/**
	\brief  Initialize a plane by plane normal and a given point that the plane includes.
	*/
	bool initPlane(const SrVector3D&,const SrPoint3D&);
	/**
	\brief  Initialize a plane by a set of points.
	*/
	bool initPlane(const SrPoint3D*,int);

	/**
	\brief  Compute the intersection between the two planes.
	*/
	bool intersectPlane(const SrPlane3D& plane,SrLine3D& line)const;

public:
	SrVector3D	mNormal;
	SrReal		mD;

private:
	bool computeNormal( const SrPoint3D* point, int number, SrVector3D& result) const
	{
		SrVector3D normal(0,0,0);
		int i;
		for( i=0 ; i<number ; i++ )
		{
			normal.x += (point[i].y-point[(i+1)%number].y)*(point[i].z+point[(i+1)%number].z);
			normal.y += (point[i].z-point[(i+1)%number].z)*(point[i].x+point[(i+1)%number].x);
			normal.z += (point[i].x-point[(i+1)%number].x)*(point[i].y+point[(i+1)%number].y);
		}
		if( EQUAL(normal.x,0) && EQUAL(normal.y,0) && EQUAL(normal.z,0))
			return false;
		result = normal;
		return true;
	}
};

/** @} */
#endif