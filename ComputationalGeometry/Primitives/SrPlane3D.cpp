/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#include "SrPlane3D.h"


SrPlane3D::SrPlane3D()
{
	mNormal.set(0,0,0);
	mD = 0;
}

SrPlane3D::SrPlane3D(const SrVector3D& norm,SrReal dValue)
{
	ASSERT(initPlane(norm,dValue));
}

SrPlane3D::SrPlane3D(const SrVector3D& direction,const SrPoint3D& point)
{
	ASSERT(initPlane(direction,point));
}

SrPlane3D::SrPlane3D(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D& p2)
{
	ASSERT(initPlane(p0,p1,p0));
}


SrPlane3D::SrPlane3D(const SrPoint3D* point,int numPoint)
{
	ASSERT(initPlane(point,numPoint));
}

SrPlane3D::~SrPlane3D()
{

}

 bool SrPlane3D::isValid()const
 {
	 if( EQUAL(mNormal.x,0)&&EQUAL(mNormal.y,0)&&EQUAL(mNormal.z,0) )
		 return false;
	 return true;
 }
 
 SrReal	SrPlane3D::toPointDistance(const SrPoint3D& p)const
 {
	 //p*n + d / ||n||
	 return fabs(toPointSignedDistance(p));
 }

 SrReal	SrPlane3D::toPointSignedDistance(const SrPoint3D& p)const
 {
	return (p.dot(mNormal)+mD)/mNormal.magnitude();
 }

 const SrPoint3D SrPlane3D::projectPoint(const SrPoint3D& p) const
 {
	 //p - (p*n + d)n / (||n||^2)
	 return p - (p.dot(mNormal)+mD)*mNormal/(mNormal.magnitudeSquared());
 }

 const SrVector3D SrPlane3D::projectVector(const SrVector3D& v) const
 {
	 //v - (v*n)n / (||n||^2)
	 return v-v.dot(mNormal)*mNormal/(mNormal.magnitudeSquared());
 }

 SrReal SrPlane3D::intersectAngleLine(const SrLine3D& line) const
 {
	 //Pi/2 - b, in which b = arccos( ||(n*d) / (||n|| * ||d||)||)
	 return SrHalfPiF64 - SrMath::acos(fabs(mNormal.dot(line.mDirection)/mNormal.magnitude()));
 }

 SrReal SrPlane3D::intersectAnglePlane(const SrPlane3D& plane) const
 {
	 //Pi - arccos( (n1*n2) / (||n1|| * ||n2||)
	 return SrPiF64 - SrMath::acos(mNormal.dot(plane.mNormal)/(mNormal.magnitude()*plane.mNormal.magnitude()));
 }

 int SrPlane3D::intersectLine(const SrLine3D& line,SrPoint3D& result) const
 {
	 SrReal denom = line.mDirection.dot(mNormal);
	 if( EQUAL(denom,0) )
	 {
		 if( EQUAL(mNormal.dot(line.mBase)+mD,0) )
			 return SR_OVERLAPPING;
		 return SR_PARALLEL;
	 }
	 else
	 {
		 SrReal t = -(mD + mNormal.dot(line.mBase)) / denom;
		 result = line.mBase + t*line.mDirection;
	 }
	 return SR_INTERSECTING;
 }

 int SrPlane3D::intersectRay(const SrRay3D& ray,SrPoint3D& result) const
 {
	 SrReal denom = ray.mDirection.dot(mNormal);
	 if( EQUAL(denom,0) )
	 {
		 if( EQUAL(mNormal.dot(ray.mBase)+mD,0) )
			 return SR_OVERLAPPING;
		 return SR_PARALLEL;
	 }
	 else
	 {
		 SrReal t = -(mD + mNormal.dot(ray.mBase)) / denom;
		 if( LESS(t,0) )
			 return SR_DISJOINT;
		 result = ray.mBase + t*ray.mDirection;
	 }
	 return SR_INTERSECTING;
 }

 int SrPlane3D::intersectSegment(const SrSegment3D& segment,SrPoint3D& result) const
 {
	 SrVector3D direction = segment.mPoint2 - segment.mPoint1;
	 SrReal denom = direction.dot(mNormal);
	 if( EQUAL(denom,0) )
	 {
		 if( EQUAL(mNormal.dot(segment.mPoint1)+mD,0) )
			 return SR_OVERLAPPING;
		 return SR_PARALLEL;
	 }
	 else
	 {
		 SrReal t = -(mD + mNormal.dot(segment.mPoint1)) / denom;
		 if( LESS(t,0) || GREATER(t,1) )
			 return SR_DISJOINT;
		 result = segment.mPoint1 + t*direction;
	 }
	 return SR_INTERSECTING;
 }

 bool SrPlane3D::initPlane(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D&p2)
 {
	 SrVector3D norm = (p1-p0).cross(p2-p0);
	 if( EQUAL(norm.x,0)&&EQUAL(norm.y,0)&&EQUAL(norm.z,0) )
		 return false;
	 mNormal = norm;
	 mD = -mNormal.dot(p0);
	 return true;
 }

 bool SrPlane3D::initPlane(const SrVector3D& direction,const SrPoint3D& point)
 {
	 if( EQUAL(direction.x,0)&&EQUAL(direction.y,0)&&EQUAL(direction.z,0) )
		 return false;
	 mNormal = direction;
	 mD = -mNormal.dot(point);

	 return true;
 }

 bool SrPlane3D::initPlane(const SrVector3D& normal ,SrReal d)
 {
	 if( EQUAL(normal.x,0)&&EQUAL(normal.y,0)&&EQUAL(normal.z,0) )
		 return false;
	 mNormal = normal;
	 mD = d;
	 return true;
 }

 bool	SrPlane3D::isOnPlane(const SrPoint3D& p)const
 {
	 return EQUAL(p.dot(mNormal)+mD,0)?true:false;
 }

 bool SrPlane3D::intersectPlane(const SrPlane3D& plane,SrLine3D& line)const
 {
	 SrVector3D direction = mNormal.cross(plane.mNormal);
	 if( EQUAL(direction.x,0)&&EQUAL(direction.y,0)&&EQUAL(direction.z,0) )
		 return false;
	 direction.normalize();
	 line.mDirection = direction;
	 SrReal s1 = mD , s2 = plane.mD;
	 SrReal n1n2dot = mNormal.dot(plane.mNormal);
	 SrReal n1sqr = mNormal.dot(mNormal);
	 SrReal n2sqr = plane.mNormal.dot(plane.mNormal);
	 SrReal denom = n1n2dot*n1n2dot - n1sqr * n2sqr;
	 SrReal a = (s1 * n2sqr - s2*n1n2dot ) / denom;
	 SrReal b = (s2 * n1sqr - s1*n1n2dot ) / denom;
	 line.mBase = a*mNormal + b*plane.mNormal;

	 return true;
 }


 bool SrPlane3D::initPlane(const SrPoint3D* point,int numPoint)
 {
	 SrVector3D normal(0,0,0);
	 if( !computeNormal(point,numPoint,normal) )
		 return false;
	 SrPoint3D avePoint = SrPoint3D(0,0,0);
	 int i;
	 for( i=0 ; i<numPoint ; i++ )
	 {
		 avePoint += point[i];
	 }
	 avePoint /= numPoint;
	 mNormal = normal;
	 mD = -mNormal.dot(avePoint);
	 return true;
 }