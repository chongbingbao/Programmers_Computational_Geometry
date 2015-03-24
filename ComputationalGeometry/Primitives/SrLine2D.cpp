/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#include "SrLine2D.h"
#include "SrPrimitive.h"


SrLine2D::SrLine2D()
{
	mBase.set(0,0);
	mDirection.set(0,0);
}

SrLine2D::SrLine2D(const SrPoint2D& p1,const SrPoint2D& p2)
{
	mBase = p1;
	mDirection = p2 - p1;
	mDirection.normalize();
}

SrLine2D::~SrLine2D()
{

}

bool SrLine2D::isValid()const
{
	if( EQUAL(mDirection.magnitudeSquared(),1.0f) )
		return true;
	return false;
}

SrReal SrLine2D::toPointDistanceSquared(const SrPoint2D& p)const
{
	SrVector2D tmp = p - mBase;
	SrReal tmpDot = mDirection.dot(tmp);
	return tmp.dot(tmp)-tmpDot*tmpDot;
}

SrReal SrLine2D::toPointDistance(const SrPoint2D& p)const
{
	return SrMath::sqrt(toPointDistanceSquared(p));
}

void SrLine2D::rotate(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0f) , s =SrMath::sin(angle*SrPiF64/180.0);
	mBase.set( mBase.x*c-mBase.y*s , mBase.x*s+mBase.y*c);
	mDirection.set( mDirection.x*c-mDirection.y*s , mDirection.x*s+mDirection.y*c);
}

void SrLine2D::translate(const SrVector2D& offset)
{
	mBase.set( mBase.x + offset.x , mBase.y + offset.y);
}

int SrLine2D::intersectLine(const SrLine2D& line,SrPoint2D& result)const
{
	SrReal kcross = mDirection.cross(line.mDirection);
	SrVector2D e = line.mBase - mBase;
	if( GREATER(kcross,0) )
	{//If the intersection angle is not 0,the two lines intersect.
		SrReal s = e.cross(line.mDirection) / kcross;
		result = mBase + s*mDirection;
		return SR_INTERSECTING;
	}
	kcross = e.cross(mDirection);
	SrReal eLenSquare = e.magnitudeSquared();
	//It's possible that line.mBase and mBase are the same point.
	if( EQUAL(eLenSquare,0) || (kcross*kcross)<SR_EPS*SR_EPS*eLenSquare )
	{//Overlapping.
		return SR_OVERLAPPING;
	}
	//Parallel,but not overlapping.
	return SR_PARALLEL;
}

int SrLine2D::intersectRay(const SrRay2D& ray,SrPoint2D& result)const
{
	SrReal kcross = ray.mDirection.cross(mDirection);
	SrVector2D e = mBase - ray.mBase;
	if( GREATER(kcross,0) )
	{//If the intersection angle is not 0.
		SrReal s = e.cross(mDirection) / kcross;
		if( LESS(s,0) )
			return SR_DISJOINT;
		//Intersecting
		result = ray.mBase + s*ray.mDirection;
		return SR_INTERSECTING;
	}
	kcross = e.cross(ray.mDirection);
	SrReal eLenSquare = e.magnitudeSquared(); 
	//It's possible that line.mBase and mBase are the same point.
	if( EQUAL(eLenSquare,0) ||  (kcross*kcross)<SR_EPS*SR_EPS*eLenSquare )
	{//Overlapping
		return SR_OVERLAPPING;
	}
	//Parallel
	return SR_PARALLEL;
}

int SrLine2D::intersectSegment(const SrSegment2D& segment,SrPoint2D& result)const
{
	SrVector2D dir = segment.mPoint2 - segment.mPoint1;
	SrReal dirLenSquare = dir.magnitudeSquared();
	SrReal kcross = dir.cross(mDirection);
	SrVector2D e = mBase - segment.mPoint1;
	if( (kcross*kcross)/dirLenSquare>=SR_EPS*SR_EPS )
	{//The intersection angle is not 0 based on relative error.||Cross(d1,d2)||^2/ (||d1||^2*||d2||^2)<=sin(a).
		SrReal s = e.cross(mDirection) / kcross;
		//If s<0 and s>1, intersected point isn't on the segment.
		if( LESS(s,0) || GREATER(s,1.0f) )
			return SR_DISJOINT;
		//Intersecting.
		result = segment.mPoint1 + s*dir;
		return SR_INTERSECTING;
	}
	kcross = e.cross(dir);
	SrReal eLenSquare = e.magnitudeSquared(); 
	if( (kcross*kcross)<SR_EPS*SR_EPS*eLenSquare*dirLenSquare )
	{
		//Overlapping.
		return SR_OVERLAPPING;
	}
	//Parallel.
	return SR_PARALLEL;
}

int SrLine2D::pointLocation(const SrPoint2D& p)const
{
	if( EQUAL(mDirection.cross(p-mBase),0) )
		return SR_LINEAR_ON;
	return SR_LINEAR_ABSENT;
}