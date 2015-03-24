/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#include "SrRay2D.h"
#include "SrPrimitive.h"


SrRay2D::SrRay2D()
{
	mBase.set(0,0);
	mDirection.set(0,0);
}


SrRay2D::SrRay2D(const SrPoint2D& p1,const SrPoint2D& p2)
{
	mBase = p1;
	mDirection = p2 - p1;
	mDirection.normalize();
}

SrRay2D::~SrRay2D()
{

}

bool SrRay2D::isValid()const
{
	if( EQUAL(mDirection.magnitudeSquared(),1.0) )
		return true;
	return false;
}

SrReal SrRay2D::toPointDistanceSquared(const SrPoint2D& p)const
{
	SrVector2D tmp = p - mBase;
	SrReal tmpDot = mDirection.dot(tmp);
	//tmpDot>0
	if( GREATER(tmpDot,0) )
		return tmp.dot(tmp)-tmpDot*tmpDot;
	//The projection point is not on the ray.
	return tmp.dot(tmp);
}

SrReal SrRay2D::toPointDistance(const SrPoint2D& p)const
{
	return SrMath::sqrt(toPointDistanceSquared(p));
}

void SrRay2D::rotate(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mBase.set( mBase.x*c-mBase.y*s , mBase.x*s+mBase.y*c);
	mDirection.set( mDirection.x*c-mDirection.y*s , mDirection.x*s+mDirection.y*c);
}

void SrRay2D::translate(const SrVector2D& offset)
{
	mBase.set( mBase.x + offset.x , mBase.y + offset.y);
}


int SrRay2D::intersectLine(const SrLine2D& line ,SrPoint2D& result)const
{
	return line.intersectRay(*this,result);
}

int SrRay2D::intersectRay(const SrRay2D& ray, SrPoint2D& result)const
{
	SrReal kcross = mDirection.cross(ray.mDirection);
	SrVector2D e = ray.mBase - mBase;
	SrReal t , s;
	if( GREATER(kcross,0) )
	{//The intersection angle is not 0 based on relative error.||Cross(d1,d2)||^2/ (||d1||^2*||d2||^2)<=sin(a).
		s = e.cross(ray.mDirection) / kcross;
		if( LESS(s,0) )
			return SR_DISJOINT;
		t = e.cross(mDirection) / kcross;
		if( LESS(t,0) )
			return SR_DISJOINT;
		//Intersecting.
		result = mBase + s*mDirection;
		return SR_INTERSECTING;
	}
	kcross = e.cross(ray.mDirection);
	SrReal eLenSquare = e.magnitudeSquared(); 
	if( GREATER(eLenSquare,0) && (kcross*kcross)>=SR_EPS*SR_EPS*eLenSquare )
	{//Parallel.
		return SR_PARALLEL;
	}
	//The two rays are on the same line.
	SrReal dotProduct = e.dot(mDirection);
	bool isIntersecting = false;

	if( GREATER(dotProduct,0) )
		return SR_OVERLAPPING;
	dotProduct = e.dot(ray.mDirection);
	if( LESS(dotProduct,0) )
		return SR_OVERLAPPING;
	else if( EQUAL(dotProduct,0) )
		isIntersecting = true;
	//Make sure that the two rays don't overlap.
	if( isIntersecting )
	{
		result = mBase;
		return SR_INTERSECTING;
	}
	return SR_DISJOINT;
}

int SrRay2D::intersectSegment(const SrSegment2D& segment, SrPoint2D& result)const
{
	SrVector2D dir = segment.mPoint2 - segment.mPoint1;
	SrReal dirLenSquare = dir.magnitudeSquared();
	ASSERT(GREATER(dirLenSquare,0));
	SrReal kcross = dir.cross(mDirection);
	SrVector2D e = mBase - segment.mPoint1;
	if( (kcross*kcross)/dirLenSquare>SR_EPS*SR_EPS  )
	{//The intersection angle is not 0 based on relative error.||Cross(d1,d2)||^2/ (||d1||^2*||d2||^2)<=sin(a).
		SrReal s = e.cross(mDirection) / kcross;
		//If s<0 and s>1, intersecting point isn't on the segment.
		if( LESS(s,0) || GREATER(s,1.0f) )
			return SR_DISJOINT;
		SrReal t = e.cross(dir) / kcross;
		//t<0
		if( LESS(t,0) )
			return SR_DISJOINT;
		//Intersecting.
		result = segment.mPoint1 + s*dir;

		return SR_INTERSECTING;
	}
	kcross = e.cross(dir);
	SrReal eLenSquare = e.magnitudeSquared(); 
	if( GREATER(eLenSquare,0) && (kcross*kcross)/(eLenSquare*dirLenSquare)>=SR_EPS*SR_EPS )
	{//Not intersecting.
		return SR_PARALLEL;
	}
	//The segment and the ray are on the same line.
	bool isIntersecting = false;
	SrReal dotProduct = (mBase - segment.mPoint1).dot(mDirection);

	if( LESS(dotProduct,0) )
	{
		return SR_OVERLAPPING;
	}
	else if( EQUAL(dotProduct,0) )
	{
		isIntersecting = true;
	}
	dotProduct = (mBase - segment.mPoint2).dot(mDirection);
	if( LESS(dotProduct,0) )
	{
		return SR_OVERLAPPING;
	}
	if( EQUAL(dotProduct,0) )
	{
		isIntersecting = true;
	}
	//Make sure that the two segments don't overlap.
	if( isIntersecting )
	{
		result = mBase;
		return SR_INTERSECTING;
	}
	return SR_DISJOINT;
}

int SrRay2D::pointLocation(const SrPoint2D& p)const
{
	SrVector2D tmp = mBase - p;
	SrReal d = tmp.cross(mDirection);
	if( !EQUAL(d,0) )
		return SR_LINEAR_ABSENT;
	d = tmp.dot(mDirection);
	if( GREATER(d,0) )
		return SR_LINEAR_ABSENT;
	return SR_LINEAR_ON;
}
