/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#include "SrLine3D.h"
#include "SrPrimitive.h"
#pragma warning (disable:4244)


SrLine3D::SrLine3D()
{
	mDirection = SrVector3D(0,0,0);
	mBase = SrVector3D(0,0,0);
}

SrLine3D::SrLine3D(const SrPoint3D& p1,const SrPoint3D& p2)
{
	mBase = p1;
	mDirection = p2 - p1;
	mDirection.normalize();
}

SrLine3D::~SrLine3D()
{

}

bool SrLine3D::isValid()const
{
	if( EQUAL(mDirection.magnitudeSquared(),1.0f) )
		return true;
	return false;
}


SrReal SrLine3D::toPointDistance(const SrPoint3D& p)const
{
	SrReal t = mDirection.dot(p-mBase);
	return (mBase + t*mDirection - p).magnitude();
}
	
void SrLine3D::rotateX(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0f);
	mBase.set(mBase.x,mBase.y*c-mBase.z*s,mBase.y*s+mBase.z*c);
	mDirection.set(mDirection.x,mDirection.y*c-mDirection.z*s,mDirection.y*s+mDirection.z*c);
}

void SrLine3D::rotateY(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mBase.set(mBase.x*c+mBase.z*s,mBase.y,-mBase.x*s+mBase.z*c);
	mDirection.set(mDirection.x*c+mDirection.z*s,mDirection.y,-mDirection.x*s+mDirection.z*c);
}

void SrLine3D::rotateZ(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mBase.set(mBase.x*c-mBase.y*s,mBase.x*s+mBase.y*c,mBase.z);
	mDirection.set(mDirection.x*c-mDirection.y*s,mDirection.x*s+mDirection.y*c,mDirection.z);
}

void SrLine3D::translate(const SrVector3D& offset)
{
	mBase.set(mBase.x+offset.x,mBase.y+offset.y,mBase.z+offset.z);
}

SrReal SrLine3D::interAngleLine(const SrLine3D& line,bool lineDirDesired)const
{
	SrReal angle = SrMath::acos(line.mDirection.dot(mDirection));
	if( lineDirDesired && GREATER(angle,SrHalfPiF32) )
	{
		return SrPiF32 - angle;
	}
	return angle;
}

SrReal SrLine3D::distanceLineSquared(const SrLine3D& line)const
{
	SrVector3D u = mBase - line.mBase;
	SrReal a = mDirection.dot(mDirection);
	SrReal b = mDirection.dot(line.mDirection);
	SrReal c = line.mDirection.dot(line.mDirection);
	SrReal d = mDirection.dot(u);
	SrReal e = line.mDirection.dot(u);
	SrReal f = u.dot(u);
	SrReal det = a*c - b*b;
	SrReal s , t;
	if( EQUAL(det,0) )
	{//Arbitrarily choose the base point of line0
		s = 0;
		//Choose largest denominator to minimize floating-point problems.
		//Make sure b<c, because c=1 and b<1.
		t = e / c;
		return f + t*(2*e + t*c);
	}
	SrReal invDet = 1/det;
	s = (b*e - c*d)*invDet;
	t = (a*e - b*d)*invDet;
	return s*(s*a + 2*t*b + 2*d) + t*(t*c + 2*e) + f;
}

SrReal SrLine3D::distanceRaySquared(const SrRay3D& ray)const
{
	SrVector3D u = mBase - ray.mBase;
	SrReal a = mDirection.dot(mDirection);
	SrReal b = mDirection.dot(ray.mDirection);
	SrReal c = ray.mDirection.dot(ray.mDirection);
	SrReal d = mDirection.dot(u);
	SrReal e = ray.mDirection.dot(u);
	SrReal det = a*c - b*b;
	SrReal sNum,sDenom,tNum,tDenom;
	tDenom = sDenom = det;
	if( EQUAL(det,0) )
	{
		tNum = 0;
		sNum = -d;
		sDenom = a;

	}
	else
	{
		sNum = b*e - c*d;
		tNum = a*e - b*d;
	}
	//check t
	if( LESS(tNum,0) )
	{
		tNum = 0;
		sNum = -d;
		sDenom = a;
	}
	// Parameters of nearest points on restricted domain
	SrReal s = 0  , t = 0;
	if( UNEQUAL(sDenom ,0) )
		s = sNum / sDenom ;
	if( UNEQUAL(tDenom ,0) )
		t = tNum / tDenom;

	SrVector3D v = u + s*mDirection - t*ray.mDirection;
	return v.dot(v);
}

SrReal SrLine3D::distanceSegmentSquared(const SrSegment3D& segment)const
{
	SrVector3D direction = segment.mPoint2 - segment.mPoint1;
	SrVector3D u = mBase - segment.mPoint1;
	SrReal a = mDirection.dot(mDirection);
	SrReal b = mDirection.dot(direction);
	SrReal c = direction.dot(direction);
	SrReal d = mDirection.dot(u);
	SrReal e = direction.dot(u);
	SrReal det = a*c - b*b;
	SrReal sNum,sDenom,tNum,tDenom;
	tDenom = sDenom = det;
	if( EQUAL(det,0) )
	{
		sNum = 0;
		tNum = e;
		tDenom = c;

	}
	else
	{
		sNum = b*e - c*d;
		tNum = a*e - b*d;
	}

	if( LESS(tNum,0) )
	{
		tNum = 0;
		sNum = -d;
		sDenom = a;

	}
	else if( GREATER(tNum,tDenom) )
	{
		tNum = tDenom;
		sNum =  b - d;
		sDenom = a;
	}
	// Parameters of nearest points on restricted domain
	SrReal s = 0  , t = 0;
	if( UNEQUAL(sDenom ,0) )
		s= sNum / sDenom ;
	if( UNEQUAL(tDenom ,0) )
		t = tNum / tDenom;
	SrVector3D v = u + s*mDirection - t*direction;
	return v.dot(v);
}

