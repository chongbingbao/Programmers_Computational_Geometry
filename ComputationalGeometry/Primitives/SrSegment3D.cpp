/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#include "SrSegment3D.h"
#include "SrPrimitive.h"
#pragma warning (disable:4244)


SrSegment3D::SrSegment3D()
{
	mPoint1.set(0,0,0);
	mPoint2.set(0,0,0);
}


SrSegment3D::SrSegment3D(const SrPoint3D& p1,const SrPoint3D& p2)
{
	mPoint1 = p1;
	mPoint2 = p2;
}

SrSegment3D::~SrSegment3D()
{

}


bool SrSegment3D::isValid()const
{
	if( EQUAL(mPoint1.x,mPoint2.x)&&EQUAL(mPoint1.y,mPoint2.y)&&EQUAL(mPoint1.z,mPoint2.z) )
		return false;
	return true;
}


SrReal	SrSegment3D::toPointDistance(const SrPoint3D& p)const
{
	SrVector3D direction = mPoint2 - mPoint1;
	SrReal len = direction.magnitudeSquared();
	SrReal t = direction.dot(p-mPoint1);
	//t<=0
	if( LEQUAL(t,0) )
	{
		return (p-mPoint1).magnitude();
	}
	else if( GEQUAL(t,len) )
	{
		return (p-mPoint2).magnitude();
	}
	return (mPoint1 + t*direction/len - p).magnitude();
}


void SrSegment3D::rotateX(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mPoint1.set(mPoint1.x,mPoint1.y*c-mPoint1.z*s,mPoint1.y*s+mPoint1.z*c);
	mPoint2.set(mPoint2.x,mPoint2.y*c-mPoint2.z*s,mPoint2.y*s+mPoint2.z*c);
}

void SrSegment3D::rotateY(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mPoint1.set(mPoint1.x*c+mPoint1.z*s,mPoint1.y,-mPoint1.x*s+mPoint1.z*c);
	mPoint2.set(mPoint2.x*c+mPoint2.z*s,mPoint2.y,-mPoint2.x*s+mPoint2.z*c);
}

void SrSegment3D::rotateZ(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mPoint1.set(mPoint1.x*c-mPoint1.y*s,mPoint1.x*s+mPoint1.y*c,mPoint1.z);
	mPoint2.set(mPoint2.x*c-mPoint2.y*s,mPoint2.x*s+mPoint2.y*c,mPoint2.z);

}

void SrSegment3D::translate(const SrVector3D& offset)
{
	mPoint1.set(mPoint1.x+offset.x,mPoint1.y+offset.y,mPoint1.z+offset.z);
	mPoint2.set(mPoint2.x+offset.x,mPoint2.y+offset.y,mPoint2.z+offset.z);
}

SrReal SrSegment3D::distanceSegmentSquared(const SrSegment3D& segment)const
{
	SrVector3D dir0 = mPoint2 - mPoint1;
	SrVector3D dir1 = segment.mPoint2 - segment.mPoint1;
	SrVector3D u = mPoint1 - segment.mPoint1;
	SrReal a = dir0.dot(dir0);
	SrReal b = dir0.dot(dir1);
	SrReal c = dir1.dot(dir1);
	SrReal d = dir0.dot(u);
	SrReal e = dir1.dot(u);
	SrReal det = a*c - b*b;
	SrReal sNum,sDenom,tNum,tDenom;

	sDenom = tDenom = det;
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

	//check s
	if( LESS(sNum,0) )
	{
		sNum = 0;
		tNum = e;
		tDenom = c;
	}
	else if( GREATER(sNum,det) )
	{
		sNum = det;
		tNum = e + b;
		tDenom = c;
	}
	//check t
	if( LESS(tNum,0) )
	{
		tNum = 0;
		if( LESS(-d,0) )
		{
			sNum = 0;
		}
		else if( GREATER(-d,a) )
		{
			sNum	= 1.0f;
			sDenom	= 1.0f;
		}
		else
		{
			sNum = -d;
			sDenom = a;
		}
	}
	else if( GREATER(tNum,tDenom) )
	{
		tNum = tDenom;
		if( LESS((-d + b),0) )
		{
			sNum = 0;
		}
		else if( GREATER((-d + b),a) )
		{
			sNum	= 1.0f;
			sDenom	= 1.0f;
		}
		else
		{
			sNum = -d + b;
			sDenom = a;
		}
	}
	//Parameters of nearest points on restricted domain
	SrReal s = 0  , t = 0;
	if( UNEQUAL(sDenom,0) )
		s= sNum / sDenom ;
	if( UNEQUAL(tDenom ,0) )
		t = tNum / tDenom;
	SrVector3D v = u + s*dir0 - t*dir1;
	return v.dot(v);

}

SrReal SrSegment3D::distanceLineSquared(const SrLine3D& line)const
{
	return line.distanceSegmentSquared(*this);
}

SrReal SrSegment3D::distanceRaySquared(const SrRay3D& ray)const
{
	return ray.distanceSegmentSquared(*this);
}