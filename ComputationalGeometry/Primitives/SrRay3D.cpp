/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#include "SrRay3D.h"
#include "SrPrimitive.h"
#pragma warning (disable:4244)


SrRay3D::SrRay3D()
{
	mDirection = SrVector3D(0,0,0);
	mBase = SrVector3D(0,0,0);
}


SrRay3D::SrRay3D(const SrPoint3D& p1,const SrPoint3D& p2)
{
	mBase = p1;
	mDirection = p2 - p1;
	mDirection.normalize();
}

SrRay3D::~SrRay3D()
{

}


	
bool SrRay3D::isValid()const
{
	if( EQUAL(mDirection.magnitudeSquared(),1.0f) )
		return true;
	return false;
}
	
SrReal	SrRay3D::toPointDistance(const SrPoint3D& p)const
{
	SrReal t = mDirection.dot(p-mBase);
	//t<=0
	if( LEQUAL(t,0) )
	{
		return (p-mBase).magnitude();
	}
	return (mBase + t*mDirection - p).magnitude();
}

	
void SrRay3D::rotateX(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mBase.set(mBase.x,mBase.y*c-mBase.z*s,mBase.y*s+mBase.z*c);
	mDirection.set(mDirection.x,mDirection.y*c-mDirection.z*s,mDirection.y*s+mDirection.z*c);

}
	
void SrRay3D::rotateY(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mBase.set(mBase.x*c+mBase.z*s,mBase.y,-mBase.x*s+mBase.z*c);
	mDirection.set(mDirection.x*c+mDirection.z*s,mDirection.y,-mDirection.x*s+mDirection.z*c);

}
	
void SrRay3D::rotateZ(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mBase.set(mBase.x*c-mBase.y*s,mBase.x*s+mBase.y*c,mBase.z);
	mDirection.set(mDirection.x*c-mDirection.y*s,mDirection.x*s+mDirection.y*c,mDirection.z);

}

void SrRay3D::translate(const SrVector3D& offset)
{
	mBase.set(mBase.x+offset.x,mBase.y+offset.y,mBase.z+offset.z);
}

	
SrReal SrRay3D::distanceLineSquared(const SrLine3D& line)const
{
	return line.distanceRaySquared(*this);
}

SrReal SrRay3D::distanceRaySquared(const SrRay3D& ray)const
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
	//check t
	if( LESS(tNum,0) )
	{
		tNum = 0;
		if( LESS(-d,0) )
		{
			sNum = 0;
		}
		else
		{
			sNum = -d;
			sDenom = a;
		}
	}
	// Parameters of nearest points on restricted domain
	SrReal s = 0  , t = 0;
	if( UNEQUAL(sDenom,0) )
		s= sNum / sDenom ;
	if( UNEQUAL(tDenom,0) )
		t = tNum / tDenom;
	SrVector3D v = u + s*mDirection - t*ray.mDirection;
	return v.dot(v);
}

SrReal SrRay3D::distanceSegmentSquared(const SrSegment3D& segment)const
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
	//check s
	if( LESS(sNum,0) )
	{
		sNum = 0;
		tNum = e;
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
		else
		{
			sNum = -d;
			sDenom  = a;
		}
	}
	else if( GREATER(tNum,tDenom) )
	{
		tNum = tDenom;
		if( LESS((-d + b),0) )
			sNum = 0;
		else
		{
			sNum = -d + b;
			sDenom = a;
		}
	}
	// Parameters of nearest points on restricted domain
	SrReal s = 0  , t = 0;
	if( UNEQUAL(sDenom,0) )
		s= sNum / sDenom ;
	if( UNEQUAL(tDenom,0) )
		t = tNum / tDenom;

	SrVector3D v = u + s*mDirection - t*direction;
	return v.dot(v);
}