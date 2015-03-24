/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/12/14
****************************************************************************/
#include "SrSegment2D.h"
#include "SrPrimitive.h"

SrSegment2D::SrSegment2D()
{
	mPoint1.set(0,0);
	mPoint2.set(0,0);
}

SrSegment2D::SrSegment2D(const SrPoint2D& p1,const SrPoint2D& p2)
{
	mPoint1 = p1;
	mPoint2 = p2;
}

SrSegment2D::~SrSegment2D()
{

}

bool SrSegment2D::isValid()const
{
	if( EQUAL(mPoint1.x,mPoint2.x)&&EQUAL(mPoint1.y,mPoint2.y))
		return false;
	return true;
}

SrReal SrSegment2D::toPointDistanceSquared(const SrPoint2D& p)const
{
	SrVector2D d = mPoint2 - mPoint1;
	SrVector2D pp1 = p - mPoint1;
	SrReal t = d.dot(pp1);
	//t<=0
	if( LEQUAL(t,0) )
	{
		//t<=0.Minimum distance is from p to point1.
		return pp1.dot(pp1);
	}
	SrReal dDotd = d.dot(d);
	//The division can be deferred until absolutely needed.
	if( GEQUAL(t,dDotd) )
	{
		//t>=1,Minimum distance is from p to point2.
		SrVector2D pp2 = p - mPoint2;
		return pp2.dot(pp2);
	}
	//Intersection point is on the segment.
	return pp1.dot(pp1) - t*t / dDotd;
}

SrReal SrSegment2D::toPointDistance(const SrPoint2D& p)const
{
	return SrMath::sqrt(toPointDistanceSquared(p));
}

void SrSegment2D::rotate(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mPoint1.set( mPoint1.x*c-mPoint1.y*s , mPoint1.x*s+mPoint1.y*c);
	mPoint2.set( mPoint2.x*c-mPoint2.y*s , mPoint2.x*s+mPoint2.y*c);
}

void SrSegment2D::translate(const SrVector2D& offset)
{
	mPoint1.set( mPoint1.x + offset.x , mPoint1.y + offset.y);
	mPoint2.set( mPoint2.x + offset.x , mPoint2.y + offset.y);
}

int SrSegment2D::intersectLine(const SrLine2D& line,SrPoint2D& result)const
{
	return line.intersectSegment(*this,result);
}

int SrSegment2D::intersectRay(const SrRay2D& ray,SrPoint2D& result)const
{
	return ray.intersectSegment(*this,result);
}

int SrSegment2D::pointLocation(const SrPoint2D& p)const
{
	SrReal d = (mPoint2 - mPoint1).cross(p - mPoint1);
	if( !EQUAL(d,0) )
		return SR_LINEAR_ABSENT;
	SrReal minX , maxX, minY, maxY;
	if( GREATER(mPoint1.x,mPoint2.x) )
	{
		minX = mPoint2.x;
		maxX = mPoint1.x;
	}
	else
	{
		minX = mPoint1.x;
		maxX = mPoint2.x;
	}
	if( GREATER(mPoint1.y,mPoint2.y) )
	{
		minY = mPoint2.y;
		maxY = mPoint1.y;
	}
	else
	{
		minY = mPoint1.y;
		maxY = mPoint2.y;
	}
	if( LESS(p.x,minX) || GREATER(p.x,maxX) || LESS(p.y,minY) || GREATER(p.y,maxY) )
		return SR_LINEAR_ABSENT;
	return SR_LINEAR_ON;
}


int SrSegment2D::intersectSegment(const SrSegment2D& segment,SrPoint2D& result)const
{
	SrVector2D direction1 = mPoint2 - mPoint1;
	SrVector2D direction2 = segment.mPoint2 - segment.mPoint1;

	SrReal dir1Square = direction1.magnitudeSquared();
	SrReal dir2Square = direction2.magnitudeSquared();
	ASSERT(GREATER(dir1Square,0) && GREATER(dir2Square,0) );
	SrReal kcross = direction1.cross(direction2);
	SrVector2D e = segment.mPoint1 - mPoint1;

	if( (kcross*kcross)>SR_EPS*SR_EPS*dir1Square*dir2Square  )
	{//The intersection angle is not 0 based on relative error.||Cross(d1,d2)||^2/ (||d1||^2*||d2||^2)<=sin(a).
		SrReal s = e.cross(direction2) / kcross;
		if( LESS(s,0) || GREATER(s,1) )
			return SR_DISJOINT;
		SrReal t = e.cross(direction1) / kcross;
		if( LESS(t,0) || GREATER(t,1) )
			return SR_DISJOINT;
		result = mPoint1 + s*direction1;

		return SR_INTERSECTING;
	}

	kcross = e.cross(direction1);
	SrReal eSquare = e.magnitudeSquared(); 
	if( GREATER(eSquare,0) && (kcross*kcross)/(eSquare*dir1Square)>SR_EPS*SR_EPS )
	{
		return SR_PARALLEL;
	}
	//The two segments are on the same line.
	SrReal denom , t0, t1;
	if( GREATER(fabs(direction1.x),fabs(direction1.y)) )
	{
		t0 = segment.mPoint1.x - mPoint1.x;
		t1 = segment.mPoint2.x - mPoint1.x;
		denom = direction1.x;
	}
	else
	{
		t0 = segment.mPoint1.y - mPoint1.y;
		t1 = segment.mPoint2.y - mPoint1.y;
		denom = direction1.y;
	}
	if( LESS(denom,0) )
	{
		denom = -denom;
		t0    = -t0;
		t1	  = -t1;
	}
	SrReal tmp;
	if( t0>t1 )
	{
		tmp = t0;
		t0 = t1;
		t1 = tmp;
	}
	if( GREATER(t0,denom) || LESS(t1,0) )
		return SR_DISJOINT;
	else if( EQUAL(t1,0) )
	{
		result = mPoint1;
		return SR_INTERSECTING;
	}
	else if(  EQUAL(t0,1) )
	{
		result = mPoint2;
		return SR_INTERSECTING;
	}

	return SR_OVERLAPPING;
}

int SrSegment2D::intersectSegment(const SrSegment2D& segment)const
{
	SrVector2D A = mPoint2 - mPoint1;
	SrVector2D B = segment.mPoint1 - segment.mPoint2;
	SrVector2D C = mPoint1 - segment.mPoint1;
	SrReal a = B.y*C.x - B.x*C.y;
	SrReal b;
	SrReal denom = A.y*B.x - A.x*B.y;
	if( GREATER( denom,0 ) )
	{
		if( LESS(a,0) || GREATER(a,denom) )
			return SR_DISJOINT;
		b = A.x*C.y - A.y*C.x;
		if( LESS(b,0) || GREATER(b,denom) )
			return SR_DISJOINT;
		return SR_INTERSECTING;
	}
	else if (LESS( denom,0 ))
	{
		if( GREATER(a,0) || LESS(a,denom) )
			return SR_DISJOINT;
		b = A.x*C.y - A.y*C.x;
		if( GREATER(b,0) || LESS(b,denom) )
			return SR_DISJOINT;
		return SR_INTERSECTING;
	}
	else
	{
		if( !EQUAL(a,0) )
			return SR_DISJOINT;

		SrReal denom , t0, t1;
		if( GREATER(fabs(A.x),fabs(A.y)) )
		{
			t0 = segment.mPoint1.x - mPoint1.x;
			t1 = segment.mPoint2.x - mPoint1.x;
			denom = A.x;
		}
		else
		{
			t0 = segment.mPoint1.y - mPoint1.y;
			t1 = segment.mPoint2.y - mPoint1.y;
			denom = A.y;
		}
		if( LESS(denom,0) )
		{
			denom = -denom;
			t0    = -t0;
			t1	  = -t1;
		}
		SrReal tmp;
		if( GREATER(t0,t1) )
		{
			tmp = t0;
			t0 = t1;
			t1 = tmp;
		}
		if( GREATER(t0,denom) || LESS(t1,0) )
			return SR_DISJOINT;
		else if( EQUAL(t1,0) || EQUAL(t0,1) )
			return SR_INTERSECTING;

		return SR_INTERSECTING;
	}
	return SR_DISJOINT;
}