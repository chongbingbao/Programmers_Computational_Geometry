#include "SrTriangle2D.h"

SrTriangle2D::SrTriangle2D()
{
	mPoint[0] = mPoint[1] = mPoint[1] = SrPoint2D(0,0);
}

SrTriangle2D::SrTriangle2D(const SrPoint2D& p0,const SrPoint2D& p1,const SrPoint2D& p2)
{
	mPoint[0] = p0;
	mPoint[1] = p1;
	mPoint[2] = p2;
}

SrTriangle2D::~SrTriangle2D()
{

}

SrReal	SrTriangle2D::toPointDistance(const SrPoint2D& p) const
{
	SrSegment2D seg0(mPoint[0],mPoint[2]);
	SrReal minDist = seg0.toPointDistance(p),tmpDist;
	SrSegment2D seg1(mPoint[0],mPoint[1]);
	tmpDist = seg1.toPointDistance(p);
	if( LESS(tmpDist,minDist) )
		minDist = tmpDist;
	SrSegment2D seg2(mPoint[1],mPoint[2]);
	tmpDist = seg2.toPointDistance(p);
	if( LESS(tmpDist,minDist) )
		minDist = tmpDist;
	return minDist;
}

bool	SrTriangle2D::isValid() const
{
	if( EQUAL(area(),0) )
		return false;
	return true;
}

SrReal	SrTriangle2D::area() const
{
	return fabs((mPoint[1] - mPoint[0]).cross(mPoint[2] - mPoint[0]));
}

int		SrTriangle2D::pointLocation(const SrPoint2D& p) const
{
	SrReal d0 = (mPoint[1] - mPoint[0]).cross(p - mPoint[0]);
	SrReal d1 = (mPoint[2] - mPoint[1]).cross(p - mPoint[1]);
	SrReal d2 = (mPoint[0] - mPoint[2]).cross(p - mPoint[2]);
	SrReal s0 = d0*d1, s1 = d1*d2;

	if( LESS(s0,0) || LESS(s1,0) )
		return SR_POSITION_OUTSIDE;
	else if( EQUAL(s0,0) || EQUAL(s1,0) )
		return SR_POSITION_ON;

	return SR_POSITION_INSIDE;
}

SrReal	SrTriangle2D::perimeter() const
{
	return (mPoint[1]-mPoint[0]).magnitude() + 
		   (mPoint[2]-mPoint[1]).magnitude() +
		   (mPoint[0]-mPoint[2]).magnitude();
}

void SrTriangle2D::rotate(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	int i;
	for( i=0 ; i<3 ; i++ )
		mPoint[i].set( mPoint[i].x*c-mPoint[i].y*s , mPoint[i].x*s+mPoint[i].y*c);

}

void SrTriangle2D::translate(const SrVector2D& offset)
{
	int i;
	for( i=0 ; i<3 ; i++ )
		mPoint[i].set( mPoint[i].x + offset.x , mPoint[i].y + offset.y);

}

int SrTriangle2D::intersectTriangle(const SrTriangle2D& triangle)const
{
	int i , j;
	SrSegment2D seg1 , seg2;
	for( i=0 ; i<3 ; i++ )
	{
		seg1.mPoint1 = mPoint[i];
		seg1.mPoint2 = mPoint[(i+1)%3];
		for( j=0 ; j<3 ; j++ )
		{
			seg2.mPoint1 = triangle.mPoint[i];
			seg2.mPoint2 = triangle.mPoint[(i+1)%3];
			if( seg1.intersectSegment(seg2) == SR_INTERSECTING )
				return SR_INTERSECTING;
		}
	}
	if( pointLocation(triangle.mPoint[0]) != SR_POSITION_OUTSIDE )
		return SR_INTERSECTING;
	else if( triangle.pointLocation(mPoint[0])!=SR_POSITION_OUTSIDE )
		return SR_INTERSECTING;
	return SR_DISJOINT;
}