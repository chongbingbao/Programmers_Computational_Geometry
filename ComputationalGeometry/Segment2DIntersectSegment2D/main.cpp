/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/04/20
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"

#include <time.h>
#include <stdio.h>


/**
\brief 2D segment class.

This is a 2D segment class with public data members,two endpoints.
*/
class SrSegment2D
{
public:
	/**
	\brief Default constructor, the two endpoints is set to (0,0).
	*/
	SrSegment2D()
	{
		mPoint1.set(0,0);
		mPoint2.set(0,0);
	}
	/**
	\brief The segment is initialized by two points.
	*/
	SrSegment2D(const SrPoint2D& p1,const SrPoint2D& p2)
	{
		mPoint1 = p1;
		mPoint2 = p2;
	}
	/*
	*\brief  Determine the relation of the point and the segment.
	*\return SR_LINEAR_ON			if on the segment.
			 SR_LINEAR_ABSENT		or else.
	*/
	int pointLocation(const SrPoint2D& p)const
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
	/**
	\brief  The segment is valid if the two endpoints aren't the same.
	*/
	bool isValid()const
	{
		if( EQUAL(mPoint1.x,mPoint2.x)&&EQUAL(mPoint1.y,mPoint2.y))
			return false;
		return true;
	}
public:
	SrPoint2D	mPoint1;
	SrPoint2D	mPoint2;
};


#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D
#define Segment2D	SrSegment2D

#define EPS			SR_EPS

/*
*\brief  Determine the relation of the two segments.
*\return SR_INTERSECTING		if intersecting.
		 SR_SEPARATING			if not intersecting.
*/
int SegmentIntersectSegment_Antonio(const Segment2D& seg1,const Segment2D& seg2)
{
	Vector2D A = seg1.mPoint2 - seg1.mPoint1;
	Vector2D B = seg2.mPoint1 - seg2.mPoint2;
	Vector2D C = seg1.mPoint1 - seg2.mPoint1;
	Real a = B.y*C.x - B.x*C.y;
	Real b;
	Real c = A.y*B.x - A.x*B.y;
	if(c > 0)
	{
		if(a < 0 || a > c)
			return SR_DISJOINT;
		b = A.x*C.y - A.y*C.x;
		if(b < 0 || b > c)
			return SR_DISJOINT;
		return SR_INTERSECTING;
	}
	else if (c < 0)
	{
		if(a > 0 || a < c)
			return SR_DISJOINT;
		b = A.x*C.y - A.y*C.x;
		if(b > 0 || b < c)
			return SR_DISJOINT;
		return SR_INTERSECTING;
	}
	else
	{
		if(fabs(a) >= EPS)
			return SR_DISJOINT;

		SrReal denom , t0, t1;
		if(fabs(A.x) > fabs(A.y))
		{
			t0 = seg2.mPoint1.x - seg1.mPoint1.x;
			t1 = seg2.mPoint2.x - seg1.mPoint1.x;
			denom = A.x;
		}
		else
		{
			t0 = seg2.mPoint1.y - seg1.mPoint1.y;
			t1 = seg2.mPoint2.y - seg1.mPoint1.y;
			denom = A.y;
		}
		if(denom < 0)
		{
			denom = -denom;
			t0    = -t0;
			t1	  = -t1;
		}
		Real tmp;
		if(t0 > t1)
		{
			tmp = t0;
			t0 = t1;
			t1 = tmp;
		}
		if(t0 > denom || t1 < 0)
			return SR_DISJOINT;
		return SR_INTERSECTING;
	}
	return SR_DISJOINT;
}

/*
*\brief  Determine the relation of the two segments.
*\return SR_INTERSECTING		if intersecting.
		 SR_SEPARATING		if not intersecting.
*/
int SegmentIntersectSegment_Parameter(const Segment2D& seg1,const Segment2D& seg2)
{
	Vector2D direction1 = seg1.mPoint2 - seg1.mPoint1;
	Vector2D direction2 = seg2.mPoint2 - seg2.mPoint1;

	Real dir1Square = direction1.magnitudeSquared();
	Real dir2Square = direction2.magnitudeSquared();
	ASSERT(GREATER(dir1Square,0) && GREATER(dir2Square,0) );
	Real kcross = direction1.cross(direction2);
	Vector2D e = seg2.mPoint1 - seg1.mPoint1;

	if((kcross*kcross) > EPS * EPS * dir1Square * dir2Square  )
	{//The intersection angle is not 0 based on relative error.||Cross(d1,d2)||^2/ (||d1||^2*||d2||^2)<=sin(a).
		Real s = e.cross(direction2) / kcross;
		if(s < 0|| s > 1)
			return SR_DISJOINT;
		Real t = e.cross(direction1) / kcross;
		if(t < 0 || t > 1)
			return SR_DISJOINT;
		return SR_INTERSECTING;
	}

	kcross = e.cross(direction1);
	Real eSquare = e.magnitudeSquared(); 
	if(eSquare > 0 && (kcross * kcross) / (eSquare * dir1Square) > EPS * EPS)
	{
		return SR_DISJOINT;
	}
	//The two segments are on the same line.
	Real denom , t0, t1;
	if(fabs(direction1.x) > fabs(direction1.y))
	{
		t0 = seg2.mPoint1.x - seg1.mPoint1.x;
		t1 = seg2.mPoint2.x - seg1.mPoint1.x;
		denom = direction1.x;
	}
	else
	{
		t0 = seg2.mPoint1.y - seg1.mPoint1.y;
		t1 = seg2.mPoint2.y - seg1.mPoint1.y;
		denom = direction1.y;
	}
	if(denom < 0)
	{
		denom = -denom;
		t0    = -t0;
		t1	  = -t1;
	}
	Real tmp;
	if(t0 > t1)
	{
		tmp = t0;
		t0	= t1;
		t1	= tmp;
	}
	if(t0 > denom || t1 < 0)
		return SR_DISJOINT;
	return SR_INTERSECTING;
}

/*
*\brief  Determine the relation of the two segments.
*\return SR_INTERSECTING		if intersecting.
		 SR_SEPARATING			if not intersecting.
*/
int SegmentIntersectSegment_Corment(const Segment2D& seg1,const Segment2D& seg2)
{
	Vector2D direction1 = seg1.mPoint2 - seg1.mPoint1;
	Vector2D direction2 = seg2.mPoint2 - seg2.mPoint1;
	Real d1 = direction1.cross(seg2.mPoint1 - seg1.mPoint1);
	Real d2 = direction1.cross(seg2.mPoint2 - seg1.mPoint1);
	Real d3 = direction2.cross(seg1.mPoint1 - seg2.mPoint1);
	Real d4 = direction2.cross(seg1.mPoint2 - seg2.mPoint1);
	if( ((d1 < 0 && d2 > 0) || (d1 > 0 && d2 < 0)) &&
		((d3 < 0 && d4 > 0) || (d3 > 0 && d4 < 0)))
		return SR_INTERSECTING;
	else if( fabs(d1) < EPS && seg1.pointLocation(seg2.mPoint1) == SR_LINEAR_ON )
		return SR_INTERSECTING;
	else if( fabs(d2) < EPS && seg1.pointLocation(seg2.mPoint2)==SR_LINEAR_ON )
		return SR_INTERSECTING;
	else if( fabs(d3) < EPS && seg2.pointLocation(seg1.mPoint1)==SR_LINEAR_ON )
		return SR_INTERSECTING;
	else if( fabs(d4) < EPS && seg2.pointLocation(seg1.mPoint2)==SR_LINEAR_ON )
		return SR_INTERSECTING;

	return SR_DISJOINT;
}

void TestSegmentIntersectionSegment()
{
	int numCase = 1000000;
	Segment2D* seg1 = new Segment2D[numCase];
	Segment2D* seg2 = new Segment2D[numCase];
	int i ;
	for( i=0 ; i<numCase ; i++ )
	{
		do 
		{
			seg1[i].mPoint1.x = rand()%100;
			seg1[i].mPoint1.y = rand()%100;
			seg1[i].mPoint2.x = rand()%100;
			seg1[i].mPoint2.y = rand()%100;
		} while (!seg1[i].isValid());
		do 
		{
			seg2[i].mPoint1.x = rand()%100;
			seg2[i].mPoint1.y = rand()%100;
			seg2[i].mPoint2.x = rand()%100;
			seg2[i].mPoint2.y = rand()%100;
		} while (!seg2[i].isValid());
	}
	//Check the correctness of the algorithms
	for( i=0 ; i<numCase ; i++ )
	{
		int status1 = SegmentIntersectSegment_Antonio(seg1[i],seg2[i]);
		int status2 = SegmentIntersectSegment_Corment(seg1[i],seg2[i]);
		int status3 = SegmentIntersectSegment_Parameter(seg1[i],seg2[i]);
		ASSERT(status1 == status2);
		ASSERT(status2 == status3);
	}
	printf("The Parameter Method:\n");
	double seconds = clock();
	for( i=0 ; i<numCase ; i++ )
	{
		SegmentIntersectSegment_Parameter(seg1[i],seg2[i]);
	}
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("Time:	%.6lf\n",seconds);


	printf("The Corment Method:\n");
	seconds = clock();
	for( i=0 ; i<numCase ; i++ )
	{
		SegmentIntersectSegment_Corment(seg1[i],seg2[i]);
	}
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("Time:	%.6lf\n",seconds);

	printf("The Antonio Method:\n");
	seconds = clock();
	for( i=0 ; i<numCase ; i++ )
	{
		SegmentIntersectSegment_Antonio(seg1[i],seg2[i]);
	}
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("Time:	%.6lf\n",seconds);
	delete []seg1;
	delete []seg2;
}

int main( )
{
	TestSegmentIntersectionSegment();
	return 0;
}