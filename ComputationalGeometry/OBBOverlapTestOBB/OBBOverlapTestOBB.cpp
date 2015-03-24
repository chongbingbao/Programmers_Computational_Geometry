/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/19
****************************************************************************/
/************************************************************************		
\description	
****************************************************************************/
#include "../include/SrGeometricTools.h"
#include "../include/SrDataType.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

/**
\brief 3D oriented bounding box (OBB) class.
*/
class SrOBBox3D
{
public:
	SrOBBox3D()
	{
		mCenter = SrVector3D(0,0,0);
		mAxis[0] = mAxis[1] = mAxis[2] = SrVector3D(0,0,0);
		mHalfLength[0]	= mHalfLength[1] = mHalfLength[2] = 0;
	}
	SrOBBox3D(const SrPoint3D& c,const SrVector3D* pAxis,const SrReal* pHalfLength)
	{
		mCenter = c;
		mAxis[0] = pAxis[0];
		mAxis[1] = pAxis[1];
		mAxis[2] = pAxis[2];

		mHalfLength[0] = pHalfLength[0];
		mHalfLength[1] = pHalfLength[1];
		mHalfLength[2] = pHalfLength[2];
	}
	/**
	\brief  The OBB is valid if one of the three axis is unit length, 
			the length of them is greater than 0, and the axis should 
			be perpendicular to each other.
	*/
	bool	isValid() const
	{
		if( UNEQUAL(mAxis[0].dot(mAxis[1]),0) ||
			UNEQUAL(mAxis[0].dot(mAxis[2]),0)||
			UNEQUAL(mAxis[1].dot(mAxis[2]),0))
			return false;

		if( LEQUAL(mHalfLength[0],0) || 
			LEQUAL(mHalfLength[1],0) ||
			LEQUAL(mHalfLength[2],0))
			return false;
		if( UNEQUAL(mAxis[0].magnitudeSquared(),1.0) ||
			UNEQUAL(mAxis[1].magnitudeSquared(),1.0) ||
			UNEQUAL(mAxis[2].magnitudeSquared(),1.0) )
			return false;

		return true;
	}

public:
	SrPoint3D	mCenter;
	SrVector3D	mAxis[3];
	SrReal		mHalfLength[3];
};

//Case 1
bool ASeparatingAxis(const SrPoint3D&t , const SrReal aHLen[3],const SrVector3D bFabsAxis[3],const SrReal bHLen[3])
{
	if( GREATER(fabs(t.x), aHLen[0] + bHLen[0]*bFabsAxis[0].x + bHLen[1]*bFabsAxis[1].x + bHLen[2]*bFabsAxis[2].x) )
		return true;
	if( GREATER(fabs(t.y), aHLen[1] + bHLen[0]*bFabsAxis[0].y + bHLen[1]*bFabsAxis[1].y + bHLen[2]*bFabsAxis[2].y) )
		return true;
	if( GREATER(fabs(t.z), aHLen[2] + bHLen[0]*bFabsAxis[0].z + bHLen[1]*bFabsAxis[1].z + bHLen[2]*bFabsAxis[2].z) )
		return true;
	return false;
}

//Case 2
bool BSeparatingAxis(const SrPoint3D&t , const SrReal aHLen[3],const SrVector3D bFabsAxis[3],const SrReal bHLen[3])
{
	if( GREATER(fabs(t.x), aHLen[0]*bFabsAxis[0].x + aHLen[1]*bFabsAxis[0].y + aHLen[2]*bFabsAxis[0].z + bHLen[0]) )
		return true;
	if( GREATER(fabs(t.y), aHLen[0]*bFabsAxis[1].x + aHLen[1]*bFabsAxis[1].y + aHLen[2]*bFabsAxis[1].z + bHLen[1]) )
		return true;
	if( GREATER(fabs(t.z), aHLen[0]*bFabsAxis[2].x + aHLen[1]*bFabsAxis[2].y + aHLen[2]*bFabsAxis[2].z + bHLen[2]) )
		return true;
	return false;
}

bool ABSeparatingAxis(const SrPoint3D&c , const SrReal aHLen[3],const SrVector3D bAxis[3],const SrVector3D bFabsAxis[3],const SrReal bHLen[3])
{
	SrReal s , r;

	//X and bAxis[0]
	s = fabs(-bAxis[0].z*c.y + bAxis[0].y*c.z);
	r = aHLen[1]*bFabsAxis[0].z + aHLen[2]*bFabsAxis[0].y + bHLen[1]*bFabsAxis[2].x + bHLen[2]*bFabsAxis[1].x;
	if( GREATER(s,r) )	
		return true;
	//X and bAxis[1]
	s = fabs(-bAxis[1].z*c.y + bAxis[1].y*c.z);
	r = aHLen[1]*bFabsAxis[1].z + aHLen[2]*bFabsAxis[1].y + bHLen[0]*bFabsAxis[2].x + bHLen[2]*bFabsAxis[0].x;
	if( GREATER(s,r) )	
		return true;
	//X and bAxis[2]
	s = fabs(-bAxis[2].z*c.y + bAxis[2].y*c.z);
	r = aHLen[1]*bFabsAxis[2].z + aHLen[2]*bFabsAxis[2].y + bHLen[0]*bFabsAxis[1].x + bHLen[1]*bFabsAxis[0].x;
	if( GREATER(s,r) )
		return true;

	//Y and bAxis[0]
	s = fabs(bAxis[0].z*c.x - bAxis[0].x*c.z);
	r = aHLen[0]*bFabsAxis[0].z + aHLen[2]*bFabsAxis[0].x + bHLen[1]*bFabsAxis[2].y + bHLen[2]*bFabsAxis[1].y;
	if( GREATER(s,r) )	
		return true;
	//Y and bAxis[1]
	s = fabs(bAxis[1].z*c.x - bAxis[1].x*c.z);
	r = aHLen[0]*bFabsAxis[1].z + aHLen[2]*bFabsAxis[1].x + bHLen[0]*bFabsAxis[2].y + bHLen[2]*bFabsAxis[0].y;
	if( GREATER(s,r) )	
		return true;
	//Y and bAxis[2]
	s = fabs(bAxis[2].z*c.x - bAxis[2].x*c.z);
	r = aHLen[0]*bFabsAxis[2].z + aHLen[2]*bFabsAxis[2].x + bHLen[0]*bFabsAxis[1].y + bHLen[1]*bFabsAxis[0].y;
	if( GREATER(s,r) )	
		return true;

	//Z and bAxis[0]
	s = fabs(-bAxis[0].y*c.x + bAxis[0].x*c.y);
	r = aHLen[0]*bFabsAxis[0].y + aHLen[1]*bFabsAxis[0].x + bHLen[1]*bFabsAxis[2].z + bHLen[2]*bFabsAxis[1].z;
	if( GREATER(s,r) )
		return true;
	//Z and bAxis[1]
	s = fabs(-bAxis[1].y*c.x + bAxis[1].x*c.y);
	r = aHLen[0]*bFabsAxis[1].y + aHLen[1]*bFabsAxis[1].x + bHLen[0]*bFabsAxis[2].z + bHLen[2]*bFabsAxis[0].z;
	if( GREATER(s,r) )	
		return true;
	//Z and bAxis[2]
	s = fabs(-bAxis[2].y*c.x + bAxis[2].x*c.y);
	r = aHLen[0]*bFabsAxis[2].y + aHLen[1]*bFabsAxis[2].x + bHLen[0]*bFabsAxis[1].z + bHLen[1]*bFabsAxis[0].z;
	if( GREATER(s,r) )	
		return true;

	return false;
}

bool	OBBOverlapTestOBB_OptimizedSeparatingAxisMethod(const SrOBBox3D& obbA, const SrOBBox3D& obbB)
{
	SrVector3D bAxis[3], o, c, t;

	o = obbB.mCenter - obbA.mCenter;

	c.x = o.dot(obbA.mAxis[0]);
	c.y = o.dot(obbA.mAxis[1]);
	c.z = o.dot(obbA.mAxis[2]);

	bAxis[0].x = obbA.mAxis[0].dot(obbB.mAxis[0]);
	bAxis[0].y = obbA.mAxis[1].dot(obbB.mAxis[0]);
	bAxis[0].z = obbA.mAxis[2].dot(obbB.mAxis[0]);

	bAxis[1].x = obbA.mAxis[0].dot(obbB.mAxis[1]);
	bAxis[1].y = obbA.mAxis[1].dot(obbB.mAxis[1]);
	bAxis[1].z = obbA.mAxis[2].dot(obbB.mAxis[1]);

	bAxis[2].x = obbA.mAxis[0].dot(obbB.mAxis[2]);
	bAxis[2].y = obbA.mAxis[1].dot(obbB.mAxis[2]);
	bAxis[2].z = obbA.mAxis[2].dot(obbB.mAxis[2]);

	SrVector3D bFabsAxis[3];
	bFabsAxis[0].set(fabs(bAxis[0].x),fabs(bAxis[0].y),fabs(bAxis[0].z));
	bFabsAxis[1].set(fabs(bAxis[1].x),fabs(bAxis[1].y),fabs(bAxis[1].z));
	bFabsAxis[2].set(fabs(bAxis[2].x),fabs(bAxis[2].y),fabs(bAxis[2].z));
	//case 1
	if( ASeparatingAxis(c, obbA.mHalfLength, bFabsAxis, obbB.mHalfLength) )
		return false;
	//case 2
	t.set(c.dot(bAxis[0]),c.dot(bAxis[1]),c.dot(bAxis[2]));
	if( BSeparatingAxis(t, obbA.mHalfLength, bFabsAxis, obbB.mHalfLength) )
		return false;
	////case 3
	if( ABSeparatingAxis(c,obbA.mHalfLength,bAxis,bFabsAxis,obbB.mHalfLength) )
		return false;
	return true;
}

bool	OBBOverlapTestOBB_OptimizedSeparatingAxisMethod2(const SrOBBox3D& obbA, const SrOBBox3D& obbB)
{
	SrVector3D t;
	t = obbB.mCenter - obbA.mCenter;
	int i , j , k;
	SrReal s , r;
	for( i=0 ; i<3 ; i++ )
	{
		s = fabs(obbA.mAxis[i].dot(t));
		r = obbA.mHalfLength[i];
		for( j=0 ; j<3 ; j++ )
			r += obbB.mHalfLength[j]*fabs(obbB.mAxis[j].dot(obbA.mAxis[i]));
		if( GREATER(s,r) )
			return false;
	}
	for( i=0 ; i<3 ; i++ )
	{
		s = fabs(obbB.mAxis[i].dot(t));
		r = obbB.mHalfLength[i];
		for( j=0 ; j<3 ; j++ )
			r += obbA.mHalfLength[j]*fabs(obbA.mAxis[j].dot(obbB.mAxis[i]));
		if( GREATER(s,r) )
			return false;
	}
	SrVector3D n;
	for( i=0 ; i<3 ; i++ )
		for( j=0 ; j<3 ; j++ )
		{
			n = obbA.mAxis[i].cross(obbB.mAxis[j]);
			s = fabs(n.dot(t));
			r = 0.0;
			for( k=0 ; k<3 ; k++ )
				r += obbA.mHalfLength[k]*fabs(obbA.mAxis[k].dot(n)) + obbB.mHalfLength[k]*fabs(obbB.mAxis[k].dot(n));
			if( GREATER(s,r) )
				return false;
		}

	return true;
}


/*
\brief	判断线段与给定OBB是否重叠，无法计算出交点。
		参考论文：Gregory, A., et al. "H-COLLIDE: A framework for fast and accurate collision detection for haptic interaction."
\return	true	重叠
		false	不重叠
*/
bool SegmentOverlapTestOBB(const SrPoint3D& s0, const SrPoint3D& s1, const SrOBBox3D& obb)
{
	SrPoint3D point1 , point2;
	SrVector3D p1 = s0 - obb.mCenter;
	SrVector3D p2 = s1 - obb.mCenter;

	point1.x = obb.mAxis[0].dot(p1);
	point1.y = obb.mAxis[1].dot(p1);
	point1.z = obb.mAxis[2].dot(p1);

	point2.x = obb.mAxis[0].dot(p2);
	point2.y = obb.mAxis[1].dot(p2);
	point2.z = obb.mAxis[2].dot(p2);

	SrPoint3D m = (point1 + point2)*0.5;
	SrVector3D w = m - point1;

	SrReal X = fabs(w.x), Y = fabs(w.y) , Z = fabs(w.z);
	if( GREATER(fabs(m.x),X+obb.mHalfLength[0]) )
		return false;
	if( GREATER(fabs(m.y),Y+obb.mHalfLength[1]) )
		return false;
	if( GREATER(fabs(m.z),Z+obb.mHalfLength[2]) )
		return false;
	if( GREATER(fabs(m.y*w.z - m.z*w.y),obb.mHalfLength[1]*Z + obb.mHalfLength[2]*Y) )
		return false;
	if( GREATER(fabs(m.x*w.z - m.z*w.x),obb.mHalfLength[0]*Z + obb.mHalfLength[2]*X) )
		return false;
	if( GREATER(fabs(m.x*w.y - m.y*w.x),obb.mHalfLength[0]*Y + obb.mHalfLength[1]*X) )
		return false;
	return true;
}

bool SubOBBOverlapTestOBB_NaiveMethod(const SrOBBox3D& obbA, const SrOBBox3D& obbB)
{
	SrPoint3D s0 , s1;

	//X Direction
	s0 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;
	s0 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;
	s0 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;
	s0 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;
	//Y Direction
	s0 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;
	s0 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;
	s0 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;
	s0 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;

	//Z Direction
	s0 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;
	s0 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] + obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;
	s0 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter + obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;
	s0 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] - obbA.mHalfLength[2]*obbA.mAxis[2];
	s1 = obbA.mCenter - obbA.mHalfLength[0]*obbA.mAxis[0] - obbA.mHalfLength[1]*obbA.mAxis[1] + obbA.mHalfLength[2]*obbA.mAxis[2];
	if( SegmentOverlapTestOBB(s0,s1,obbB) )
		return true;

	return false;
}

bool OBBOverlapTestOBB_NaiveMethod(const SrOBBox3D& obbA, const SrOBBox3D& obbB)
{
	
	if( SubOBBOverlapTestOBB_NaiveMethod(obbA,obbB) )
		return true;
	if( SubOBBOverlapTestOBB_NaiveMethod(obbB,obbA) )
		return true;
	return false;
}

const SrOBBox3D RandomOBB(int range)
{
	SrPoint3D p0 , p1, up;
	SrOBBox3D obb;
	do 
	{
		p0.x = rand() % range;
		p0.y = rand() % range;
		p0.z = rand() % range;

		p1.x = rand() % range;
		p1.y = rand() % range;
		p1.z = rand() % range;

		up.x = rand() % range;
		up.y = rand() % range;
		up.z = rand() % range;

		obb.mAxis[0] = p1 - p0;
		obb.mAxis[1] = up.cross(obb.mAxis[0]);
		obb.mAxis[2] = obb.mAxis[1].cross(obb.mAxis[0]);

		obb.mAxis[0].normalize();
		obb.mAxis[1].normalize();
		obb.mAxis[2].normalize();

		obb.mCenter.x = rand() % range;
		obb.mCenter.y = rand() % range;
		obb.mCenter.z = rand() % range;

		obb.mHalfLength[0] = rand() % range;
		obb.mHalfLength[1] = rand() % range;
		obb.mHalfLength[2] = rand() % range;
	} while (!obb.isValid());

	return obb;
}

void Test_OBBOverlapTestOBB()
{
	int numObb = 10000 , i;
	SrOBBox3D* obbA = new SrOBBox3D[numObb];
	SrOBBox3D* obbB = new SrOBBox3D[numObb];

	int range = 100;
	for( i=0 ; i<numObb ; i++ )
	{
		obbA[i] = RandomOBB(range);
		obbB[i] = RandomOBB(range);
	}

	bool status0 , status1;
	for( i=0 ; i<numObb ; i++ )
	{
		status0 = OBBOverlapTestOBB_NaiveMethod(obbA[i],obbB[i]);
		status1 = OBBOverlapTestOBB_OptimizedSeparatingAxisMethod(obbA[i],obbB[i]);
		ASSERT(status0 == status1);
	}

	double mTime ;
	mTime = clock();
	for( i=0 ; i<numObb ; i++ )
	{
		OBBOverlapTestOBB_NaiveMethod(obbA[i],obbB[i]);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("%f\n",mTime);

	mTime = clock();
	for( i=0 ; i<numObb ; i++ )
	{
		OBBOverlapTestOBB_OptimizedSeparatingAxisMethod(obbA[i],obbB[i]);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("%f\n",mTime);

	delete []obbA;
	delete []obbB;
}



int main( )
{
	Test_OBBOverlapTestOBB();
	return 0;
}