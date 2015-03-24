/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/19
****************************************************************************/
/************************************************************************		
\description	两种方法，判断线段与包围盒是否重叠。
****************************************************************************/
#include "../include/SrGeometricTools.h"
#include "../include/SrDataType.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

/**
\brief 3D segment class.
This is a 3D segment class with public data members,two endpoints.
*/
class SrSegment3D
{
public:
	/**
	\brief Default constructor, the two endpoints is set to (0,0,0).
	*/
	SrSegment3D()
	{
		mPoint1.set(0,0,0);
		mPoint2.set(0,0,0);
	}
	/**
	\brief The segment is initialized by two points.
	*/
	SrSegment3D(const SrPoint3D& p1,const SrPoint3D& p2)
	{
		mPoint1 = p1;
		mPoint2 = p2;
	}
	/**
	\brief  true if the line is valid.
	*/
	bool isValid()const
	{
		if( EQUAL(mPoint1.x,mPoint2.x)&&EQUAL(mPoint1.y,mPoint2.y)&&EQUAL(mPoint1.z,mPoint2.z) )
			return false;
		return true;
	}
public:
	SrPoint3D mPoint1;
	SrPoint3D mPoint2;
};

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
		int i;
		if( UNEQUAL(mAxis[0].dot(mAxis[1]),0) ||
			UNEQUAL(mAxis[0].dot(mAxis[2]),0)||
			UNEQUAL(mAxis[1].dot(mAxis[2]),0))
			return false;
		for( i=0 ; i<3 ; i++ )
		{
			if( LEQUAL(mHalfLength[i],0) )
				return false;
			if( UNEQUAL(mAxis[i].magnitudeSquared(),1.0) )
				return false;
		}
		return true;
	}

public:
	SrPoint3D	mCenter;
	SrVector3D	mAxis[3];
	SrReal		mHalfLength[3];
};

/*
\brief	判断线段与给定OBB是否重叠，无法计算出交点。
		参考论文：Gregory, A., et al. "H-COLLIDE: A framework for fast and accurate collision detection for haptic interaction."
\return	true	重叠
		false	不重叠

在某些平台上，fabs()函数的开销较大，导致快速法反而比厚板法慢。
*/
bool SegmentOverlapTestOBB_FastTesting(const SrSegment3D& segment, const SrOBBox3D& obb)
{
	SrPoint3D point1 , point2;
	SrVector3D p1 = segment.mPoint1 - obb.mCenter;
	SrVector3D p2 = segment.mPoint2 - obb.mCenter;

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


/*
\brief	利用厚板法，判断线段与给定OBB是否重叠。采用这种方法，可以计算出交点。
\return	true	重叠
		false	不重叠
*/
bool SegmentOverlapTestOBB_SlabsMethod(const SrSegment3D& segment, const SrOBBox3D& obb)
{
	SrPoint3D point1 , point2;
	SrVector3D p1 = segment.mPoint1 - obb.mCenter;
	SrVector3D p2 = segment.mPoint2 - obb.mCenter;

	point1.x = obb.mAxis[0].dot(p1);
	point1.y = obb.mAxis[1].dot(p1);
	point1.z = obb.mAxis[2].dot(p1);

	point2.x = obb.mAxis[0].dot(p2);
	point2.y = obb.mAxis[1].dot(p2);
	point2.z = obb.mAxis[2].dot(p2);
	
	//判断线段端点是否在包围盒内
	if( GEQUAL(point1.x,-obb.mHalfLength[0])&&LEQUAL(point1.x, obb.mHalfLength[0])&&
		GEQUAL(point1.y,-obb.mHalfLength[1])&&LEQUAL(point1.y, obb.mHalfLength[1])&&
		GEQUAL(point1.z,-obb.mHalfLength[2])&&LEQUAL(point1.z, obb.mHalfLength[2]))
		return true;
	if( GEQUAL(point2.x,-obb.mHalfLength[0])&&LEQUAL(point2.x, obb.mHalfLength[0])&&
		GEQUAL(point2.y,-obb.mHalfLength[1])&&LEQUAL(point2.y, obb.mHalfLength[1])&&
		GEQUAL(point2.z,-obb.mHalfLength[2])&&LEQUAL(point2.z, obb.mHalfLength[2]))
		return true;

	//判断线段是否与包围盒相交
	SrVector3D direction = point2 - point1, base = point1;
	SrReal tNear = SR_MIN_F32;
	SrReal tFar = SR_MAX_F32;
	SrReal t0,t1,tmp;
	int i;
	for( i=0 ; i<3 ; i++ )
	{
		if( EQUAL(direction[i],0) )
		{
			if( LESS(base[i],-obb.mHalfLength[i]) || GREATER(base[i], obb.mHalfLength[i]) )
				return false;
			continue;
		}
		t0 = (-obb.mHalfLength[i] - base[i]) / direction[i];
		t1 = ( obb.mHalfLength[i] - base[i]) / direction[i];

		if( GREATER(t0,t1) )
		{
			tmp = t0;
			t0 = t1;
			t1 = tmp;
		}
		if( GREATER(t0,tNear) )
			tNear = t0;
		if( LESS(t1,tFar) )
			tFar = t1;
		if( GREATER(tNear,tFar) || LESS(tFar,0) || GREATER(tNear,1.0) )
			return false;
	}

	return true;
}

/*
\brief 随机生成线段数据，测试两种线段与OBB相交测试算法的正确性，并测试两种算法的效率。
*/
void Test_SegmentOverlapTestOBB()
{
	SrVector3D axis[3];
	axis[0] = SrVector3D(1,0,0);
	axis[1] = SrVector3D(0,1,0);
	axis[2] = SrVector3D(0,0,1);
	SrReal halfLen[3] = {250,250,250};
	SrPoint3D center = SrVector3D(750,750,750);
	SrOBBox3D obb(center,axis,halfLen);

	int numSeg = 1000000;
	SrSegment3D* segment = new SrSegment3D[numSeg];
	int i , range = 2000;
	SrPoint3D p0 , p1, p2;
	for( i=0 ; i<numSeg ; i++ )
	{
		do 
		{
			p0.x = rand() % range;
			p0.y = rand() % range;
			p0.z = rand() % range;

			p1.x = rand() % range;
			p1.y = rand() % range;
			p1.z = rand() % range;

			segment[i].mPoint1 = p0;
			segment[i].mPoint2 = p1;
		} while (!segment[i].isValid());
	}
	bool status0 , status1;
	//测试算法的正确性
	for( i=0 ; i<numSeg ; i++ )
	{
		status0 = SegmentOverlapTestOBB_FastTesting(segment[i],obb);
		status1 = SegmentOverlapTestOBB_SlabsMethod(segment[i],obb);
		ASSERT(status0 == status1);
	}

	double mTime ;
	printf("快速测试法，时间开销(s)：");
	mTime = clock();
	for( i=0 ; i<numSeg ; i++ )
	{
		SegmentOverlapTestOBB_FastTesting(segment[i],obb);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("%f\n",mTime);

	printf("厚板法，时间开销(s)：");
	mTime = clock();
	for( i=0 ; i<numSeg ; i++ )
	{
		SegmentOverlapTestOBB_SlabsMethod(segment[i],obb);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("%f\n",mTime);

	delete []segment;
}

int main() 
{
	Test_SegmentOverlapTestOBB();
	return 0;
}