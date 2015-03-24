/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/19
****************************************************************************/
/************************************************************************		
\description	通过寻找包围盒上最接近平面法向量的对角线的方法，
				实现平面与AABB、OBB的相交测试；
				通过对投影距离与包围盒中心到平面的距离的方法，
				实现平面与AABB、OBB的相交测试。
****************************************************************************/
#include "../include/SrGeometricTools.h"
#include "../include/SrDataType.h"
#include <stdio.h>
#include <math.h>
#include <time.h>


/**
\brief 3D axis-aligned bounding box (AABB) class.
*/
class SrAABBox3D
{
public:
	SrAABBox3D()
	{
		mMinAABB.set(0,0,0);
		mMaxAABB.set(0,0,0);
	}
	SrAABBox3D(const SrPoint3D& mn,const SrPoint3D& mx)
	{
		mMinAABB = mn;
		mMaxAABB = mx;
	}
	/**
	\brief The AABB is valid if the three components in "minAABB" are greater to those in "maxAABB".
	*/
	bool	isValid() const
	{
		if( GEQUAL(mMinAABB.x,mMaxAABB.x) ||
			GEQUAL(mMinAABB.y,mMaxAABB.y) ||
			GEQUAL(mMinAABB.z,mMaxAABB.z))
			return false;
		return true;
	}

public:
	SrPoint3D mMinAABB;
	SrPoint3D mMaxAABB;
};


/**
\brief 3D plane class.

This is a 3D plane class with public data members.
The line is parameterized as ^n*^X+d=0,in which ^n is the 'normal' data,d is the 'd' data.
The normal isn't normalized.
*/
class SrPlane3D
{
public:
	/**
	\brief Default constructor.
	*/
	SrPlane3D()
	{
		mNormal.set(0,0,0);
		mD = 0;
	}
	bool init(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D&p2)
	{
		SrVector3D norm = (p1-p0).cross(p2-p0);
		if(EQUAL(norm.x,0)&&EQUAL(norm.y,0)&&EQUAL(norm.z,0))
			return false;
		mNormal = norm;
		mD = -mNormal.dot(p0);
		return true;
	}
	/**
	\brief  The plane is valid if the normal of the plane is not zero.
	*/
	bool	isValid()const
	{
		if( EQUAL(mNormal.x,0)&&EQUAL(mNormal.y,0)&&EQUAL(mNormal.z,0) )
			return false;
		return true;
	}

public:
	SrVector3D	mNormal;
	SrReal		mD;
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
\brief	通过寻找包围盒上最接近平面法向量的对角线，实现平面与AABB的相交测试
\return	SR_PLANE_TANGENT	若平面与AABB相切
\return	SR_PLANE_FRONT		若AABB在平面的正半空间上
\return	SR_PLANE_BACK		若AABB在平面的负半空间上
\return	SR_PLANE_INTERSECTING	若平面与AABB相交
*/
int	Plane3DHitTestAABB_DiagonalMethod(const SrPlane3D& plane,const SrAABBox3D& aabb)
{
	SrPoint3D minPoint,maxPoint;
	int i;
	for( i=0 ; i<3 ; i++ )
	{
		if( GEQUAL(plane.mNormal[i],0) )
		{
			minPoint[i] = aabb.mMinAABB[i];
			maxPoint[i] = aabb.mMaxAABB[i];
		}
		else
		{
			minPoint[i] = aabb.mMaxAABB[i];
			maxPoint[i] = aabb.mMinAABB[i];
		}
	}
	SrReal tmp ;
	tmp = plane.mNormal.dot(minPoint) + plane.mD;
	if( EQUAL(tmp,0) )
		return SR_PLANE_TANGENT;
	else if( GREATER(tmp,0) )
		return SR_PLANE_FRONT;
	tmp = plane.mNormal.dot(maxPoint) + plane.mD ;
	if( EQUAL(tmp,0) )
		return SR_PLANE_TANGENT;
	else if( LESS(tmp , 0) )
		return SR_PLANE_BACK;
	return SR_PLANE_INTERSECTING;
}

/*
\brief	通过对投影距离与包围盒中心到平面的距离，实现平面与AABB的相交测试
\return	SR_PLANE_TANGENT	若平面与AABB相切
\return	SR_PLANE_FRONT		若AABB在平面的正半空间上
\return	SR_PLANE_BACK		若AABB在平面的负半空间上
\return	SR_PLANE_INTERSECTING	若平面与AABB相交
*/
int	Plane3DHitTestAABB_DistanceMethod(const SrPlane3D& plane,const SrAABBox3D& aabb)
{
	SrVector3D v0 = SrVector3D(1.0,0,0);
	SrVector3D v1 = SrVector3D(0,1.0,0);
	SrVector3D v2 = SrVector3D(0,0,1.0);
	SrReal r = 0;
	r += fabs(plane.mNormal.dot(v0)*(aabb.mMaxAABB[0] - aabb.mMinAABB[0]));
	r += fabs(plane.mNormal.dot(v1)*(aabb.mMaxAABB[1] - aabb.mMinAABB[1]));
	r += fabs(plane.mNormal.dot(v2)*(aabb.mMaxAABB[2] - aabb.mMinAABB[2]));
	SrPoint3D center = aabb.mMinAABB + aabb.mMaxAABB;
	SrReal tmp = (plane.mNormal.dot(center) + 2*plane.mD) ;
	if( EQUAL(tmp , r) || EQUAL(tmp , -r) )
		return SR_PLANE_TANGENT;
	else if( GREATER(tmp,r) )
		return SR_PLANE_FRONT;
	else if( LESS(tmp,-r) )
		return SR_PLANE_BACK;
	return SR_PLANE_INTERSECTING;
}


/*
\brief	通过寻找包围盒上最接近平面法向量的对角线，实现平面与OBB的相交测试
\return	SR_PLANE_TANGENT	若平面与OBB相切
\return	SR_PLANE_FRONT		若OBB在平面的正半空间上
\return	SR_PLANE_BACK		若OBB在平面的负半空间上
\return	SR_PLANE_INTERSECTING	若平面与OBB相交
*/
int	Plane3DHitTestOBB_DiagonalMethod(const SrPlane3D& plane,const SrOBBox3D& obb)
{
	SrPoint3D minPoint,maxPoint;
	SrVector3D normal;
	normal.x = obb.mAxis[0].dot(plane.mNormal);
	normal.y = obb.mAxis[1].dot(plane.mNormal);
	normal.z = obb.mAxis[2].dot(plane.mNormal);
	SrReal d = obb.mCenter.dot(plane.mNormal);
	int i;
	for( i=0 ; i<3 ; i++ )
	{
		if( GEQUAL(normal[i],0) )
		{
			minPoint[i] = - obb.mHalfLength[i];
			maxPoint[i] =   obb.mHalfLength[i];
		}
		else
		{
			minPoint[i] =   obb.mHalfLength[i];
			maxPoint[i] = - obb.mHalfLength[i];
		}
	}
	SrReal tmp ;
	tmp = normal.dot(minPoint) + d + plane.mD;
	if( EQUAL(tmp,0) )
		return SR_PLANE_TANGENT;
	else if( GREATER(tmp,0) )
		return SR_PLANE_FRONT;
	tmp = normal.dot(maxPoint) + d + plane.mD;
	if( EQUAL(tmp,0) )
		return SR_PLANE_TANGENT;
	else if( LESS(tmp , 0) )
		return SR_PLANE_BACK;
	return SR_PLANE_INTERSECTING;
}

/*
\brief	通过对投影距离与包围盒中心到平面的距离，实现平面与OBB的相交测试
\return	SR_PLANE_TANGENT	若平面与OBB相切
\return	SR_PLANE_FRONT		若OBB在平面的正半空间上
\return	SR_PLANE_BACK		若OBB在平面的负半空间上
\return	SR_PLANE_INTERSECTING	若平面与OBB相交
*/
int	Plane3DHitTestOBB_DistanceMethod(const SrPlane3D& plane,const SrOBBox3D& obb)
{
	SrReal r = 0;
	r += fabs(plane.mNormal.dot(obb.mAxis[0])*obb.mHalfLength[0]);
	r += fabs(plane.mNormal.dot(obb.mAxis[1])*obb.mHalfLength[1]);
	r += fabs(plane.mNormal.dot(obb.mAxis[2])*obb.mHalfLength[2]);
	SrReal tmp = (plane.mNormal.dot(obb.mCenter) + plane.mD) ;
	if( EQUAL(tmp , r) || EQUAL(tmp , -r) )
		return SR_PLANE_TANGENT;
	else if( GREATER(tmp,r) )
		return SR_PLANE_FRONT;
	else if( LESS(tmp,-r) )
		return SR_PLANE_BACK;
	return SR_PLANE_INTERSECTING;
}

/*
\brief 随机生成平面数据，测试两种平面与AABB、OBB相交测试算法的正确性，并测试两种算法的效率。
*/
void Test_Plane3DHitTestBoundingBox()
{
	int numPlane = 10000;
	SrPlane3D* plane = new SrPlane3D[numPlane];
	int i , range = 2000;
	SrPoint3D p0 , p1, p2;
	for( i=0 ; i<numPlane ; i++ )
	{
		do 
		{
			p0.x = rand() % range;
			p0.y = rand() % range;
			p0.z = rand() % range;

			p1.x = rand() % range;
			p1.y = rand() % range;
			p1.z = rand() % range;

			p2.x = rand() % range;
			p2.y = rand() % range;
			p2.z = rand() % range;
		} while (!plane[i].init(p0,p1,p2) || !plane[i].isValid());
	}
	SrAABBox3D aabb(SrPoint3D(500,500,500),SrPoint3D(1000,1000,1000));
	int status0 , status1;
	//测试算法的正确性
	for( i=0 ; i<numPlane ; i++ )
	{
		status0 = Plane3DHitTestAABB_DiagonalMethod(plane[i],aabb);
		status1 = Plane3DHitTestAABB_DistanceMethod(plane[i],aabb);
		ASSERT(status0 == status1);
	}
	double mTime ;
	printf("平面与AABB相交检测:最接近对角线法，时间开销(s)：");
	mTime = clock();
	for( i=0 ; i<numPlane ; i++ )
	{
		Plane3DHitTestAABB_DiagonalMethod(plane[i],aabb);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("%f\n",mTime);

	printf("平面与AABB相交检测:距离判定法，时间开销(s)：");
	mTime = clock();
	for( i=0 ; i<numPlane ; i++ )
	{
		Plane3DHitTestAABB_DistanceMethod(plane[i],aabb);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("%f\n",mTime);

	SrVector3D axis[3];
	axis[0] = SrVector3D(1,0,0);
	axis[1] = SrVector3D(0,1,0);
	axis[2] = SrVector3D(0,0,1);
	SrReal halfLen[3] = {250,250,250};
	SrPoint3D center = SrVector3D(750,750,750);
	SrOBBox3D obb(center,axis,halfLen);
	//测试算法的正确性
	for( i=0 ; i<numPlane ; i++ )
	{
		status0 = Plane3DHitTestOBB_DiagonalMethod(plane[i],obb);
		status1 = Plane3DHitTestOBB_DistanceMethod(plane[i],obb);
		ASSERT(status0 == status1);
	}

	printf("平面与OBB相交检测:最接近对角线法，时间开销(s)：");
	mTime = clock();
	for( i=0 ; i<numPlane ; i++ )
	{
		Plane3DHitTestOBB_DiagonalMethod(plane[i],obb);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("%f\n",mTime);

	printf("平面与OBB相交检测:距离判定法，时间开销(s)：");
	mTime = clock();
	for( i=0 ; i<numPlane ; i++ )
	{
		Plane3DHitTestOBB_DistanceMethod(plane[i],obb);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("%f\n",mTime);

	delete []plane;
}

int main( )
{
	Test_Plane3DHitTestBoundingBox();
	return 0;
}