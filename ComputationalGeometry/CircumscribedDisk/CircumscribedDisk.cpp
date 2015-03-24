/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/18
****************************************************************************/
/************************************************************************		
\description	在二维空间上，给定3个不共线的点，计算它们的外接圆；
				在三维空间上，给定4个不共面的点，计算它们的外接球体；
****************************************************************************/
#include "../include/SrGeometricTools.h"
#include "../include/SrDataType.h"
#include <stdio.h>

/**
\brief 3D sphere class.

This is a 3D sphere class with public data members.
It includes center and radius.
*/
class SrSphere3D
{
public:
	/**
	\brief Default constructor. Set center to (0,0) and radius to 0.
	*/
	SrSphere3D()
	{
		mCenter.set(0,0,0);
		mRadius = -1.0;
	}
	/**
	\brief The circle is initialized by center and radius.
	*/
	SrSphere3D(const SrPoint3D& ct ,SrReal rd)
	{
		mCenter = ct;
		mRadius = rd;
	}
public:
	SrPoint3D	mCenter;
	SrReal		mRadius;
};

/**
\brief 2D circle class.

This is a 2D circle class with public data members.
It includes center and radius.
*/
class SrCircle2D
{
public:
	/**
	\brief Default constructor. Set center to (0,0) and radius to 0.
	*/
	SrCircle2D()
	{
		mCenter.set(0,0);
		mRadius = 0;
	}
	/**
	\brief The circle is initialized by center and radius.
	*/
	SrCircle2D(const SrPoint2D& ct,SrReal rd)
	{
		mCenter = ct;
		mRadius = rd;
	}
public:
	SrPoint2D	mCenter;
	SrReal		mRadius;
};

/*
\brief	计算给定三角形的外接圆。
\return true	若三角形的三个顶点不共线，存在外接圆。
		false	若三角形的三个顶点共线，不存在外接圆。
*/
bool InitCircumscribedCircle(const SrPoint2D& p0,const SrPoint2D& p1,const SrPoint2D& p2,SrCircle2D& circle)
{
	SrVector2D e1 = p1-p0, e2 = p2-p0;
	SrReal triArea  = e1.cross(e2);
	//Check the three points are not on one line.
	if( EQUAL(triArea,0) )
		return false;
	SrReal len1 = e1.magnitudeSquared(), len2 = e2.magnitudeSquared();
	circle.mCenter.x = p0.x + (len1*e2.y - e1.y*len2)/(2*triArea);
	circle.mCenter.y = p0.y + (len2*e1.x - e2.x*len1)/(2*triArea);
	circle.mRadius = (circle.mCenter-p0).magnitude();
	return true;
}

/*
\brief 测试给定圆是否是三角形的外接圆。
\return true	若圆心到三角形三个顶点的距离都等于半径。
		false	反之。
*/
bool IsCircumscribedCircle(const SrPoint2D& p0,const SrPoint2D& p1,const SrPoint2D& p2, const SrCircle2D& circle)
{
	//如果发生错误，有可能是浮点数精度误差造成的
	if( EQUAL((circle.mCenter - p0).magnitude(),circle.mRadius) &&
		EQUAL((circle.mCenter - p1).magnitude(),circle.mRadius) && 
		EQUAL((circle.mCenter - p2).magnitude(),circle.mRadius))
		return true;
	else
	{
		printf("Radius:%f, %f,%f,%f\n",circle.mRadius,(circle.mCenter - p0).magnitude(),
			(circle.mCenter - p1).magnitude(),(circle.mCenter - p2).magnitude());
	}
	return false;
}

/*
\brief 随机生成数据，测试三角形的外接圆算法的正确性。
若点的数值范围很大，由于精确问题，会导致错误的发生。
*/
void Test_CircumscribedCircle()
{
	int num = 10000 , i;
	SrPoint2D* p0 = new SrPoint2D[num];
	SrPoint2D* p1 = new SrPoint2D[num];
	SrPoint2D* p2 = new SrPoint2D[num];
	int range = 10000;
	for( i=0 ; i<num ;  i++ )
	{
		p0[i].x = rand() % range;
		p0[i].y = rand() % range;
		p1[i].x = rand() % range;
		p1[i].y = rand() % range;
		p2[i].x = rand() % range;
		p2[i].y = rand() % range;
	}
	SrCircle2D circle;
	for( i=0 ; i<num ; i++ )
	{
		if( InitCircumscribedCircle(p0[i],p1[i],p2[i],circle) )
		{
			ASSERT(IsCircumscribedCircle(p0[i],p1[i],p2[i],circle));
		}
	}
}

/*
\brief	计算给定四面体的外接球
\return true	若四面体的四个顶点不共面，存在外接球.
		false	若四面体的四个顶点共面，不存在外接球.
*/
bool InitCircumscribedSphere(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D& p2,const SrPoint3D& p3,SrSphere3D& sphere)
{
	SrVector3 v1 = p1 - p0 , v2 = p2 - p0 , v3 = p3 - p0;
	SrReal V = v1.dot(v2.cross(v3));
	//Check that the three points are not on the same plane.
	if( EQUAL(V,0) )
		return false;
	V*=2.0;
	SrReal L1 = v1.magnitudeSquared() , L2 = v2.magnitudeSquared() , L3 = v3.magnitudeSquared();
	sphere.mCenter.x = (p0.x + ( (v2.y*v3.z - v3.y*v2.z)*L1 - (v1.y*v3.z - v3.y*v1.z)*L2 + (v1.y*v2.z - v2.y*v1.z)*L3 ) / V);
	sphere.mCenter.y = (p0.y + (-(v2.x*v3.z - v3.x*v2.z)*L1 + (v1.x*v3.z - v3.x*v1.z)*L2 - (v1.x*v2.z - v2.x*v1.z)*L3 ) / V);
	sphere.mCenter.z = (p0.z + ( (v2.x*v3.y - v3.x*v2.y)*L1 - (v1.x*v3.y - v3.x*v1.y)*L2 + (v1.x*v2.y - v2.x*v1.y)*L3 ) / V);
	sphere.mRadius = (sphere.mCenter - p0).magnitude();

	return true;
}

/*
\brief 测试给定球是否是四面体的外接球。
\return true	若球心到四面体的四个顶点的距离都等于半径。
		false	反之。
*/
bool IsCircumscribedSphere(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D& p2,const SrPoint3D& p3,SrSphere3D& sphere)
{
	if( EQUAL((sphere.mCenter - p0).magnitude(),sphere.mRadius) &&
		EQUAL((sphere.mCenter - p1).magnitude(),sphere.mRadius) && 
		EQUAL((sphere.mCenter - p2).magnitude(),sphere.mRadius) &&
		EQUAL((sphere.mCenter - p3).magnitude(),sphere.mRadius))
		return true;
	return false;
}

/*
\brief 随机生成数据，测试四面体的外接球算法的正确性。
若点的数值范围很大，由于精确问题，会导致错误的发生。
*/
void Test_CircumscribedSphere()
{
	int num = 100 , i;
	SrPoint3D* p0 = new SrPoint3D[num];
	SrPoint3D* p1 = new SrPoint3D[num];
	SrPoint3D* p2 = new SrPoint3D[num];
	SrPoint3D* p3 = new SrPoint3D[num];
	int range = 100;
	for( i=0 ; i<num ;  i++ )
	{
		p0[i].x = rand() % range;
		p0[i].y = rand() % range;
		p0[i].z = rand() % range;

		p1[i].x = rand() % range;
		p1[i].y = rand() % range;
		p1[i].z = rand() % range;

		p2[i].x = rand() % range;
		p2[i].y = rand() % range;
		p2[i].z = rand() % range;

		p3[i].x = rand() % range;
		p3[i].y = rand() % range;
		p3[i].z = rand() % range;
	}
	SrSphere3D sphere;
	for( i=0 ; i<num ; i++ )
	{
		if( InitCircumscribedSphere(p0[i],p1[i],p2[i],p3[i],sphere) )
		{
			ASSERT(IsCircumscribedSphere(p0[i],p1[i],p2[i],p3[i],sphere));
		}
	}
}


int main()
{
	Test_CircumscribedCircle();
	Test_CircumscribedSphere();
	return 0;
}