/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/18
****************************************************************************/
/************************************************************************		
\description	在二维空间上，给定3个不共线的点，计算它们的内切圆；
				在三维空间上，给定4个不共面的点，计算它们的内切球体；
****************************************************************************/

#include "../include/SrGeometricTools.h"
#include "../include/SrDataType.h"
#include <stdio.h>
#include <math.h>

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
\brief	在二维空间下，计算点到直线的距离。
\param[in]	p0,p1	确定一条直线，方向由点p0指向点p1。
*/
SrReal PointToLineDistance(const SrPoint2D& p,const SrPoint2D& p0, const SrPoint2D& p1)
{
	SrVector2D  mBase= p1;
	SrVector2D	mDirection = p1 - p0;
	mDirection.normalize();
	SrVector2D tmp = p - mBase;
	SrReal tmpDot = mDirection.dot(tmp);
	return sqrt(tmp.dot(tmp)-tmpDot*tmpDot);
}

SrReal PointToPlaneDistance(const SrPoint3D& p,const SrPoint3D& p0, const SrPoint3D& p1,const SrPoint3D& p2)
{
	SrVector3D norm = (p1-p0).cross(p2-p0);
	if( EQUAL(norm.x,0)&&EQUAL(norm.y,0)&&EQUAL(norm.z,0) )
		return -1.0;
	SrVector3D mNormal = norm;
	SrReal mD = -mNormal.dot(p0);
	return fabs((p.dot(mNormal)+mD)/mNormal.magnitude());
}

/*
\brief	计算给定三角形的内切圆。
\return true	若三角形的三个顶点不共线，存在内切圆。
		false	若三角形的三个顶点共线，不存在内切圆。
*/
bool InitInscribedCircle(const SrPoint2D& p0,const SrPoint2D& p1,const SrPoint2D& p2,SrCircle2D& circle)
{
	SrReal A = fabs((p1-p0).cross(p2-p0));
	//Check the three points are not on one line.
	if( EQUAL(A,0) )
		return false;
	SrReal l0 = (p0-p1).magnitude(), l1 = (p1-p2).magnitude() , l2 = (p2-p0).magnitude();
	SrReal l = l0 + l1 + l2;
	circle.mRadius = A / l;
	circle.mCenter = (l1*p0 + l2*p1 + l0*p2) / l;
	return true;
}

/*
\brief 测试给定圆是否是三角形的内切圆。
\return true	若圆心到三角形三条边的距离都等于半径。
		false	反之。
*/
bool IsInscribedCircle(const SrPoint2D& p0,const SrPoint2D& p1,const SrPoint2D& p2, const SrCircle2D& circle)
{
	if( EQUAL(PointToLineDistance(circle.mCenter,p0,p1),circle.mRadius) &&
		EQUAL(PointToLineDistance(circle.mCenter,p1,p2),circle.mRadius) && 
		EQUAL(PointToLineDistance(circle.mCenter,p2,p0),circle.mRadius))
		return true;
	return false;
}


/*
\brief 随机生成数据，测试三角形的内切圆算法的正确性。
若点的数值范围很大，由于精确问题，会导致错误的发生。
*/
void Test_InscribedCircle()
{
	int num = 100000 , i;
	SrPoint2D* p0 = new SrPoint2D[num];
	SrPoint2D* p1 = new SrPoint2D[num];
	SrPoint2D* p2 = new SrPoint2D[num];
	int range = 100;
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
		if( InitInscribedCircle(p0[i],p1[i],p2[i],circle) )
		{
			ASSERT(IsInscribedCircle(p0[i],p1[i],p2[i],circle));
		}
	}
}

/*
\brief	计算给定四面体的内切球。
\return true	若四面体的四个顶点不共面，存在内切球。
		false	若四面体的四个顶点共面，不存在内切球。
*/
bool InitInscribedSphere(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D& p2,const SrPoint3D& p3,SrSphere3D& sphere)
{
	SrVector3D v1 = p1 - p0 , v2 = p2 - p0 , v3 = p3 - p0;
	SrReal V = fabs(v1.dot(v2.cross(v3)));
	if( EQUAL(V,0) )
		return false;
	SrReal a0 , a1 , a2 , a3;
	//Compute the normal of the triangle composed of p1,p2,p3
	SrVector3D n0 = (p2 - p1).cross(p3 - p1);
	n0.normalize();
	//Compute the area of the triangle composed of p1,p2,p3
	a0 = n0.dot(p1.cross(p2) + p2.cross(p3) + p3.cross(p1));

	SrVector3 n1 = (p2 - p0).cross(p3 - p0);
	n1.normalize();
	a1 = n1.dot(p0.cross(p2) + p2.cross(p3) + p3.cross(p0));

	SrVector3 n2 = (p1 - p0).cross(p3 - p0);
	n2.normalize();
	a2 = n2.dot(p0.cross(p1) + p1.cross(p3) + p3.cross(p0));

	SrVector3 n3 = (p1 - p0).cross(p2 - p0);
	n3.normalize();
	a3 = n3.dot(p0.cross(p1) + p1.cross(p2) + p2.cross(p0));

	SrReal A = a0 + a1 + a2 + a3;
	sphere.mRadius = V/A;
	sphere.mCenter = (p0*a0 + p1*a1 + p2*a2 + p3*a3)/A;
	return true;
}

/*
\brief 测试给定球是否是四面体的内切球。
\return true	若球心到四面体四个面的距离都等于半径。
		false	反之。
*/
bool IsInscribedSphere(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D& p2, const SrPoint3D& p3, const SrSphere3D& sphere)
{
	//如果发生错误，有可能是浮点数精度误差造成的
	if( EQUAL(PointToPlaneDistance(sphere.mCenter,p0,p1,p2),sphere.mRadius) &&
		EQUAL(PointToPlaneDistance(sphere.mCenter,p0,p2,p3),sphere.mRadius) && 
		EQUAL(PointToPlaneDistance(sphere.mCenter,p0,p1,p3),sphere.mRadius) &&
		EQUAL(PointToPlaneDistance(sphere.mCenter,p1,p2,p3),sphere.mRadius))
		return true;
	return false;
}

/*
\brief 随机生成数据，测试四面体的内切球算法的正确性。
若点的数值范围很大，由于精确问题，会导致错误的发生。
*/
void Test_InscribedSphere()
{
	int num = 100000 , i;
	SrPoint3D* p0 = new SrPoint3D[num];
	SrPoint3D* p1 = new SrPoint3D[num];
	SrPoint3D* p2 = new SrPoint3D[num];
	SrPoint3D* p3 = new SrPoint3D[num];
	int range = 10000;
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
		if( InitInscribedSphere(p0[i],p1[i],p2[i],p3[i],sphere) )
		{
			ASSERT(IsInscribedSphere(p0[i],p1[i],p2[i],p3[i],sphere));
		}
	}
}

int main( )
{
	Test_InscribedCircle();
	Test_InscribedSphere();
	return 0;
}