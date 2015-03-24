/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/18
****************************************************************************/
/************************************************************************		
\description	采用参数方程法和优化法，实现射线与球体的相交测试。
****************************************************************************/
#include "../include/SrGeometricTools.h"
#include "../include/SrDataType.h"
#include <stdio.h>
#include <time.h>

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
\brief 3D ray class.

This is a 3D ray class with public data members.
The ray is parameterized as X(t) = P+t*d,in which P is the 'base' data,d is the 'direction' data,t>=0.
the 'direction' is unit length.
*/
class SrRay3D
{
public:
	/**
	\brief Default constructor, base and direction is set to (0,0).
	*/
	SrRay3D()
	{
		mBase.set(0,0,0);
		mDirection.set(0,0,0);
	}
	/**
	\brief The line is initialized by two points.The point p1 is the endpoint of the ray.
	*/
	SrRay3D(const SrPoint3D& p1,const SrPoint3D& p2)
	{
		mBase = p1;
		mDirection = p2 - p1;
		mDirection.normalize();
	}
	/**
	\brief  The ray is valid if the direction of the line is unit.
	*/
	bool isValid()const
	{
		if( EQUAL(mDirection.magnitudeSquared(),1.0) )
			return true;
		return false;
	}

public:
	SrPoint3D	mBase;
	SrVector3D	mDirection;
};


/*
\brief	采用参数方程法，实现射线与球体的相交测试。
\param[out]	result	射线可以用参数方程P+tD表示，返回第一个相切点的t值。
\return	true	若射线与球体相交或者相切；
		false	若射线与球体分离（无交点）。

*/
bool RayHitTestSphere_Parameter(const SrRay3D& ray,const SrSphere3D& sphere,SrReal& result)
{
	SrVector3D e = ray.mBase - sphere.mCenter;
	SrReal b = e.dot(ray.mDirection), c = e.dot(e)-sphere.mRadius*sphere.mRadius;
	SrReal delta = b*b - c;

	if( LESS(delta,0) )
		return false;
	else if( EQUAL(delta,0) )
	{
		if( LESS(-b,0) )
			return false;
		result = -b;
		return true;
	}
	else
	{
		SrReal d = sqrt(delta);
		SrReal t1 = -b - d, t2 = -b + d;
		if( LESS(t2,0) )
			return false;
		else if( LESS(t1,0) )
			result = t2;
		else
			result = t1;
		return true;

	}
	return true;
}

/*
\brief	采用优化法，实现射线与球体的相交测试。
\param[out]	result	射线可以用参数方程P+tD表示，返回第一个相切点的t值。
\return	true	若射线与球体相交或者相切；
		false	若射线与球体分离（无交点）。

*/
bool RayHitTestSphere_Optimized(const SrRay3D& ray,const SrSphere3D& sphere,SrReal& result)
{
	SrVector3D l = sphere.mCenter - ray.mBase;
	SrReal s = l.dot(ray.mDirection);
	SrReal squaredL = l.dot(l);
	SrReal squaredRadius = sphere.mRadius * sphere.mRadius;
	if( LESS(s,0) && GREATER(squaredL,squaredRadius) )
		return false;
	SrReal squaredM = squaredL - s*s;
	if( GREATER(squaredM,squaredRadius) )
		return false;
	SrReal q = sqrt(squaredRadius - squaredM);
	if( squaredL > squaredRadius )
		result = s - q;
	else
		result = s + q;
	return true;
}

/*
\brief 随机生成射线数据，测试两种相交测试方法的正确性，并测试两种方法的CPU时钟开销。
*/
void Test_RayHitTestSphere()
{
	int numRay = 100000;
	SrRay3D* ray = new SrRay3D[numRay];
	//确定测试球体
	SrSphere3D sphere(SrPoint3D(1000,1000,1000),1000);
	int i;
	SrPoint3D p0 , p1;
	//随机生成测试的射线
	for( i=0 ; i<numRay ; i++ )
	{
		do 
		{
			p0.x = rand() % 2000;
			p0.y = rand() % 2000;
			p0.z = rand() % 2000;
			p1.x = rand() % 2000;
			p1.y = rand() % 2000;
			p1.z = rand() % 2000;
			ray[i] = SrRay3D(p0,p1);
		} while (!ray[i].isValid());
	}

	SrReal t0,t1;
	bool ret0, ret1;
	//测试算法的正确性
	for( i=0 ; i<numRay ; i++ )
	{
		ret0 = RayHitTestSphere_Parameter(ray[i],sphere,t0);
		ret1 = RayHitTestSphere_Optimized(ray[i],sphere,t1);
		ASSERT(ret0==ret1);
		//printf("%f,%f\n",t0, t1);
		//ASSERT(EQUAL(t0,t1));
	}
	//参数方程法，消耗时间的统计
	double mTime = clock();
	for( i=0 ; i<numRay ; i++ )
	{
		ret0 = RayHitTestSphere_Parameter(ray[i],sphere,t0);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("时间：%f\n",mTime);
	//优化法，消耗时间的统计
	mTime = clock();
	for( i=0 ; i<numRay ; i++ )
	{
		ret0 = RayHitTestSphere_Optimized(ray[i],sphere,t0);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("时间：%f\n",mTime);

}


int main()
{
	Test_RayHitTestSphere();
	return 0;
}