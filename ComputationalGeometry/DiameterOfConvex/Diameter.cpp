/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/05
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"
#include <list>
#include <time.h>
#include <set>

#define Real		SrReal
#define Point2D		SrPoint2D

/*
\brief	设三角形的三个点为p0,p1,p2，它的有符号面积为A，返回2A
*/
Real Area(const Point2D& p0 , const Point2D& p1, const Point2D& p2)
{
	return (p1 - p0).cross(p2 - p0);
}

/*
\brief	求凸多边形上顶点i的下一个顶点
*/
int Next(int i, int n)
{
	return (i + 1) % n;
}
/*
\brief	旋转测径法，计算凸多边形的直径
\param	vertex		指向顶点数组的指针，顶点按照逆时针排序；
\param	numVertex	表示顶点数组的个数；
*/
Real  Diameter_RotatingCalipers(const Point2D* vertex , int numVertex)
{
	int p = numVertex - 1;
	int q = 0;
	Point2D ep0 = vertex[p];
	Point2D ep1 = vertex[Next(p,numVertex)];
	while( GREATER(Area(ep0,ep1,vertex[Next(q,numVertex)]) , Area(ep0,ep1,vertex[q]) ) )
		q = Next(q,numVertex);
	int q0 = q;
	Real diameter = -1.0 , dist ;
	p = Next(p,numVertex);
	while( q!=0 && p<=q0 )
	{
		ep0		 = vertex[p];
		ep1		 = vertex[Next(p,numVertex)];
		dist	 = (vertex[p] - vertex[q]).magnitude();
		diameter = GREATER(dist,diameter)?dist:diameter;

		while( GREATER(Area(ep0,ep1,vertex[Next(q,numVertex)]) , Area(ep0,ep1,vertex[q])) )
		{
			q = Next(q,numVertex);
			if( p!=q0 || q!=0 )
			{
				dist = (vertex[p] - vertex[q]).magnitude();
				diameter = GREATER(dist,diameter)?dist:diameter;

			}
			else
			{
				return diameter;
			}
		}

		if( EQUAL(Area(ep0,ep1,vertex[Next(q,numVertex)]) , Area(ep0,ep1,vertex[q])) )
		{
			if( p!=q0 || q!=numVertex-1 )
			{
				dist = (vertex[p] - vertex[Next(q,numVertex)]).magnitude();
				diameter = GREATER(dist,diameter)?dist:diameter;
			}
			else
			{
				dist = (vertex[Next(p,numVertex)] - vertex[q]).magnitude();
				diameter = GREATER(dist,diameter)?dist:diameter;
				return diameter;
			}
		}
		p = Next(p,numVertex);

	}
	return diameter;

}

/*
\brief	暴力法，计算凸多边形的直径。
*/
Real Diameter_Naive(const Point2D* vertex, int numVertex)
{
	Real diameter = -1.0 , dist;
	int i ,j;
	for( i=0 ; i<numVertex ; i++ )
		for( j=1 ; j<numVertex ; j++ )
		{
			dist = (vertex[i] - vertex[j]).magnitude();
			diameter = GREATER(dist,diameter)?dist:diameter;
		}
		return diameter;
}

/*
\brief	随机生成用于测试的凸多边形，生成的凸多边形是逆时针顺序的，在一个圆盘上
\param[in]	numPoint	规定生成的凸多边形顶点个数
\param[in]	radius		规定圆盘的半径
\param[in]	center		规定圆盘的中心
\param[out]	pts			多边形顶点的指针
*/
void GenerateConvex(int numPoint, Real radius, const Point2D& center,Point2D* pts )
{
	ASSERT(numPoint>=3);

	Real*	angle = new Real[numPoint];
	int range = 1000;
	int	base  = 100;
	Real sum = 0 , partSum = 0, curAngle;
	int i;
	for( i=0 ; i<numPoint ; i++ )
	{
		angle[i] = ((rand() % range) + base) / (Real)(range);
		sum += angle[i];
	}
	for( i=0 ; i<numPoint ; i++ )
	{
		partSum += angle[i];
		curAngle = 2*SrPiF32*partSum / sum;
		pts[i].x = -sin(curAngle)*radius + center.x;
		pts[i].y = cos(curAngle)*radius + center.y;
	}
	delete []angle;
}


void Test(int numPoint)
{
	printf("Number of Convex:%d\n",numPoint);
	Point2D* points = new Point2D[numPoint];
	GenerateConvex(numPoint,100000.0,Point2D(0,0),points);

	Real diam0 , diam1 ;
	double seconds = clock();
	diam0 = Diameter_RotatingCalipers(points,numPoint);
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("Clock:%.6lf\n",seconds);
	printf("Rotating Calipers--Diameter:%f\n",diam0);

	seconds = clock();
	diam1 = Diameter_Naive(points,numPoint);
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("Clock:%.6lf\n",seconds);
	printf("Naive Method--Diameter:%f\n",diam1);

	ASSERT(EQUAL(diam0,diam1));
	delete []points;

}

int main( )
{
	int casInx = 0 , i;
	for( i = 4 ; i<10000 ; i+= 1 )
	{
		printf("Case %d:\n", ++casInx);
		Test(i);
	}
	return 0;
}