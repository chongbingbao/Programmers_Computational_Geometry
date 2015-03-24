///************************************************************************		
//\link	www.twinklingstar.cn
//\author Twinkling Star
//\date	2014/10/28
//****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"
#include <time.h>
#include <stdio.h>

#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D

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

/*
\brief	求凸多边形上顶点indx的下一个顶点
*/
int Next(int indx, int n)
{
	if( indx==n-1 )
		return 0;
	else
		return indx + 1;
}
/*
\brief	比较两个浮点数的大小，返回较大值
*/
Real Max(Real a , Real b)
{
	return (a>b)?a:b;
}

/*
\brief	旋转测径法，计算两个凸多边形的最大距离
*/
Real MaximumDist_RotatingCalipers(const Point2D* pts1,int n1, const Point2D* pts2, int n2)
{
	int i;
	//求多边形pts1上，Y轴方向上值最小的点
	int mnInx = 0;
	for( i = 1 ; i < n1 ; ++ i )
	{
		if( pts1[i].y < pts1[mnInx].y )
			mnInx = i;
	}
	//求多边形pts2上，Y轴方向上值最大的点
	int mxInx = 0;
	for( i = 0 ; i < n2 ; ++i )
	{
		if( pts2[i].y > pts2[mxInx].y )
			mxInx = i;
	}
	int crInx1 = mnInx , crInx2 = mxInx;
	Real maxDist = 0 , dist;
	int cnt1 = 0, cnt2 = 0;
	do
	{
		int nxInx1 = Next(crInx1 , n1);
		int nxInx2 = Next(crInx2 , n2);
		Vector2D d1 = pts1[nxInx1] - pts1[crInx1];
		Vector2D d2 = pts2[nxInx2] - pts2[crInx2];
		Real angle = d1.cross(d2);

		if( angle < 0 )
		{//旋转凸多边形pts1的轴
			dist = (pts1[nxInx1] - pts2[crInx2]).magnitude();
			maxDist = Max(dist , maxDist);

			crInx1 = Next(crInx1 , n1);
			cnt1 += 1;
		}
		else
		{//旋转凸多边形pts2的轴
			dist = (pts2[nxInx2] - pts1[crInx1]).magnitude();
			maxDist = Max(dist , maxDist);

			crInx2 = Next(crInx2 , n2);
			cnt2 += 1;
		}
	}while( cnt1 <= n1 || cnt2 <= n2 );

	return maxDist;
}

/*
\brief	暴力法，计算两个凸多边形的最大距离
*/
Real MaximumDist_Naive(const Point2D* pts1,int n1, const Point2D* pts2, int n2)
{
	Real dist , maxDist = 0;
	int i , j;
	for( i = 0 ; i < n1 ; ++i )
	{
		for( j = 0 ; j < n2 ; ++j )
		{
			//点pts1[i]到pts2[j]之间的距离
			dist = (pts1[i] - pts2[j]).magnitude();
			maxDist = Max(dist,maxDist);
		}
	}
	return maxDist;
}

void Test()
{
	int n1 = 200;
	Point2D* pts1 = new Point2D[n1];
	GenerateConvex(n1,1000,Point2D(0,0),pts1);

	int n2 = 300;
	Point2D* pts2 = new Point2D[n2];
	GenerateConvex(n2,1000,Point2D(5000,5000),pts2);

	Real dist1 = MaximumDist_RotatingCalipers(pts1,n1,pts2,n2);
	Real dist2 = MaximumDist_Naive(pts1,n1,pts2,n2);

	//对比旋转测径法和暴力法两种算法计算出的结果，测试算法的正确性
	ASSERT(EQUAL(dist1,dist2));
	printf("Maximum Distance-Rotating Calipers:\t%f\n",dist1);
	printf("Maximum Distance-Naive	          :\t%f\n",dist2);

	delete []pts1;
	delete []pts2;
}

int main()
{
	int cntCas = 100;
	int total = cntCas;
	while( cntCas-- )
	{
		printf("Case %d:\n",total-cntCas);
		Test();
	}
	return 0;
}