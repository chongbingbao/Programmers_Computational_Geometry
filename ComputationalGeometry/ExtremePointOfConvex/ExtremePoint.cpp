/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/11
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"
#include <time.h>
#include <stdio.h>

#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D

#define EPS			SR_EPS
#define MIN_VALUE	SR_MIN_F32
#define MAX_VALUE	SR_MAX_F32

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
\brief 判断e与u是否同方向。
\return	返回1，	若e与u同方向；
		返回0，	若e与u垂直；
		返回-1，若e与u方向相反；
*/
int Direction(const Vector2D& e, const Vector2D& u)
{
	Real d = e.dot(u);
	if( fabs(d) < EPS )	return 0;
	if( d < 0 )		return -1;
	return 1;
}
/*
\brief 判断点a是否在b的上方。
*/
bool IsAbove(const Point2D& a , const Point2D& b, const Vector2D& u)
{
	return u.dot(a-b) > 0;
}

/*
\brief 判断点a是否在b的下方。
*/
bool IsBelow(const Point2D& a , const Point2D& b, const Vector2D& u)
{
	return u.dot(a-b) < 0;
}
/*
\brief 二分查找法，求多边形上方向是u的极点，设凸多边形是逆时针顺序的。
*/
int ExtremePoint_BinarySearch( const Point2D* p, int n, const Vector2D& u)
{
	int	a = 0, b = n, m;
	int upA = Direction(p[1] - p[0],u) , upM;
	if( upA<=0 && !IsAbove(p[n-1],p[0],u) )
		return 0;
	while(true)
	{
		m = (a + b) / 2;
		upM = Direction(p[(m+1)%n] - p[m],u);
		if( upM<=0 && !IsAbove(p[m-1],p[m],u) )
			return m;
		if( upA>0 )
		{
			if( upM<0 )
			{				//选择[a,m]
				b = m;		
			}
			else if( IsAbove(p[a],p[m],u) )
			{				//选择[a,m]
				b = m;		
			}
			else
			{				//选择[m,b]
				a = m;
				upA = upM;
			}
		}
		else
		{
			if( upM>0 )
			{				//选择[m,b]
				a = m;
				upA = upM;
			}
			else if( IsBelow(p[a],p[m],u) )
			{				//选择[a,m]
				b = m;
			}
			else
			{				//选择[m,b]
				a = m;
				upA = upM;
			}
		}
	}
	return 0;
}
/*
\brief 暴力算法，求多边形上方向是u的极点，设凸多边形是逆时针顺序的。
*/
int ExtremePoint_Naive( const Point2D* p, int n, const Vector2D& u)
{
	int i , mxI = 0;
	int count = 0;
	Real mx = u.dot(p[0]);
	for( i=1 ; i<n ; i++ )
	{
		if( u.dot(p[i]) > mx )
		{
			mx = u.dot(p[i]);
			mxI = i;
		}
	}
	return mxI;
}

void TestExtremePoint()
{
	int i , r = 10000 , mx = 1000;
	Point2D*	p;
	Vector2D u = Vector2D(0,1.0);
	p = new Point2D[mx];
	for( i=3 ; i<mx ; i++ )
	{
		GenerateConvex(i,r,Point2D(0,0),p);
		ASSERT(ExtremePoint_Naive(p,i,u)==ExtremePoint_BinarySearch(p,i,u));
	}
	delete []p;
}


int main( )
{
	TestExtremePoint();
	return 0;
}