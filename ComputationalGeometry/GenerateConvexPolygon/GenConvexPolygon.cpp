/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/05/29
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"

#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>

#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D

Point2D*		gConvexPolygon;
int				gNumConvexPolygon;
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
		curAngle = 2 * SrPiF32 * partSum / sum;
		pts[i].x = -sin(curAngle) * radius + center.x;
		pts[i].y =  cos(curAngle) * radius + center.y;
	}
	delete []angle;
}


int CompareVertex(const Point2D& p0 , const Point2D& p1)
{
	if(p0.x < p1.x) return  1;
	if(p0.x > p1.x) return -1;
	if(p0.y < p1.y) return  1;
	if(p0.y > p1.y) return -1;
	return 0;
}

/*
\brief	判断一个多边形是否是凸多边形，设输入的多边形是逆时针顺序的
\param[in]	p, n		规定输入的多边形
*/
bool IsConvex( const Point2D* p,int n)
{
	int last , current , next;
	int sign , tmpSign;
	int signChange = 0;
	sign = CompareVertex(p[n - 1],p[0]);
	for( last = 0 , current = 1; current < n ; last = current, current ++ )
	{
		tmpSign = CompareVertex(p[last], p[current]);
		if( tmpSign == 0 )
			return false;
		if( tmpSign != signChange )
		{
			signChange = tmpSign;
			signChange ++;
			if( signChange > 2 )
				return false;
		}
	}
	bool isGreater = false;
	Real angle;
	for( last = n - 2, current = n - 1, next = 0 ; next < n ; last = current, current = next, next++)
	{
		angle = (p[next] - p[current]).cross(p[last] - p[current]);
		if( angle < 0 )
			return false;
		else if( !isGreater && angle > 0 )
			isGreater = true;
	}
	if( isGreater )
		return true;
	//Degenerate case.
	return false;
}

void TestGenConvexPolygon()
{
	gNumConvexPolygon = 50;
	gConvexPolygon = new SrPoint2D[gNumConvexPolygon];
	GenerateConvex(gNumConvexPolygon,200.0f,SrPoint2D(200,200),gConvexPolygon);
	ASSERT(IsConvex(gConvexPolygon,gNumConvexPolygon));

	delete []gConvexPolygon;
}

int main(int argc, char** argv)
{
	TestGenConvexPolygon();
	return 0;
}
