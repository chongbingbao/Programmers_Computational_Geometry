/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/11/02
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"

#include <stdio.h>
#include <math.h>

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
\brief	采用CyrusBeck法，计算线性对象与凸多边形的相交，设凸多边形是逆时针顺序的。
\param [in]	vertes		多边形顶点数组
\param [in] numVertes	多边形的顶点个数
\param [in] base,direction	指定线性对象，线性对象的参数方程表示为 base + t * direction
*/
int LineInterConvex_CyrusBeck(const Point2D* vertes, int numVertes,
							  const Point2D& base,const Vector2D& direction,
							  Point2D* res)
{
	Real numer , denom;
	Real tIn = MIN_VALUE, tOut = MAX_VALUE;

	Real tHit;
	Vector2D  tmp , normal;
	int last , current;
	for( last = numVertes-1 ,current = 0; current < numVertes ; last = current , current++ )
	{
		tmp = vertes[current] - vertes[last];
		normal.set( tmp.y,-tmp.x );
		tmp = vertes[current] - base;
		numer = normal.dot(tmp);
		denom = normal.dot(direction);
		if( denom < 0 )
		{//Ray is entering
			tHit = numer / denom;
			if( tHit > tOut )
				return 0;
			else if( tHit > tIn )
				tIn = tHit;
		}
		else if ( denom > 0 )
		{//Ray is exiting
			tHit = numer / denom;
			if( tHit < tIn )
				return 0;
			if( tHit < tOut )
				tOut = tHit;
		}
		else if( numer< 0 )
			return 0;
	}
	res[0] = base + direction * tIn;
	if( tIn == tOut )
		return 1;
	res[1] = base + direction * tOut;
	return 2;
}


/*
\brief	判断e与u是否同方向。
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
\brief	判断点a是否在b的上方。
*/
bool IsAbove(const Point2D& a , const Point2D& b, const Vector2D& u)
{
	return u.dot(a-b) > 0;
}
/*
\brief	判断点a是否在b的下方。
*/
bool IsBelow(const Point2D& a , const Point2D& b, const Vector2D& u)
{
	return u.dot(a-b) < 0;
}
/*
\brief	二分查找法，求多边形上方向是u的极点，设凸多边形是逆时针顺序的。
*/
int ExtremePoint( const Point2D* p, int n, const Vector2D& u)
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
\brief	判断直线与线段是否相交，若相交，返回交点。
\param[in] point0, point1	规定线段的两个端点
\param[in] base1, dir1		规定直线的参数方程
\param[out] res				若相交，返回交点。
*/
bool LineInterSegment(const Point2D& point0, const Point2D& point1,
					  const Point2D& base1, const Vector2D& dir1,
					  Point2D &res)
{
	Vector2D dir0 = point1 - point0;
	Real dirLenSquare = dir0.magnitudeSquared();
	Real kcross = dir0.cross(dir1);
	Vector2D e = base1 - point0;
	if( (kcross*kcross)/dirLenSquare>=EPS*EPS )
	{//The intersection angle is not 0 based on relative error.||Cross(d1,d2)||^2/ (||d1||^2*||d2||^2)<=sin(a).
		Real s = e.cross(dir1) / kcross;
		//If s<0 and s>1, intersected point isn't on the segment.
		if( s < 0 || s > 1.0 )
			return false;
		//Intersecting.
		res = point0 + dir0 * s;
		return true;
	}
	return false;
}

/*
\brief	判断两个实数s和t是否同号，可以确保s和t不为零。
*/
bool Diff(Real s,Real t)
{
	if( (s<0&&t<0) || (s>0&&t>0) )
		return false;
	return true;
}

Point2D BinarySearch(const Point2D* point, int numPoint, 
					 const Point2D& base, const Vector2D& dir, 
					 const Vector2D& normal,Real d, 
					 int left, int right)
{
	int mid;
	Real s, t;
	s = normal.dot(point[left]) + d;
	if( fabs(s) < EPS )
	{
		return point[left];
	}
	//printf("	s:%f,t:%f\n",s,t);
	//ASSERT(Diff(s,t));
	while( (left+1) != right )
	{
		mid = (left + right) >> 1;
		t = normal.dot(point[mid]) + d;
		if( fabs(t) < EPS )
		{
			return point[mid];
		}
		else if( Diff(s,t) )
		{
			right = mid;
		}
		else
		{
			left = mid;
		}
	}
	Point2D res;
	ASSERT(LineInterSegment(point[left],point[right % numPoint], base,dir, res));
	return res;
}


/*
\brief	采用二分查找法，计算线性对象与凸多边形的相交，设凸多边形是逆时针顺序的。
*/
int LineInterConvex_Binary(const Point2D* point, int numPoint,const Point2D&base, const Vector2D& dir, Point2D* res)
{
	Vector2D normal , u;
	normal.x = -dir.y;
	normal.y =  dir.x;
	if(normal.dot(point[0] - base) <= 0)
		u =  normal;
	else
		u = -normal;
	int i = ExtremePoint(point,numPoint,u);
	Real dist = u.dot(point[i] - base);
	if( fabs(dist) < EPS )
	{
		res[0] = point[i];
		return 1;
	}
	else if( dist < 0 )
	{
		return 0;
	}
	Real d = -u.dot(base);
	res[0] = BinarySearch(point,numPoint,base,dir,u,d,0,i);
	res[1] = BinarySearch(point,numPoint,base,dir,u,d,i,numPoint);
	return 2;
}

bool Equal(const Point2D& p , const Point2D& q )
{
	if( EQUAL(p.x,q.x) && EQUAL(p.x,q.x) )
		return true;
	return false;
}

void TestLineInterConvex()
{
	int num = 2000, i;
	Point2D res0[2],res1[2];
	Point2D* v = new Point2D[num];
	GenerateConvex(num,10,Point2D(0,0),v);
	int numLine = 1000;
	Point2D* base = new Point2D[numLine];
	Point2D* dir  = new Point2D[numLine];
	for( i = 0 ; i < numLine ; ++i )
	{
		do 
		{
			base[i].x = rand() % 20;
			base[i].y = rand() % 20;

			dir[i].x  = rand() % 20;
			dir[i].y  = rand() % 20;

		} while (dir[i].magnitudeSquared()<=0);
	}
	for( i = 0 ; i < numLine ; ++ i )
	{
		printf("Line %d:\n",i + 1);
		int flag0 = LineInterConvex_CyrusBeck(v,num,base[i],dir[i],res0);
		int flag1 = LineInterConvex_Binary(v,num,base[i],dir[i],res1);
		printf("	Number of Intersection: %d\n",flag0);
		//ASSERT(flag0 == flag1);
		//if( flag0==1 )
		//{
		//	ASSERT( Equal(res0[0],res1[0]) );
		//}
		//else if( flag0 == 2 )
		//{
		//	ASSERT( (Equal(res0[0],res1[0])&&Equal(res0[1],res1[1])) || (Equal(res0[0],res1[1])&&Equal(res0[1],res1[0])) );
		//}
	}
	delete []v;
}

int main()
{
	TestLineInterConvex();
	return 0;
}