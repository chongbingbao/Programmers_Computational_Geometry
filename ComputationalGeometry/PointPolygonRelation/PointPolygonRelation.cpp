/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/04/24
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"
#include <stdio.h>
#include <list>
#include <time.h>


#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D

#define EPS			SR_EPS


typedef std::list<Point2D>				PointList;
typedef std::list<Point2D>::iterator	PointListIterator;

#define  ON_EDGE	0x00
#define  INSIDE		0x01
#define  OUTSIDE	0x02

/*
\brief	判断三点p0-p1-p2的朝向，若是逆时针方向，返回1；若是顺时针方向，返回-1；否则，返回0.
*/
int Orientation(const Point2D& p0, const Point2D& p1, const Point2D& p2)
{
	Real d = (p1 - p0).cross(p2 - p0);
	if(d > 0)	return  1;
	if(d < 0)	return -1;
	return 0;
}

void Divide(PointList& ptList, const Point2D& p0, const Point2D& p1, int sign, PointList& part1, PointList& part2)
{
	PointListIterator  iter;
	int r ;
	for( iter = ptList.begin() ; iter != ptList.end() ; iter++ )
	{
		r = sign * Orientation(p0, p1, *iter);
		if( r > 0 )
			part1.push_back(*iter);
		else if( r < 0 )
			part2.push_back(*iter);
	}
	ptList.clear();
}

void Partition(PointList& ptList, const Point2D& p0, const Point2D& p1, PointList& result)
{
	if( ptList.empty() )
	{
		result.push_back(p1);
		result.push_back(p0);
		return ;
	}

	//求出链表中随机选择的点，并把它从链表中移除
	int randNode = rand() % ptList.size();
	PointListIterator iter = ptList.begin();
	while( randNode-- )
		iter ++;
	Point2D t1 = *iter;
	ptList.erase(iter);

	//求出线段p0-p1之间的任意一个点
	int range = 100;
	Real t = (Real)(rand() % range) / (Real)(range);
	Point2D t0 = p0 + (p1 - p0)*t;

	PointList part1, part2;
	PointList res1, res2;

	int orient = Orientation(t0,t1,p0);
	Divide(ptList,t0,t1,orient,part1,part2);
	Partition(part1,p0,t1,res1);
	Partition(part2,t1,p1,res2);

	res1.pop_front();
	result.splice(result.begin(),res2);
	result.splice(result.end(),res1);
}

//随机取得两个不同的点
void RandomPointIndex(int n, int& p0, int& p1)
{

	p0 = rand() % n;
	do 
	{
		p1 = rand() % n;
	} while (p1==p0);
}

/*
\brief	根据输入的点集，采用空间分割法求出随机生成的简单多边形，设简单多边形是逆时针顺序的
\param[in]	pts, numPoint		规定点集
\param[out] simple,numSimple	输出简单多边形
*/
void GenerateSimplePolygon(const Point2D* pts, int numPoint, Point2D*& simple, int& numSimple)
{
	//随机取得两个不同的点
	int p0 , p1;
	RandomPointIndex(numPoint, p0, p1);

	//把点集存入链表ptList中，排除前面随机选择的两个点
	PointList ptList;
	int i;
	for( i = 0 ; i < numPoint ; ++ i )
	{
		if( i == p0 || i == p1 )
			continue;
		ptList.push_back(pts[i]);
	}

	PointList part1,part2;
	PointList res1, res2;
	PointList result;
	Divide(ptList, pts[p0], pts[p1], 1, part1, part2);
	Partition(part1,pts[p0],pts[p1],res1);
	Partition(part2,pts[p1],pts[p0],res2);

	res1.pop_front();

	result.splice(result.begin(),res2);
	result.splice(result.end(),res1);

	//输出简单多边形
	PointListIterator it;
	numSimple = result.size();
	simple = new Point2D[ numSimple ];
	i = 0;
	for(it = result.begin() ; it != result.end() ; ++ it)
	{
		simple[i++] = *it;
	}
}



int PointSimplePolygon_Crossing(const Point2D& p,const Point2D* vertex,int numVertex)
{
	int last , current;
	int count = 0 , isOut = OUTSIDE;
	SrPoint2D p1,p2;

	for(last = numVertex - 1, current = 0 ; current < numVertex ; last = current, current ++)
	{
		p1 = vertex[last];
		p2 = vertex[current];
		if( EQUAL(p1.x, p.x) && EQUAL(p1.y, p.y) )
			return ON_EDGE;
		else if( LESS(p.y,p1.y) && GEQUAL(p.y,p2.y) ||
			LESS(p.y,p2.y) && GEQUAL(p.y,p1.y) )
		{
			Real dy = p2.y - p1.y;
			Real numer = ((p.y - p1.y) * (p2.x - p1.x) + (p1.x - p.x) * dy) * dy;
			if(GREATER(numer, 0))
				isOut = (isOut == OUTSIDE ? INSIDE : OUTSIDE);
			else if(EQUAL(numer,0) )
				return ON_EDGE;
		}
		else if( EQUAL(p1.y,p2.y) && EQUAL(p1.y,p.y))
		{
			if( LESS(p.x,p1.x)&&GREATER(p.x,p2.x) ||
				LESS(p.x,p2.x)&&GREATER(p.x,p1.x))
				return ON_EDGE;
		}
	}
	return isOut;
}


int Quadrant(const Point2D& p , const Point2D& vertex)
{
	if(EQUAL(p.x,vertex.x) && EQUAL(p.y,vertex.y))	
		return 5;
	return GREATER(vertex.x, p.x) ? (GREATER(vertex.y,p.y) ? 0 : 3) : (GREATER(vertex.y, p.y) ? 1 : 2);
}

int AdjustDelta(const Point2D& p,const Point2D& v0,const Point2D& v1,int delta)
{
	if( delta == 3 )
		delta = -1;
	else if(delta == -3)
		delta = 1;
	else if(delta == 2 || delta == -2)
	{
		Real dy = v1.y - v0.y;
		Real numer = ((p.y - v0.y) * (v1.x - v0.x) + (v0.x - p.x) * dy) * dy;
		if(GREATER(numer,0))
			delta = -delta;
		else if(EQUAL(numer,0))
			return 5;
	}
	else if(delta == 1 || delta == -1)
	{
		if(EQUAL(v0.x,v1.x) && EQUAL(p.x,v0.x))
		{
			if(LESS(p.y,v0.y) && GREATER(p.y,v1.y) ||
			   GREATER(p.y,v0.y) && LESS(p.y,v1.y))
				return 5;
		}
		else if(EQUAL(v0.y,v1.y) && EQUAL(p.y,v0.y))
		{
			if(LESS(p.x,v0.x) && GREATER(p.x,v1.x) ||
			   GREATER(p.x,v0.x) && LESS(p.x,v1.x))
				return 5;
		}
	}
	return delta;
}

int PointSimplePolygon_IncrementalAngle(const Point2D& p,const Point2D* vertex,int numVertex)
{
	int last = numVertex - 1 , current = 0 , angle = 0;
	int delta, lastQuadrant = Quadrant(p,vertex[last]), currentQuadrant;
	if(lastQuadrant == 5)	
		return ON_EDGE;
	for( ; current < numVertex ;  current++ )
	{
		currentQuadrant = Quadrant(p, vertex[current]);
		if( currentQuadrant==5 )	
			return ON_EDGE;
		delta = currentQuadrant - lastQuadrant;
		delta = AdjustDelta(p, vertex[last], vertex[current], delta);
		if(delta == 5) 
			return ON_EDGE;
		angle += delta;
		last = current;
		lastQuadrant = currentQuadrant;
	}
	return (angle == 4 || angle == -4) ? INSIDE : OUTSIDE;
}


void TestPointSimplePolygon()
{
	int numPoint = 50, numSimple;
	Point2D *point = new Point2D[numPoint];
	Point2D *simple=NULL;

	int i , range = 10000;
	for( i = 0 ; i < numPoint; i ++ )
	{
		point[i].x = (rand() % 10);
		point[i].y = (rand() % 10);
	}

	GenerateSimplePolygon(point,numPoint,simple,numSimple);

	int casNum = 10000;
	Point2D p;
	for( i = 0 ; i < casNum ; i += 1 )
	{
		p.x = (rand() % 10);
		p.y = (rand() % 10);
		int res0 = PointSimplePolygon_Crossing(p,simple,numSimple);
		int res1 = PointSimplePolygon_IncrementalAngle(p,simple,numSimple);
		printf("case %d: Crossing:%d, Incremental Angle: %d\n",i + 1, res0, res1);
		ASSERT(res0 == res1);
	}

	delete []point;
	delete []simple;
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
		curAngle = 2 * SrPiF32 * partSum / sum;
		pts[i].x = -sin(curAngle) * radius + center.x;
		pts[i].y =  cos(curAngle) * radius + center.y;
	}
	delete []angle;
}

int PointConvexPolygon_BinarySearch(const Point2D& p,const Point2D* vertes, int numVertes, bool isCCW = true)
{
	int left = 0, right = 0 , middle;
	Vector2D edge ,normal, e0, e1;
	Real tmp;
	while(true)
	{
		if((right - left + numVertes) % numVertes == 1 )
		{
			edge = vertes[right] - vertes[left];
			e0 = p - vertes[left];

			if( isCCW )
				normal.set(edge.y , -edge.x);
			else
				normal.set(-edge.y , edge.x);

			tmp = normal.dot(e0);
			if( GREATER(tmp,0) )
				return OUTSIDE;
			else if( LESS(tmp,0) )
				return INSIDE;
			if( LESS(edge.dot(e0),0) )
				return OUTSIDE;

			e1 = p - vertes[right];
			if( GREATER(edge.dot(e1),0) )
				return OUTSIDE;
			return ON_EDGE;
		}
		middle = left < right ? ((left + right) >> 1):((left + right + numVertes) >> 1) % numVertes;

		edge = vertes[middle] - vertes[left];
		if( isCCW )
			normal.set(edge.y , -edge.x);
		else
			normal.set(-edge.y , edge.x);

		tmp = normal.dot(p - vertes[left]);
		if(GEQUAL(tmp,0))
			right = middle;
		else
			left = middle;
	}
}

void TestPointConvexPolygon()
{
	int num = 50, i;
	Point2D *convex = new Point2D[num];

	GenerateConvex(num,100,Point2D(0,0),convex);

	int casNum = 10000;
	Point2D p;
	for( i = 0 ; i < casNum ; i += 1 )
	{
		p.x = (rand() % 200);
		p.y = (rand() % 200);
		int res0 = PointSimplePolygon_Crossing(p,convex,num);
		int res1 = PointConvexPolygon_BinarySearch(p,convex,num);

		printf("case %d: Crossing:%d, Binary Search: %d\n",i + 1, res0, res1);
		ASSERT(res0 == res1);
	}

	delete []convex;
}


int main()
{
	TestPointSimplePolygon();
	TestPointConvexPolygon();
	return 0;
}
