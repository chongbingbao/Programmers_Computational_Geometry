/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/11/09
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"
#include <stdio.h>
#include <list>
#include <GL/glut.h>
using namespace std;


#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D

#define EPS			SR_EPS

typedef list<Point2D>				PointList;
typedef list<Point2D>::iterator		PointListIterator;

Point2D*	gSimplePolygon;
int			gNumSimple;

Point2D*	gConvexPolygon;
int			gNumConvex;



int Orientation(const Point2D& v1, const Point2D& v2, const Point2D& v3)
{
	Real d;
	d = (v3 - v1).cross(v2 - v1);
	if(d < 0)	return -1;
	if(d > 0)	return  1;
	return 0;
}

/*
\brief	给定一个简单多边形P，找到它的凸多边形
*/
void ConvexHullSimplePolygon(const Point2D* p, int n, Point2D*& conv, int& m)
{
	PointList L;
	if( Orientation(p[0], p[1], p[2]) > 0 )
	{
		L.push_back(p[0]);
		L.push_back(p[1]);
	}
	else
	{
		L.push_back(p[1]);
		L.push_back(p[0]);
	}
	L.push_back(p[2]);
	L.push_front(p[2]);

	int i = 3;
	PointListIterator frontIter;
	PointListIterator backIter;
	Point2D db0,db1;
	Point2D dt0,dt1;
	while( i < n )
	{
		frontIter = L.begin();
		db0 = *frontIter;
		db1 = *(++frontIter);

		backIter = L.end();
		dt1 = *(--backIter);
		dt0 = *(--backIter);

		while( i < n && Orientation(p[i], db0, db1) >= 0 && Orientation(dt0, dt1, p[i]) >= 0 )
			i += 1;
		if( i >= n )
			break;
		while ( Orientation(dt0,dt1,p[i]) <= 0)
		{
			L.pop_back();
			backIter = L.end();
			dt1 = *(--backIter);
			dt0 = *(--backIter);
		}
		L.push_back(p[i]);
		while( Orientation(p[i], db0, db1) <= 0)
		{
			L.pop_front();
			frontIter = L.begin();
			db0 = *frontIter;
			db1 = *(++frontIter);
		}
		L.push_front(p[i]);
		i += 1;
	}

	//导出凸包
	m = L.size();
	PointListIterator it;
	conv = new Point2D[m];
	i = 0;
	for( it = L.begin() ; it != L.end() ; ++ it )
	{
		conv[i++] = (*it);
	}
}


void DivideSpace(PointList& ptList,const Point2D& p0,const Point2D& p1,Real flag,PointList& part1,PointList& part2)
{
	PointListIterator  iter;
	Vector2D direction = p1 - p0;
	Real ret;
	for( iter = ptList.begin() ; iter != ptList.end() ; iter++ )
	{
		ret = flag*direction.cross(*iter - p0);
		if( GREATER(ret,0) )
			part1.push_back(*iter);
		else if( LESS(ret,0) )
			part2.push_back(*iter);
	}
	ptList.clear();
}

void SpacePartition(PointList& ptList, const Point2D& p0, const Point2D& p1, PointList& result)
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

	Real flag = (t1 - t0).cross(p0 - t0);
	DivideSpace(ptList,t0,t1,flag,part1,part2);
	SpacePartition(part1,p0,t1,res1);
	SpacePartition(part2,t1,p1,res2);

	res1.pop_front();
	result.splice(result.begin(),res2);
	result.splice(result.end(),res1);
}


/*
\brief	根据输入的点集，采用空间分割法求出随机生成的简单多边形，设简单多边形是逆时针顺序的
\param[in]	pts, numPoint		规定点集
\param[out] simple,numSimple	输出简单多边形
*/
void GenerateSimplePolygon(const Point2D* pts, int numPoint, Point2D*& simple, int& numSimple)
{
	int p0 , p1;
	//随机取得两个不同的点
	p0 = rand() % numPoint;
	do 
	{
		p1 = rand() % numPoint;
	} while (p1==p0);
	PointList ptList;
	int i;
	//把点集存入链表ptList中，排除前面随机选择的两个点
	for( i=0 ; i<numPoint ; i++ )
	{
		if( i==p0 || i==p1 )
			continue;
		ptList.push_back(pts[i]);
	}

	PointList part1,part2;
	PointList res1, res2;
	DivideSpace(ptList,pts[p0],pts[p1],1.0,part1,part2);
	SpacePartition(part1,pts[p0],pts[p1],res1);
	SpacePartition(part2,pts[p1],pts[p0],res2);

	res1.pop_front();
	PointList result;
	result.splice(result.begin(),res2);
	result.splice(result.end(),res1);

	PointListIterator it;
	numSimple = result.size();
	simple = new Point2D[ numSimple ];
	i = 0;
	for(it = result.begin() ; it != result.end() ; ++ it)
	{
		simple[i++] = *it;
	}
}


void TestConvexHullSimplePolygon()
{
	int numInput = 100;
	Point2D* pointInput = new Point2D[numInput];

	int i , range = 10000;
	for( i = 0 ; i < numInput ; i++ )
	{
		pointInput[i].x = (rand() % 400);
		pointInput[i].y = (rand() % 400);
	}

	GenerateSimplePolygon(pointInput,numInput,gSimplePolygon,gNumSimple);

	ConvexHullSimplePolygon(gSimplePolygon,gNumSimple,gConvexPolygon,gNumConvex);

	delete []pointInput;
}

void RenderCallback()
{
	glClearColor(1.0f,1.0f,1.0f,1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPointSize(1.0);

	int i;
	glColor3f(0,0,0);
	glLineWidth(2.0f);
	glBegin(GL_LINES);
		for( i=0 ; i<gNumSimple ; i++ )
		{
			glVertex2f(gSimplePolygon[i].x,gSimplePolygon[i].y);
			glVertex2f(gSimplePolygon[(i + 1) % gNumSimple].x,gSimplePolygon[(i + 1) % gNumSimple].y);
		}
	glEnd();

	glColor3f(1,0,0);
	glLineWidth(2.0f);
	glBegin(GL_LINES);
		for( i=0 ; i<gNumConvex ; i++ )
		{
			glVertex2f(gConvexPolygon[i].x,gConvexPolygon[i].y);
			glVertex2f(gConvexPolygon[(i + 1) % gNumConvex].x,gConvexPolygon[(i + 1) % gNumConvex].y);
		}
	glEnd();

	glutSwapBuffers();
}

void ReshapeCallback(int width, int height)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0,0,width,height);
	gluOrtho2D(-10.0,(GLdouble)width,-10.0,(GLdouble)height);
}

void IdleCallback()
{
	glutPostRedisplay();
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitWindowSize(640, 480);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	int mainHandle = glutCreateWindow("Find the Convex Hull of a Simple Polygon");
	TestConvexHullSimplePolygon();
	glutSetWindow(mainHandle);
	glutReshapeFunc(ReshapeCallback);
	glutDisplayFunc(RenderCallback);
	glutIdleFunc(IdleCallback);

	glutMainLoop();
	return 0;
}
