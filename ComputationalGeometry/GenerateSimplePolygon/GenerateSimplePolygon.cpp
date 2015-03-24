/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/05/26
\details 
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"
#include <GL/glut.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>
#include <list>
using namespace  std;

#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D

#define EPS			SR_EPS


typedef list<Point2D>				PointList;
typedef list<Point2D>::iterator		PointListIterator;

Point2D		*gSimplePolygon;
int			gNumSimplePolygon;


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
	Divide(ptList, pts[p0], pts[p1], 1.0, part1, part2);
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

void TestGenSimplePolygon_PartitionSpace( )
{
	int numPoint = 50;
	Point2D *point = new Point2D[numPoint];

	int i , range = 10000;
	for( i=0 ; i<numPoint; i++ )
	{
		point[i].x = (rand() % 400);
		point[i].y = (rand() % 400);
	}

	GenerateSimplePolygon(point,numPoint,gSimplePolygon,gNumSimplePolygon);

	delete []point;
}

void RenderCallback()
{
	glClearColor(1.0f,1.0f,1.0f,1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPointSize(1.0);

	glColor3f(0,0,0);
	glLineWidth(2.0f);

	glBegin(GL_LINES);
		int i;
		for( i=0 ; i<gNumSimplePolygon ; i++ )
		{
			glVertex2f(gSimplePolygon[i].x,gSimplePolygon[i].y);
			glVertex2f(gSimplePolygon[(i+1)%gNumSimplePolygon].x,gSimplePolygon[(i+1)%gNumSimplePolygon].y);
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
	int mainHandle = glutCreateWindow("Randomly Generated Simple Polygon");
	TestGenSimplePolygon_PartitionSpace();
	glutSetWindow(mainHandle);
	glutReshapeFunc(ReshapeCallback);
	glutDisplayFunc(RenderCallback);
	glutIdleFunc(IdleCallback);

	glutMainLoop();
	return 0;
}
