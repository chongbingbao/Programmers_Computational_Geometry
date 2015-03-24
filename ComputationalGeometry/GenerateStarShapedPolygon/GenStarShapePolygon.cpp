/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/05/26
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"

#include <list>
#include <algorithm>
#include <GL/glut.h>
#include <stdio.h>
using namespace std;

#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D

typedef list<Point2D>			PointList;
typedef list<Point2D>::iterator	PointListIterator;

Point2D*	gStarShapePoly;
int			gNumStarShape;
Point2D		gCenter;

//用于qsort排序的对比函数
int Compare(const void*a , const void* b)
{
	Point2D* aa = (Point2D*)a;
	Point2D* bb = (Point2D*)b;
	if(LESS(aa->x, gCenter.x) && GREATER(bb->x,gCenter.x))
		return -1;
	else if(LESS(bb->x,gCenter.x) && GREATER(aa->x,gCenter.x))
		return 1;
	Vector2D	d, aad , bbd;
	Real	d0 , d1, aaMag, bbMag;
	aad		= *aa - gCenter;
	bbd		= *bb - gCenter;
	aaMag	= aad.magnitude();
	bbMag	= bbd.magnitude();
	if( LESS(aad.x,0) && LESS(bbd.x,0) )
	{
		d.set(0, 1.0);
		d0 = d.dot(aad) / aaMag;
		d1 = d.dot(bbd) / bbMag;
		if(GREATER(d0,d1))	
			return -1;
		else if(LESS(d0,d1))
			return 1;
		return LESS(aaMag,bbMag) ? 1 : -1;
	}
	else
	{
		d.set(0, -1.0);
		d0 = d.dot(aad) / aaMag;
		d1 = d.dot(bbd) / bbMag;
		if(GREATER(d0,d1))	
			return -1;
		else if(LESS(d0,d1))
			return 1;
		return LESS(aaMag,bbMag) ? 1 : -1;
	}
	return 0;
}


/*
\brief	计算点集的中心
*/
void Center(const Point2D* point, int numPoint)
{
	int i;
	gCenter.set(0,0);
	for( i = 0 ; i < numPoint ; i++ )
		gCenter += point[i];
	gCenter.x = gCenter.x / numPoint;
	gCenter.y = gCenter.y / numPoint;
}

/*
\brief	若中心是点集中的一个点，则把它移除
*/
int  Remove(Point2D* point, int numPoint)
{
	int i,numResult = 0;
	for( i=0 ; i<numPoint ; i++ )
		if( UNEQUAL(point[i].x,gCenter.x) || UNEQUAL(point[i].y,gCenter.y) )
			point[numResult ++] = point[i];
	return numResult;
}

/*
\brief	由给定点集，随机生成一个星状多边形,设为逆时针顺序
\param[in]	point,numPoint	输入的点集
\param[out]	starPoly,m		输出的星状多边形
*/
void GenStarShapePolygon(Point2D* point, int numPoint,Point2D*& starPoly, int &m)
{
	PointList pList;
	Center(point,numPoint);
	numPoint = Remove(point,numPoint);
	qsort(point,numPoint,sizeof(Point2D),Compare);

	Vector2D dir = Vector2D(0,1.0);
	pList.push_back(point[0]);
	Vector2D last , current = point[0] - gCenter;
	Real	lastDot, currentDot = dir.dot(last);
	int i;
	for( i = 1 ; i < numPoint ; i ++ )
	{
		last = current;
		lastDot = currentDot;
		current = point[i] - gCenter;
		currentDot = dir.dot(current) / current.magnitude();
		if( EQUAL(lastDot,currentDot) && GEQUAL(current.x*last.x,0))
			continue;
		pList.push_back(point[i]);
	}

	m = pList.size();
	starPoly = new Point2D[m];
	PointListIterator it;
	for(i = 0, it = pList.begin() ; it != pList.end() ; ++ it, ++ i)
	{
		starPoly[i] = *it;
	}
}

void Test()
{
	int numPoint = 50;
	Point2D* point = new Point2D[numPoint];
	int i;
	for( i=0 ; i<numPoint ; i++ )
	{
		point[i].x = rand() % 400;
		point[i].y = rand() % 400;
	}
	GenStarShapePolygon(point,numPoint,gStarShapePoly,gNumStarShape);

	delete []point;
}

void RenderCallback()
{
	// Clear buffers
	glClearColor(1.0f,1.0f,1.0f,1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPointSize(1.0);

	int i;
	glColor3f(0,0,0);
	glLineWidth(2.0f);
	glBegin(GL_LINES);
		for( i = 0 ; i < gNumStarShape ; ++ i )
		{
			glVertex2f(gStarShapePoly[i].x,gStarShapePoly[i].y);
			glVertex2f(gStarShapePoly[(i + 1) % gNumStarShape].x,gStarShapePoly[(i + 1) % gNumStarShape].y);
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
	int mainHandle = glutCreateWindow("Randomly Generated Monotone Polygon");
	Test();
	glutSetWindow(mainHandle);
	glutReshapeFunc(ReshapeCallback);
	glutDisplayFunc(RenderCallback);
	glutIdleFunc(IdleCallback);
	glutMainLoop();

	return 0;
}
