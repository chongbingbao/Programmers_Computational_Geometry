/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/05/26
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
using namespace std;


#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D

#define EPS			SR_EPS

Point2D*	gMonotonePolygon;
int			gNumMonotonePolygon;

typedef list<Point2D>		PointList;
typedef PointList::iterator	PointListItertor;	


int Compare(const Point2D& p0 , const Point2D& p1, const Vector2D& d)
{
	Real d0, d1, c0, c1;
	d0 = d.dot(p0);
	d1 = d.dot(p1);
	if(d0 < d1)	return  1;
	if(d0 > d1)	return -1;
	c0 = d.cross(p0);
	c1 = d.cross(p1);
	if(c0 < c1)	return 1;
	if(c0 > c1) return -1;
	return 0;
}

void HeapSort(Point2D * points, int numPoint , const Vector2D& d)
{
	Point2D* heap = new Point2D[numPoint + 1];
	int i , j , n = numPoint , child;
	for( i = 0 ; i < numPoint ; i ++ ) 
		heap[i + 1] = points[i];
	//build a minimum heap.
	Point2D y;
	for(i=numPoint/2 ; i>=1; i--)
	{
		y = heap[i];
		child = 2 * i;
		while(child <= numPoint)
		{
			if(child < numPoint && Compare(heap[child],heap[child + 1], d) < 0 )	/*heap[child]>heap[child+1]*/
				child++;
			if(Compare(y,heap[child],d) >= 0)
				break;
			heap[child / 2] = heap[child];
			child *= 2;
		}
		heap[child / 2]=y;
	}
	//Pop the head of heap and update the minimum heap.
	int indx = 0;
	for(j = 1;j <= numPoint ; j ++)
	{
		y = heap[n--];
		points[indx++] = heap[1];
		i = 1,child = 2;
		while(child <= n)
		{
			if(child < n && Compare(heap[child],heap[child + 1], d) < 0)
				child++;
			if(Compare(y,heap[child],d) >= 0)
				break;
			heap[i] = heap[child];
			i = child;
			child *= 2;
		}
		heap[i] = y;
	}
	delete []heap;
}

/*
\brief	根据给定点集，生成沿着d方向的单调多边形，设单调多边形是逆时针顺序的，
		生成的单调多边形可能出现多点共线的情况
\param[in]	p, n		规定输入的点集
\param[in]	d			规定直线的方向
\param[out] q, m		输出的单调多边形
*/
void GenMonotonePolygon(const Point2D* p, int n, const Vector2D& d, Point2D*& q, int &m)
{
	int i;
	Point2D* P = new Point2D[n];
	for( i = 0 ; i < n ; ++ i )
		P[i] = p[i];

	//对点集排序
	HeapSort(P,n,d);

	//排除重复点
	int k = 1;
	for( i = 1 ; i < n ; ++ i )
		if(P[i] != P[i-1])
			P[k++] = P[i];

	//对排完序的点集，按照线段P[0]-P[k-1]，分成上、下两个点集
	Point2D begin = P[0];
	Point2D end	  = P[k - 1];
	Vector2D b	  = P[k - 1] - P[0];
	PointList top, bott;
	for( i = 1 ; i < k - 1 ; ++ i )
	{
		if(P[0] == P[n - 1])
			continue;
		Real s = b.cross(P[i] - P[0]);
		if(s > 0)
			top.push_front(P[i]);
		else if(s < 0)
			bott.push_back(P[i]);
	}
	//归并上、下两个点集，得到单调多边形
	PointListItertor it;
	m = top.size() + bott.size() + 2;
	q = new Point2D[m];

	q[0] = P[0];
	for( i = 1, it = bott.begin() ; it != bott.end() ; ++ it, ++ i )
		q[i] = *it;

	q[i++] = P[n-1];
	for( it = top.begin() ; it != top.end() ; ++ it, ++ i )
		q[i] = *it;
	delete []P;
}


void TestGenMonPolygon( )
{
	int n = 40, i;
	Point2D* p;
	p = new Point2D[n];
	for( i = 0 ; i < n ; ++ i )
	{
		p[i].x = rand() % 400;
		p[i].y = rand() % 400;
	}
	Vector2D d(1,1);
	GenMonotonePolygon(p,n,d,gMonotonePolygon,gNumMonotonePolygon);

	delete []p;
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
		for( i = 0 ; i < gNumMonotonePolygon ; ++ i )
		{
			glVertex2f(gMonotonePolygon[i].x,gMonotonePolygon[i].y);
			glVertex2f(gMonotonePolygon[(i + 1) % gNumMonotonePolygon].x,gMonotonePolygon[(i + 1) % gNumMonotonePolygon].y);
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
	TestGenMonPolygon();
	glutSetWindow(mainHandle);
	glutReshapeFunc(ReshapeCallback);
	glutDisplayFunc(RenderCallback);
	glutIdleFunc(IdleCallback);
	glutMainLoop();

	return 0;
}
