/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/11/04
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"
#include <time.h>
#include <stdio.h>
#include <list>
#include <GL/glut.h>
using namespace std;


#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D

#define EPS			SR_EPS
#define INSIDE		-1
#define OUTSIDE		 1
#define ON			 0

typedef list<Point2D>	PointList;


int	gN = 10;
Point2D* gP = new Point2D[gN];

int	gM = 20;
Point2D* gQ = new Point2D[gM];

PointList gInterList;


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

bool SegInterSeg(const Point2D& p0, const Point2D& p1, const Point2D& q0, const Point2D& q1,Point2D& result)
{
	Vector2D direction1 = p1 - p0;
	Vector2D direction2 = q1 - q0;

	Real dir1Square = direction1.magnitudeSquared();
	Real dir2Square = direction2.magnitudeSquared();

	Real kcross = direction1.cross(direction2);
	Vector2D e = q0 - p0;

	if( (kcross*kcross) > EPS*EPS*dir1Square*dir2Square  )
	{//The intersection angle is not 0 based on relative error.||Cross(d1,d2)||^2/ (||d1||^2*||d2||^2)<=sin(a).
		Real s = e.cross(direction2) / kcross;
		if( LESS(s,0) || GREATER(s,1) )
			return false;
		Real t = e.cross(direction1) / kcross;
		if( LESS(t,0) || GREATER(t,1) )
			return false;
		result = p0 + s*direction1;
		return true;
	}
	return false;
}

int Next(int k ,int n)
{
	return (k + 1)%n;
}

bool IsHyperplane(const Point2D& p0, const Point2D& p1, const Point2D& q)
{
	Vector2D e = p1 - p0;
	Vector2D norm = Vector2D(-e.y, e.x);
	printf("		%f\n",norm.dot(q - p0));
	if( GREATER(norm.dot(q - p0) , 0) )
		return true;
	return false;
}

int IsPointInConv(const Point2D* v, int n, const Point2D& p)
{
	int left = 0 , right = 0 , middle;
	Vector2D edge , normal;
	Real tmp;
	while(true)
	{
		if( (right - left + n) % n == 1 )
		{
			edge = v[right] - v[left];
			Vector2D e0 = p - v[left];
			normal.set(edge.y , -edge.x);
			tmp = normal.dot(e0);
			if( GREATER(tmp,0) )
				return OUTSIDE;
			else if( LESS(tmp,0) )
				return INSIDE;
			if( LESS(edge.dot(e0),0) )
				return OUTSIDE;
			SrVector2D e1 = p - v[right];
			if( GREATER(edge.dot(e1),0) )
				return OUTSIDE;
			return ON;
		}
		middle = left < right?((left + right) >> 1):((left + right + n) >> 1) % n;

		edge = v[middle] - v[left];
		normal.set(edge.y , -edge.x);

		tmp = normal.dot(p - v[left]);
		if( GEQUAL(tmp,0) )
			right = middle;
		else
			left = middle;
	}
}

void ConvexInterConvex(const Point2D* p,int n, const Point2D* q, int m,PointList& plist)
{
	int i = 0, j = 0, count = (n << 1) + (m << 1);
	printf("%d\n",count);
	bool isInConvP;
	Point2D p0, p1, q0, q1;
	Point2D inter;
	Vector2D ep, eq;
	Real d;
	while( count-- )
	{
		p0 = p[i];
		p1 = p[Next(i,n)];
		q0 = q[j];
		q1 = q[Next(j,m)];
		printf("(%d,%d)\n",i,j);

		if( SegInterSeg(p0,p1,q0,q1,inter) )
		{
			printf("	Intersection:(%f,%f)\n",inter.x,inter.y);
			if(plist.size()!=0 && inter == *plist.begin() )
				return;
			plist.push_back(inter);
			if( IsHyperplane(q0,q1,p0) )
				isInConvP = true;
			else
				isInConvP = false;
		}
		ep = p1 - p0;
		eq = q1 - q0;
		d  = eq.cross(ep);
		printf("	%f\n",d);
		if( GEQUAL(d,0) )
		{
			if( IsHyperplane(q0,q1,p1) )
			{
				isInConvP = false;
				j = Next(j,m);
			}
			else
			{
				isInConvP = true;
				i = Next(i,n);
			}
		}
		else
		{
			if( IsHyperplane(p0,p1,q1) )
			{
				isInConvP = true;
				i = Next(i,n);
			}
			else
			{
				isInConvP = false;
				j = Next(j,m);
			}
		}
	}
	if( IsPointInConv(p,n,q[0]) == INSIDE )
	{
		printf("Conv(Q) is inside conv(P).\n");
	}
	else if( IsPointInConv(q,m,p[0]) == OUTSIDE )
	{
		printf("Conv(P) is inside Conv(Q).\n");
	}
}


void RenderCallback()
{
	// Clear buffers
	glClearColor(1.0f,1.0f,1.0f,1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPointSize(1.0);

	glColor3f(0,0,0);
	glLineWidth(2.0f);
	int i, j;
	glBegin(GL_LINES);
	//渲染凸多边形P
	for( i = gN - 1, j = 0 ; j < gN ; i = j, j += 1 )
	{
		glVertex2f(gP[i].x,gP[i].y);
		glVertex2f(gP[j].x,gP[j].y);
	}
	//渲染凸多边形Q
	for( i = gM - 1, j = 0 ; j < gM ; i = j, j += 1 )
	{
		glVertex2f(gQ[i].x,gQ[i].y);
		glVertex2f(gQ[j].x,gQ[j].y);
	}

	PointList::iterator it;
	for( it = gInterList.begin() ; it != gInterList.end() ; it ++ )
	{
		glVertex2f((*it).x,(*it).y - 20);
		glVertex2f((*it).x,(*it).y + 20);
	}
	glEnd();
	glutSwapBuffers();
}


void Init()
{
	gInterList.clear();
	GenerateConvex(gN,200,Point2D(200,200),gP);
	GenerateConvex(gM,200,Point2D(350,200),gQ);

	Point2D point;
	int i, j;
	for( i = 0 ; i < gN ; i++ )
		for( j = 0 ; j < gM ; j++ )
		{
			if( SegInterSeg(gP[i],gP[Next(i,gN)],gQ[j],gQ[Next(j,gM)],point) )
				printf("(%d,%d)\n",i,j);
		}

		ConvexInterConvex(gP,gN,gQ,gM,gInterList);
		printf("%d\n",gInterList.size());
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
	Init();

	glutInit(&argc, argv);
	glutInitWindowSize(640, 480);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	int mainHandle = glutCreateWindow("Randomly Generated Monotone Polygon");
	glutSetWindow(mainHandle);
	glutReshapeFunc(ReshapeCallback);
	glutDisplayFunc(RenderCallback);
	glutIdleFunc(IdleCallback);
	glutMainLoop();

	return 0;
}
