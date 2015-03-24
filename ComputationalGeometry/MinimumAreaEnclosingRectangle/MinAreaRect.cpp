/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/05/27
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"

#include <stdio.h>
#include <algorithm>
#include <stdlib.h>
#include <list>


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


Real CosAngle(const Vector2D&  v0, const Vector2D& v1) 
{  
	return v0.dot(v1) / (v0.magnitude()*v1.magnitude());  
}  

void ComputeDirection(const Vector2D& edge, Vector2D& parallel, Vector2D& vertical)
{  
	parallel = edge;
	parallel.normalize();
	vertical.set(-parallel.y, parallel.x);
}  

int GetMinAngleIndex(const Vector2D* ptrEdge,int index,
					 const Vector2D& parallel, const Vector2D& vertical,
					 int paralMinIndex, int paralMaxIndex, int verMaxIndex) 
{  
	int iminAngleIndex = 0;

	Real cos = 0, maxCos = FLT_MIN;  
	cos = fabs(CosAngle(ptrEdge[index], parallel));  
	if( GREATER(cos , maxCos) ){maxCos = cos; iminAngleIndex = index;}

	cos = fabs(CosAngle(ptrEdge[verMaxIndex], parallel));  
	if( GREATER(cos , maxCos) ){maxCos = cos; iminAngleIndex = verMaxIndex; }

	cos = fabs(CosAngle(ptrEdge[paralMinIndex], vertical));  
	if( GREATER(cos , maxCos) ){maxCos = cos; iminAngleIndex = paralMinIndex; }

	cos = fabs(CosAngle(ptrEdge[paralMaxIndex], vertical));  
	if( GREATER(cos , maxCos) ){maxCos = cos; iminAngleIndex = paralMaxIndex; }

	return iminAngleIndex;  
}  

void setAntiPodalPairs(const Point2D* pts, int oriIndex, int testIndex, 
					   const Vector2D& parallel, const Vector2D& vertical,  
					   Real& paralMin, Real& paraMax, Real& verMax,  
					   int& paralMinIndex, int &paralMaxIndex, int &verMaxIndex)   
{  
	Vector2D e =  pts[testIndex] - pts[oriIndex];  
	Real paraDist = parallel.dot(e);
	if(paraDist > paraMax)
	{ 
		paraMax = paraDist; 
		paralMaxIndex = testIndex;
	}
	if(paraDist < paralMin)
	{ 
		paralMin = paraDist; 
		paralMinIndex = testIndex;
	}  
	Real verDist = vertical.dot(e); 

	if(verDist > verMax)
	{ 
		verMax = verDist; 
		verMaxIndex = testIndex;
	}  
}

Real  MinAreaRectangle(const Point2D *pts, int numPts) 
{
	Real minArea = FLT_MAX;    
	Vector2D * ptrEdge = new SrVector2D[ numPts ];
	int i;
	for( i = 0; i < numPts; i++) 
	{  
		ptrEdge[i] = pts[ (i+1)%numPts ] - pts[i];  
	}
	int flushEdgeIndex = 0;
	Vector2D parallel,vertical;  
	ComputeDirection(ptrEdge[flushEdgeIndex], parallel, vertical);

	Real parallelMin = FLT_MAX, parallelMax = FLT_MIN, verticalMax = FLT_MIN;
	int parallelMinIndex, parallelMaxIndex, verticalMaxIndex;

	for( i = 0; i < numPts; i++) 
	{  
		setAntiPodalPairs(pts, flushEdgeIndex, i, 
			parallel, vertical,  
			parallelMin, parallelMax, verticalMax,  
			parallelMinIndex, parallelMaxIndex, verticalMaxIndex); 
	}

	for(i = 0; i < numPts ; i++) 
	{  
		int iminangle = 0, nextFlusEdgeIndex = (flushEdgeIndex+1)%numPts;
		iminangle = GetMinAngleIndex(ptrEdge,nextFlusEdgeIndex,parallel,vertical,parallelMinIndex, parallelMaxIndex, verticalMaxIndex);  
		if(iminangle == 0)	break;
		int swapTmp;
		if( iminangle==nextFlusEdgeIndex )
		{
			flushEdgeIndex = nextFlusEdgeIndex;
		}
		else if( iminangle==verticalMaxIndex )
		{
			flushEdgeIndex = verticalMaxIndex;
			verticalMaxIndex = nextFlusEdgeIndex;
			swapTmp = parallelMinIndex;
			parallelMinIndex = parallelMaxIndex;
			parallelMaxIndex = swapTmp;
		}
		else if( iminangle == parallelMinIndex )
		{
			flushEdgeIndex = parallelMinIndex;
			parallelMinIndex = verticalMaxIndex;
			verticalMaxIndex = parallelMaxIndex;
			parallelMaxIndex = nextFlusEdgeIndex;
		}
		else if( iminangle == parallelMaxIndex )
		{
			flushEdgeIndex = parallelMaxIndex;
			parallelMaxIndex = verticalMaxIndex;
			verticalMaxIndex = parallelMinIndex;
			parallelMinIndex = nextFlusEdgeIndex;
		}
		ComputeDirection(ptrEdge[flushEdgeIndex],parallel,vertical);

		Real dist0 = parallel.dot(pts[parallelMaxIndex] - pts[parallelMinIndex]);
		Real dist1 = vertical.dot(pts[verticalMaxIndex] - pts[flushEdgeIndex]);
		Real area = dist0 * dist1;
		if( LESS(area,minArea) )
			minArea = area;
	}
	delete []ptrEdge;
	return minArea;
}

void TestMinAreaRect()
{
	int numPoint = 10000 ;
	Point2D* points = new Point2D[numPoint];
	GenerateConvex(numPoint,100000.0,Point2D(0,0),points);

	printf("%f\n",MinAreaRectangle(points,numPoint));

	delete []points;
}

int main( )
{
	TestMinAreaRect();
	return 0;
}