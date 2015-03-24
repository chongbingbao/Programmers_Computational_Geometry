/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/04/24
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"
#include <stdio.h>
#include <list>

/*
\brief	找到最左下角的顶点，最左下角的顶点有多个，则只保留一个顶点。
*/
int GrahamPivot(SrPoint2D* points,int numPoint,int& newNumPoint)
{
	int i, minIndex = 0, numIndex = 1;

	for( i=1 ; i<numPoint ; i++ )
	{		
		if( EQUAL(points[i].x,points[minIndex].x) && EQUAL(points[i].y,points[minIndex].y) )
		{
			continue;
		}
		if( LESS(points[i].x,points[minIndex].x) )
		{
			minIndex = numIndex;
		}
		else if( EQUAL(points[i].x,points[minIndex].x) && LESS(points[i].y,points[minIndex].y))
		{
			minIndex = numIndex;
		}

		points[numIndex++] = points[i];
	}
	newNumPoint = numIndex;
	return minIndex;
}

/*
\brief	对比极角 p0-pivot-p1：
		若极角小于90，return  1；
		若极角大于90，return -1；
		若极角为0且p0到pivot的距离小于p1到pivot的距离，return 1；
		若极角为0且p0到pivot的距离大于p1到pivot的距离，return -1；
		否则，p0与p1重叠，return 0。
*/
int GrahamCompare(const SrPoint2D& p0 , const SrPoint2D& p1, const SrPoint2D& pivot)
{
	SrReal angle = (p0 - pivot).cross(p1 - pivot);
	if( EQUAL(angle,0) )
	{
		SrReal dif = pivot.distanceSquared(p0) - pivot.distanceSquared(p1);
		if( EQUAL(dif,0) )
			return 0;
		return  GREATER(dif,0)?-1:1;
	}
	return GREATER(angle,0) ? 1:-1;
}
/*
\brief	采用堆排序，将顶点按照极角从小到大排序。
*/
void GrahamHeapSort(SrPoint2D * points, int numPoint , const SrPoint2D& pivot)
{
	SrPoint2D* heap = new SrPoint2D[numPoint + 1];
	int i , j , n = numPoint , child;
	for( i=0 ; i<numPoint ; i++ ) 
		heap[i+1] = points[i];
	//build a minimum heap.
	SrPoint2D y;
	for(i=numPoint/2 ; i>=1; i--)
	{
		y=heap[i];
		child=2*i;
		while(child<=numPoint)
		{
			if(child<numPoint&&GrahamCompare(heap[child],heap[child + 1],pivot)<0 )/*heap[child]>heap[child+1]*/
				child++;
			if(GrahamCompare(y,heap[child],pivot)>=0 )
				break;
			heap[child/2]=heap[child];
			child*=2;
		}
		heap[child/2]=y;
	}
	//Pop the head of heap and update the minimum heap.
	int indx = 0;
	for(j=1;j<=numPoint;j++)
	{
		y=heap[n--];
		points[indx++]=heap[1];
		i=1,child=2;
		while(child<=n)
		{
			if(child<n&&GrahamCompare(heap[child],heap[child + 1],pivot)<0)
				child++;
			if(GrahamCompare(y,heap[child],pivot)>=0)
				break;
			heap[i]=heap[child];
			i=child;
			child*=2;
		}
		heap[i]=y;
	}
	delete []heap;
}
/*
\brief	采用Graham扫描线法计算凸包。
*/
void	GrahamScanHull(const SrPoint2D* pPoints,int numPoints,std::list<SrPoint2D>& result)
{
	int i;
	SrPoint2D* buffer = new SrPoint2D[numPoints];
	for( i=0 ; i<numPoints ; i++ )
		buffer[i] = pPoints[i];
	int newNumPoints;
	int minIndex = GrahamPivot(buffer,numPoints,newNumPoints);
	
	SrPoint2D pivot;
	pivot = buffer[minIndex];
	buffer[minIndex] = buffer[0];
	buffer[0] = pivot;

	GrahamHeapSort(buffer + 1,newNumPoints - 1,pivot);

	result.push_back(buffer[0]);
	result.push_back(buffer[1]);
	std::list<SrPoint2D>::iterator last , current;
	for( i=2 ; i<newNumPoints ; i++ )
	{
		while(result.size()>=2 )
		{
			last = result.end();
			last --;
			current = last;
			last --;
			if( LEQUAL((buffer[i]-*current).cross(*last-*current),0) )
				result.pop_back();
			else
				break;
		}
		result.push_back(buffer[i]);
	}

	delete []buffer;
}
/*
\brief	用于判断给定的多边形是否是凸包
*/
bool InitConvex(const SrPoint2D* point,int numPoint,int &mNumVertex,SrPoint2D*& mVertex)
{
	if( numPoint<=2 )
		return false;
	int i;
	std::list<SrPoint2D> result;
	GrahamScanHull(point,numPoint,result);
	mNumVertex = result.size();
	if( mNumVertex<=2 )
	{
		mNumVertex = 0;
		return false;
	}
	mVertex = new SrPoint2D[mNumVertex];
	std::list<SrPoint2D>::iterator iter;
	for(iter = result.begin(), i=0 ; iter!=result.end() ; iter++,i++ )
		mVertex[i] = *iter;
	return true;
}
/*
\brief	Compare the 2d points lexicographically.p<q	<==> (px<qx) or ((px=qx)and(py<qy)).
\return	If pi<pj, return 1;
		If pi>pj, return -1;
		If pi==pj,return 0;
*/
int CompareVertex(const SrPoint2D& p0 , const SrPoint2D& p1)
{
	if( LESS(p0.x,p1.x) ) return 1;
	if( GREATER(p0.x,p1.x) ) return -1;
	if( LESS(p0.y,p1.y) ) return 1;
	if( GREATER(p0.y,p1.y) ) return -1;
	return 0;
}

/*
\brief 判断多边形是否是凸多边形
*/
bool IsConvex(SrPoint2D* vertex,int numVertex)
{
	int last , current , next;
	int sign = CompareVertex(vertex[numVertex - 1],vertex[0]) , tmpSign;
	int signChange = 0;
	for( last = 0 , current = 1; current<numVertex ; last = current, current ++ )
	{
		tmpSign = CompareVertex(vertex[last],vertex[current]);
		if( tmpSign==0 )
			return false;
		if( tmpSign!=signChange )
		{
			signChange = tmpSign;
			signChange ++;
			if( signChange>2 )
				return false;
		}
	}
	bool isGreater = false;
	SrReal angle;
	for(last = numVertex-2, current = numVertex-1, next = 0 ; next < numVertex ; last = current, current=next, next++)
	{
		angle = (vertex[next]-vertex[current]).cross(vertex[last]-vertex[current]);
		if( LESS(angle,0) )
			return false;
		else if( !isGreater && GREATER(angle,0) )
			isGreater = true;
	}
	if( isGreater )
		return true;
	//Degenerate case.
	return false;
}

void TestGrahamConvex()
{
	int numPoint = 10000, numCase = 100;
	int cs = 0;
	SrPoint2D* points = new SrPoint2D[numPoint];
	while(numCase--)
	{
		int i;
		for( i=0 ; i<numPoint ; i++ )
		{
			points[i].x = rand()%1000;
			points[i].y = rand()%1000;
		}
		SrPoint2D* result = NULL;
		int numResult;
		InitConvex(points,numPoint,numResult,result);
		if(IsConvex(result,numResult))
			printf("Case %d Succeeds!\n",++cs);
		else
			printf("Case %d Fails!\n",++cs);
		delete []result;
	}
	delete []points;
}

int main( )
{
	TestGrahamConvex();
	return 0;
}