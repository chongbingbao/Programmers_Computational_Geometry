/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/05/07
****************************************************************************/
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#include "SrGeometricTools.h"
#include "SrDataType.h"

#include <time.h>
#include <stdio.h>
#include <list>
#include <map>
#include <algorithm>


typedef SrPoint3D					Point;
typedef SrVector3					Vector;
typedef	unsigned char				BYTE;

class cVertex
{
public:
	cVertex()
	{
		mOnHull = false;
		mPoint.x = mPoint.y = mPoint.z = 0;
	}
public:
	Point			mPoint;				//The location information of this vertex.
	bool			mOnHull;			//Indicate whether or not the point is the extreme point of the convex polyhedron.
};


class cFacet
{
public:
	cFacet()
	{
		mVertexIndex[0]	  = -1;
		mVertexIndex[1]	  = -1;
		mVertexIndex[2]	  = -1;
	}
	void		initFace(int p0, int p1,int p2)
	{
		if( p0<p1 && p0<p2 )
		{
			mVertexIndex[0] = p0;
			mVertexIndex[1] = p1;
			mVertexIndex[2] = p2;
		}
		else if( p1<p0 && p1<p2 )
		{
			mVertexIndex[0] = p1;
			mVertexIndex[1] = p2;
			mVertexIndex[2] = p0;
		}
		else
		{
			mVertexIndex[0] = p2;
			mVertexIndex[1] = p0;
			mVertexIndex[2] = p1;
		}

	}
	bool operator < (const cFacet& facet)const
	{
		if( mVertexIndex[0]==facet.mVertexIndex[0] && mVertexIndex[1]==facet.mVertexIndex[1])
			return mVertexIndex[2]<facet.mVertexIndex[2];
		if( mVertexIndex[0]==facet.mVertexIndex[0] )
			return mVertexIndex[1]<facet.mVertexIndex[1];		
		return mVertexIndex[0]<facet.mVertexIndex[0];
	}

public:
	int		mVertexIndex[3];				//The point information of this facet.
};

class cItermVertex
{
public:
	SrPoint2D	mPoint;
	int			mIndex;
};

typedef struct
{
	int	vertexIndex[3];
}tFacet;

typedef struct
{
	SrPoint3D*	vertex;
	tFacet*		facet;
	int			numVertex;
	int			numFacet;
}tHull;

class cEdge
{
public:
	cEdge()
	{
		mVertexIndex[0] = -1;
		mVertexIndex[1] = -1;
		mCount = 0;
	}
	bool operator < (const cEdge& edge)const
	{
		if( mVertexIndex[0]==edge.mVertexIndex[0] )
			return mVertexIndex[1]<edge.mVertexIndex[1];
		return mVertexIndex[0]<edge.mVertexIndex[0];
	}
public:
	int			mVertexIndex[2];
	int			mCount;
};

typedef std::list<cFacet*>					FacetList;
typedef std::list<cFacet*>::iterator		FacetListIterator;
typedef std::list<cEdge*>					EdgeList;
typedef std::list<cEdge*>::iterator			EdgeListIterator;
typedef std::map<cEdge,cEdge*>				EdgeMap;
typedef std::map<cEdge,cEdge*>::iterator	EdgeMapIterator;
typedef std::map<cFacet,cFacet*>			FacetMap;
typedef std::map<cFacet,cFacet*>::iterator	FacetIterator;



int compareVertex(const void*	q0 , 
				  const void*	q1)
{
	cVertex* p0 = *(cVertex**)q0;
	cVertex* p1 = *(cVertex**)q1;
	if( LESS(p0->mPoint.x,p1->mPoint.x) )	return -1;
	if( GREATER(p0->mPoint.x,p1->mPoint.x) ) return 1;
	if( LESS(p0->mPoint.y,p1->mPoint.y) )	return -1;
	if( GREATER(p0->mPoint.y,p1->mPoint.y) ) return 1;
	if( LESS(p0->mPoint.z,p1->mPoint.z) )	return -1;
	if( GREATER(p0->mPoint.z,p1->mPoint.z) ) return 1;
	return 0;
}

int compare(const cVertex*		p0 , 
			const cVertex*		p1)
{
	if( LESS(p0->mPoint.x,p1->mPoint.x) )	return -1;
	if( GREATER(p0->mPoint.x,p1->mPoint.x) ) return 1;
	if( LESS(p0->mPoint.y,p1->mPoint.y) )	return -1;
	if( GREATER(p0->mPoint.y,p1->mPoint.y) ) return 1;
	if( LESS(p0->mPoint.z,p1->mPoint.z) )	return -1;
	if( GREATER(p0->mPoint.z,p1->mPoint.z) ) return 1;
	return 0;
}

SrPoint2D	gExtremePoint;

int compareAngle(const void*	q0, 
				 const void*	q1)
{
	cItermVertex* p0 = (cItermVertex*)q0;
	cItermVertex* p1 = (cItermVertex*)q1;
	SrReal angle = (p0->mPoint - gExtremePoint ).cross(p1->mPoint - gExtremePoint );
	if( EQUAL(angle,0) )
	{
		SrReal dif = gExtremePoint .distanceSquared(p0->mPoint) - gExtremePoint .distanceSquared(p1->mPoint);
		if( EQUAL(dif,0) )
			return 0;
		return  GREATER(dif,0)?1:-1;
	}
	return GREATER(angle,0) ? -1:1;
}

void setVertexFlag(cVertex**			buffer,
				   std::list<int>&		indexList)
{
	std::list<int>::iterator indexIterator;
	for( indexIterator = indexList.begin() ; indexIterator!=indexList.end() ; indexIterator++ )
	{
		buffer[*indexIterator]->mOnHull = true;
	}
}

bool getExtremePoint(cVertex**				points,
					 int					numPoint,
					 const SrVector3D&		normal,
					 const SrVector3D&		direction,
					 int					originalIndex,
					 int&					maxDistIndex,
					 std::list<int>&		indexList)
{
	int i ;
	SrReal maxRatio , vk , uk;
	SrVector3D edge;
	indexList.clear();
	for( i=0 ; i<numPoint ; i++ )
	{
		edge = points[i]->mPoint - points[originalIndex]->mPoint;
		vk = direction.dot(edge);
		uk = normal.dot(edge);
		if(!GEQUAL(uk,0))
		{
			printf("%f\n",uk);
			exit(0);
		}
		if( EQUAL(uk,0) && GEQUAL(vk,0 ) )
		{
			continue;
		}
		if( UNEQUAL(uk,0) )
		{
			maxRatio = -vk / uk;
		}
		maxDistIndex = i;
		indexList.push_back(i);
		break;
	}
	if( i==numPoint )
		return false;
	SrReal tmpRatio, tmpVk, tmpUk;
	for( i+=1 ; i<numPoint ; i++ )
	{
		edge = points[i]->mPoint - points[originalIndex]->mPoint;
		tmpVk = direction.dot(edge);
		tmpUk = normal.dot(edge);
		if(!GEQUAL(tmpUk,0))
		{
			printf("%f\n",tmpUk);
			exit(0);
		}
		if( EQUAL(tmpUk,0) && EQUAL(tmpVk,0) )
			continue;
		if( EQUAL(uk,0) )
		{
			if( EQUAL(tmpUk,0) )
			{
				if( LESS(tmpVk,vk) )
				{
					vk = tmpVk;
					uk = 0;
					maxDistIndex = i;
				}
				if( LESS(tmpVk,0) )
				{
					indexList.push_back(i);
				}
			}
		}
		else if( EQUAL(tmpUk,0) )
		{
			if( LESS(tmpVk,0) )
			{
				vk = tmpVk;
				uk = 0;
				maxDistIndex = i;
				indexList.clear();
				indexList.push_back(i);
			}
		}
		else
		{
			tmpRatio = - tmpVk / tmpUk;
			if( GREATER(tmpRatio, maxRatio) )
			{
				vk = tmpVk;
				uk = tmpUk;
				maxRatio = tmpRatio;
				maxDistIndex = i;
				indexList.clear();
				indexList.push_back(i);
			}
			else if( EQUAL(tmpRatio,maxRatio) )
			{
				if( GREATER(tmpUk,uk) )
				{
					vk = tmpVk;
					uk = tmpUk;
					maxDistIndex = i;
				}
				indexList.push_back(i);
			}
		}
	}
	points[maxDistIndex]->mOnHull = true;
	return true;
}



cFacet* initFirstFacet(cVertex** points,int numPoint)
{
	int index[3];
	index[0] = 0;
	points[index[0]]->mOnHull = true;
	SrVector3D normal ,direction;
	SrReal uk,vk;
	normal.set(1,0,0);
	direction.set(0,1,0);
	std::list<int>	indexList;
	if( !getExtremePoint(points,numPoint,normal,direction,index[0],index[1],indexList) )
		return NULL;
	//vertexIndex[1] = *indexList.begin();
	SrVector3D edgeVector = points[index[1]]->mPoint - points[index[0]]->mPoint;
	vk = direction.dot(edgeVector);
	uk = normal.dot(edgeVector);
	normal = normal*(-vk) + direction*uk;
	//SrReal d = -normal.dot(points[vertexIndex[0]]->mPoint);
	//int i;
	//for( i=0 ; i<numPoint ; i++ )
	//{
	//	if(LESS(normal.dot(points[i]->mPoint) + d,0))
	//	{
	//		ASSERT(0);
	//	}
	//}
	direction = edgeVector.cross(normal);
	if( !getExtremePoint(points,numPoint,normal,direction,index[0],index[2],indexList) )
		return NULL;
	//vertexIndex[2] = *indexList.begin();
	cFacet* facet = new cFacet();
	edgeVector = points[index[2]]->mPoint - points[index[0]]->mPoint;
	vk = direction.dot(edgeVector);
	uk = normal.dot(edgeVector);
	normal = normal*(-vk) + direction*uk;
	//d = -normal.dot(points[vertexIndex[0]]->mPoint);
	//for( i=0 ; i<numPoint ; i++ )
	//{
	//	if(LESS(normal.dot(points[i]->mPoint) + d,0))
	//	{
	//		ASSERT(0);
	//	}
	//}
	//normal.normalize();
	//printf("Normal:%.f,%.f,%.f\n",normal.x,normal.y,normal.z);
	facet->initFace(index[0],index[2],index[1]);
	//cVertex* p0 = points[facet->mVertexIndex[0]];
	//cVertex* p1 = points[facet->mVertexIndex[1]];
	//cVertex* p2 = points[facet->mVertexIndex[2]];
	////printf("Initial:	%d,%d,%d\n",facet->mVertexIndex[0],facet->mVertexIndex[1],facet->mVertexIndex[2]);
	//normal = (p1->mPoint - p0->mPoint).cross(p2->mPoint - p0->mPoint);
	//normal.normalize();
	//printf("Normal:%.f,%.f,%.f\n",normal.x,normal.y,normal.z);
	//SrReal d = -normal.dot(p0->mPoint);
	//for( int i=0 ; i<numPoint ; i++ )
	//{
	//	if(GREATER(normal.dot(points[i]->mPoint) + d,0))
	//	{
	//		ASSERT(0);
	//	}
	//}

	return facet;
}

void point3dProjectPoint2d(const SrPoint3D&		point,
						   SrPoint2D&			result,
						   int					axis)
{
	if( axis==SR_X_AXIS )
	{
		result.x = point.y;
		result.y = point.z;
	}
	else if( axis==SR_Y_AXIS )
	{
		result.x = point.z;
		result.y = point.x;
	}
	else
	{
		result.x = point.x;
		result.y = point.y;
	}
}

bool removeDuplicate(cVertex**			buffer , 
					 int				numPoint, 
					 int&				sizePoint)
{
	int i;
	qsort(buffer,numPoint,sizeof(buffer[0]),compareVertex);
	if( compare(buffer[0],buffer[numPoint-1])==0 )
	{
		sizePoint = numPoint ;
		return false;
	}
	sizePoint = 0;
	for( i=1 ; i<numPoint ; i++ )
	{
		if( compare(buffer[i],buffer[sizePoint])==0 )
		{
			delete buffer[i];
			buffer[i] = NULL;
		}
		else
		{
			buffer[++sizePoint] = buffer[i];
		}
	}
	sizePoint += 1;
	if( sizePoint<=3 )
		return false;

	return true;
}

bool coplanar(cVertex**			buffer , 
			  int						sizePoint)
{
	int i;
	cVertex*	p0 = buffer[0];
	cVertex*	p1 = buffer[sizePoint - 1];
	SrVector3D normal;
	SrReal d, dist;
	for( i=1 ; i<sizePoint-1 ; i++ )
	{
		normal = (p1->mPoint - p0->mPoint).cross(buffer[i]->mPoint - p0->mPoint);
		if( GREATER(normal.magnitudeSquared(),0) )
			break;
	}
	if( i==sizePoint-1 )
		return false;
	d = -normal.dot(p0->mPoint);
	cVertex*	p2 = buffer[i];
	for( ; i<sizePoint-1 ; i++ )
	{
		dist = normal.dot(buffer[i]->mPoint) + d;
		if( UNEQUAL(dist,0) )
			break;
	}
	if( i==sizePoint-1 )
		return false;

	return true;
}

void allocateVertex(const SrPoint3D*	point, 
					int					numPoint,
					cVertex**&			buffer)
{
	int i ;
	cVertex* newVertex;
	buffer = new cVertex*[numPoint];
	for( i=0 ; i<numPoint ; i++ )
	{
		newVertex	 =  new cVertex();
		newVertex->mPoint.x	 = point[i].x;
		newVertex->mPoint.y	 = point[i].y;
		newVertex->mPoint.z	 = point[i].z;
		buffer[i] = newVertex;
	}
}

void deallocateVertex(cVertex**&		buffer , 
					  int			sizePoint)
{
	int i;
	for( i=0 ; i<sizePoint ; i++ )
		delete buffer[i];
	delete []buffer;
	buffer = NULL;
}

void deallocateEdge(EdgeMap& edgeMap)
{
	EdgeMapIterator edgeIterator;
	for( edgeIterator = edgeMap.begin() ; edgeIterator!=edgeMap.end() ; edgeIterator ++ )
	{
		delete edgeIterator->second;
	}
	edgeMap.clear();
}

void degenerate(cVertex**				vertex,
				cItermVertex*			itermVertex,
				const SrVector3D&		normal,		//点集所在的面的法向量
				const cEdge*			edge,		//边edge的两个端点和链表indexList中的点共面
				std::list<int>&			indexList)
{
	std::list<int>::iterator indexIterator;
	int numInput = 0, maxAxis = 0, i;
	SrReal x = fabs(normal.x);
	SrReal y = fabs(normal.y);
	SrReal z = fabs(normal.z);
	if( GEQUAL(x,y) && GEQUAL(x,z) )
		maxAxis = SR_X_AXIS;
	else if( GEQUAL(y,x) && GEQUAL(y,z) )
		maxAxis = SR_Y_AXIS;
	else
		maxAxis = SR_Z_AXIS;
	itermVertex[0].mIndex = edge->mVertexIndex[0];
	point3dProjectPoint2d(vertex[edge->mVertexIndex[0]]->mPoint,itermVertex[0].mPoint,maxAxis);
	itermVertex[1].mIndex = edge->mVertexIndex[1];
	point3dProjectPoint2d(vertex[edge->mVertexIndex[1]]->mPoint,itermVertex[1].mPoint,maxAxis);
	numInput = 2;
	for( indexIterator = indexList.begin() ; indexIterator!=indexList.end() ; indexIterator ++ )
	{
		itermVertex[numInput].mIndex = *indexIterator;
		point3dProjectPoint2d(vertex[*indexIterator]->mPoint,itermVertex[numInput].mPoint,maxAxis);
		numInput ++;
	}
	gExtremePoint  = itermVertex[0].mPoint;
	qsort(itermVertex+1,numInput - 1,sizeof(cItermVertex),compareAngle);
	int numResult = 1;
	for( i=2 ; i<numInput ; i++ )
	{
		while(numResult>=1 )
		{
			SrReal tmp = (itermVertex[i].mPoint-itermVertex[numResult].mPoint).cross(itermVertex[numResult - 1].mPoint-itermVertex[numResult].mPoint);
			if( LEQUAL(tmp,0) )
				numResult --;
			else
				break;
		}
		itermVertex[++numResult] = itermVertex[i];
	}
	indexList.clear();
	indexList.push_back(edge->mVertexIndex[0]);
	indexList.push_back(edge->mVertexIndex[1]);
	if( itermVertex[1].mIndex==edge->mVertexIndex[1] )
	{
		for( i=2 ; i<=numResult ; i++ )
		{
			indexList.push_back(itermVertex[i].mIndex);
		}
	}
	else
	{
		for( i=numResult-1 ; i>=1 ; i-- )
		{
			indexList.push_back(itermVertex[i].mIndex);
		}
	}
}

//Insert edge endpointIndex1 --> endpointIndex2.
bool insertEdge(int					endpointIndex1, 
				int					endpointIndex2,
				EdgeMap&			edgePool)
{
	cEdge tmpEdge;
	EdgeMapIterator edgeIterator;
	tmpEdge.mVertexIndex[0] = endpointIndex2;
	tmpEdge.mVertexIndex[1] = endpointIndex1;
	edgeIterator = edgePool.find(tmpEdge);
	if( edgeIterator!=edgePool.end() )
	{
		//edgePool.erase(edgeIterator);
		edgeIterator->second->mCount += 1;
		return false;
	}
	tmpEdge.mVertexIndex[0] = endpointIndex1;
	tmpEdge.mVertexIndex[1] = endpointIndex2;
	edgeIterator = edgePool.find(tmpEdge);
	if( edgeIterator!=edgePool.end() )
	{
		edgeIterator->second->mCount += 1;
		return false;
	}
	cEdge* newEdge = new cEdge();
	newEdge->mVertexIndex[0] = endpointIndex1;
	newEdge->mVertexIndex[1] = endpointIndex2;
	newEdge->mCount = 1;
	edgePool.insert(std::pair<cEdge,cEdge*>(*newEdge,newEdge));
	return true;

}

void exportHull(cVertex**			buffer,
				int					sizePoint,
				FacetList&			convexFacetList,
				tHull*				hull)
{
	FacetListIterator facetIterator;
	//for( facetIterator = convexFacetList.begin() ; facetIterator!=convexFacetList.end() ; facetIterator ++ )
	//{
	//	cFacet* facet = (*facetIterator);
	//	printf("Facet Index:%d,%d,%d\n",facet->mVertexIndex[0],facet->mVertexIndex[1],facet->mVertexIndex[2]);
	//}
	int* reorder = new int[sizePoint];
	int numIndex = 0 , i;
	for( i=0 ; i<sizePoint ; i++ )
	{
		if( buffer[i]->mOnHull )
		{
			reorder[i] = numIndex++;
			//printf("%d,%d\n",i,reorder[i]);
		}
		else
		{
			delete buffer[i];
			buffer[i] = NULL;
		}
	}
	hull->numVertex = numIndex;
	hull->vertex = new SrPoint3D[numIndex];
	hull->numFacet = convexFacetList.size();
	hull->facet = new tFacet[hull->numFacet];
	for( i=0 ; i<sizePoint ; i++ )
	{
		if( buffer[i]!=NULL )
		{
			hull->vertex[reorder[i]] = buffer[i]->mPoint;
			delete buffer[i];
		}
	}
	delete []buffer;
	int facetIndex = 0;
	for( facetIterator = convexFacetList.begin() ; facetIterator!=convexFacetList.end() ; facetIterator ++ )
	{
		cFacet* facet = (*facetIterator);
		hull->facet[facetIndex].vertexIndex[0] = reorder[facet->mVertexIndex[0]];
		hull->facet[facetIndex].vertexIndex[1] = reorder[facet->mVertexIndex[1]];
		hull->facet[facetIndex].vertexIndex[2] = reorder[facet->mVertexIndex[2]];
		facetIndex ++;
		delete facet;
	}
	delete []reorder;

	int j;
	for( i=0 ; i<hull->numFacet ; i++ )
		for( j=i+1 ; j<hull->numFacet ; j++ )
			if( hull->facet[i].vertexIndex[0] == hull->facet[j].vertexIndex[0] &&
				hull->facet[i].vertexIndex[1] == hull->facet[j].vertexIndex[1] &&
				hull->facet[i].vertexIndex[2] == hull->facet[j].vertexIndex[2])
				ASSERT(0);

}

void initEdgeSet(cVertex** vertex,cFacet* facet,EdgeMap& edgePool)
{
	int i;
	cEdge* edge;
	for( i=0 ; i<3 ; i++ )
	{
		edge = new cEdge();
		edge->mVertexIndex[0] = facet->mVertexIndex[i];
		edge->mVertexIndex[1] = facet->mVertexIndex[(i+1)%3];
		edge->mCount += 1;
		edgePool.insert(std::make_pair(*edge,edge));
	}
}


cEdge* getNextEdge(EdgeList& edgeList,EdgeMap& edgePool)
{
	EdgeListIterator edgeListIterator;
	EdgeMapIterator	 edgeMapIterator;
	cEdge tmpEdge;

	while(!edgeList.empty())
	{
		edgeListIterator=edgeList.begin();
		tmpEdge.mVertexIndex[0] = (*edgeListIterator)->mVertexIndex[0];
		tmpEdge.mVertexIndex[1] = (*edgeListIterator)->mVertexIndex[1];
		edgeMapIterator = edgePool.find(tmpEdge);
		ASSERT(edgeMapIterator!=edgePool.end());
		if( edgeMapIterator->second->mCount<=1 )
			break;
		edgeList.erase(edgeListIterator);
	}
	if(edgeList.empty())
		return NULL;
	cEdge* facet = edgeMapIterator->second;

	return facet;
}

cEdge* getNextEdge(cFacet* facet,int index,EdgeMap& edgeMap)
{
	cEdge edge[2];
	int edgeIndex0 = facet->mVertexIndex[index];
	int edgeIndex1 = facet->mVertexIndex[(index+1)%3];
	edge[0].mVertexIndex[0] = edgeIndex0;
	edge[0].mVertexIndex[1] = edgeIndex1;
	edge[1].mVertexIndex[0] = edgeIndex1;
	edge[1].mVertexIndex[1] = edgeIndex0;
	EdgeMapIterator iterator;
	int i;
	for( i=0 ; i<2 ; i++ )
	{
		iterator = edgeMap.find(edge[i]);
		if( iterator!=edgeMap.end() )
		{
			if( iterator->second->mCount>=2 )
			{
				return NULL;
			}
			else
			{
				iterator->second->mCount += 1;
				return iterator->second;
			}
		}
	}
	//ASSERT(0);
	return NULL;
}



bool giftWrapping(const SrPoint3D* points, int numPoint,tHull* hull)
{
	int sizePoint;
	cVertex**vertex = NULL;
	allocateVertex(points,numPoint,vertex);
	if( !removeDuplicate(vertex,numPoint,sizePoint) )
	{
		deallocateVertex(vertex,sizePoint);
		return false;
	}
	//debug
	int i;
	for( i=0 ; i<sizePoint ; i++ )
		printf("%d:%.f,%.f,%.f\n",i,vertex[i]->mPoint.x,vertex[i]->mPoint.y,vertex[i]->mPoint.z);
	printf("\n");
	if( !coplanar(vertex,sizePoint) )
	{
		deallocateVertex(vertex,sizePoint);
		return false;
	}
	printf("Size of Points:%d\n",sizePoint);

	cFacet* facet = initFirstFacet(vertex,sizePoint);

	SrVector3D normal , direction;
	FacetList convexFacetList;
	EdgeMap edgeMap;
	cEdge* edge;
	cVertex* p0,*p1,*p2;
	cVertex* q0,*q1,*q2;

	convexFacetList.push_back(facet);
	FacetList	facetPool , candidateFacetList;
	candidateFacetList.push_back(facet);
	initEdgeSet(vertex,facet,edgeMap);
	int maxDistIndex;
	cItermVertex* itermediate = new cItermVertex[sizePoint];

	printf("%d,%d,%d\n",facet->mVertexIndex[0],facet->mVertexIndex[1],facet->mVertexIndex[2]);
	int caseNum = 0;
	while(!candidateFacetList.empty())
	{
	
		printf("Case :%d, candidate set:%d\n",++caseNum,candidateFacetList.size());
		cFacet* candidateFacet = *candidateFacetList.begin();
		candidateFacetList.pop_front();
		p0 = vertex[candidateFacet->mVertexIndex[0]];
		p1 = vertex[candidateFacet->mVertexIndex[1]];
		p2 = vertex[candidateFacet->mVertexIndex[2]];
		printf("Facet: %d,%d,%d\n",candidateFacet->mVertexIndex[0],candidateFacet->mVertexIndex[1],candidateFacet->mVertexIndex[2]);
		normal = (p1->mPoint - p0->mPoint).cross(p2->mPoint - p0->mPoint);
		normal = -normal;

		for( i=0 ; i<3 ; i++ )
		{
			edge = getNextEdge(candidateFacet,i,edgeMap);
			if( edge==NULL )
				continue;
			printf("	Edge(%d,%d)\n",edge->mVertexIndex[0],edge->mVertexIndex[1]);
			std::list<int>	indexList;

			q0 = vertex[edge->mVertexIndex[1]];
			q1 = vertex[edge->mVertexIndex[0]];
			direction = normal.cross(q1->mPoint - q0->mPoint);
			ASSERT(getExtremePoint(vertex,sizePoint,normal,direction,edge->mVertexIndex[0],maxDistIndex,indexList));
			q2 = vertex[maxDistIndex];
			SrVector3D newFacetNormal = (q1->mPoint - q0->mPoint).cross(q2->mPoint - q0->mPoint);

			std::list<int>::iterator indexIterator;
			if( indexList.size()>1 )
			{
				degenerate(vertex,itermediate,newFacetNormal,edge,indexList);
			}
			else
			{
				indexList.push_front(edge->mVertexIndex[1]);
				indexList.push_front(edge->mVertexIndex[0]);
			}
			setVertexFlag(vertex,indexList);
			for( indexIterator = indexList.begin(); indexIterator!=indexList.end() ; indexIterator ++ )
			{
				printf("%d ",*indexIterator);
			}
			printf("\n");
			indexIterator = indexList.begin();
			int p0Index = *indexIterator;
			indexIterator ++;
			int p1Index = *indexIterator;
			indexIterator ++;
			int p2Index = *indexIterator;
			while(indexIterator!=indexList.end())
			{
				indexIterator ++;
				printf("		Edge(%d,%d)\n",p2Index,p1Index);
				printf("		Edge(%d,%d)\n",p0Index,p2Index);
				bool result = insertEdge(p2Index,p1Index,edgeMap);
				facet = new cFacet();
				facet->initFace(p0Index,p2Index,p1Index);
				printf("			Insert Facet(%d,%d,%d)\n",facet->mVertexIndex[0],facet->mVertexIndex[1],facet->mVertexIndex[2]);
				convexFacetList.push_back(facet);
				candidateFacetList.push_back(facet);

				if( indexIterator==indexList.end() )
					result = insertEdge(p0Index,p2Index,edgeMap);
				p1Index = p2Index;
				if( indexIterator!=indexList.end() )
					p2Index = *indexIterator;
			}

		}
	}
	deallocateEdge(edgeMap);
	delete []itermediate;

	exportHull(vertex,sizePoint,convexFacetList,hull);

	return true;
}

bool isConvex(tHull* hull)
{
	int i , j;
	SrVector3D normal;
	SrReal	d;
	for( i=0 ; i<hull->numFacet ; i++ )
	{
		SrPoint3D v0 = hull->vertex[hull->facet[i].vertexIndex[0]];
		SrPoint3D v1 = hull->vertex[hull->facet[i].vertexIndex[1]];
		SrPoint3D v2 = hull->vertex[hull->facet[i].vertexIndex[2]];
		normal = (v1 - v0).cross(v2 - v0);
		d = -normal.dot(v0);
		for( j=0 ; j<hull->numVertex ; j++ )
		{
			SrReal result = normal.dot(hull->vertex[0]) + d;
			if( GREATER(result,0) )
				return false;
		}
	}
	return true;
}

void testgiftWrapping3D()
{
	int numPoint = 9 , i;
	SrPoint3D* point  = new SrPoint3D[numPoint];

	//freopen("tt.txt","w",stdout);
	for( i=0 ; i<numPoint ; i++ )
	{
		point[i].x = rand()%3;
		point[i].y = rand()%3;
		point[i].z = rand()%3;
	}

	tHull hull;

	double timeCount = clock();

	if( giftWrapping(point,numPoint,&hull) )
	{
		timeCount = (clock() - timeCount)/CLOCKS_PER_SEC;
		printf("time:%.4f\n",timeCount);
		printf("Number of Facets:%d, Number of vertexes:%d\n",hull.numFacet,hull.numVertex);
		ASSERT(isConvex(&hull));
		delete[] hull.facet;
		delete[] hull.vertex;
	}
	delete []point;
}

int main( )
{

	testgiftWrapping3D();

	_CrtDumpMemoryLeaks();

	return 0;
}