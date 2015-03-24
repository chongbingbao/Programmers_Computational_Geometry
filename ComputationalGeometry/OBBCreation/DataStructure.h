#ifndef DATA_STRUCTURE_H_
#define DATA_STRUCTURE_H_
/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/04/03
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"
#include <list>
#include <assert.h>
#include <map>
/** \addtogroup algorithms
  @{
*/



#define  FACET_NULL			0x00
#define  FACET_VISITED		0x01
#define  FACET_BORDER		0x02

typedef SrPoint3D					cPoint;
typedef SrVector3					cVector;

class cFacet;
class cEdge;
class cOutsideSet;
class cVertex;

typedef std::list<cVertex*>			VertexList;
typedef VertexList::iterator		VertexIterator;

typedef std::list<cFacet*>			FacetList;
typedef FacetList::iterator			FacetIterator;

typedef std::map<cVertex*,cEdge*>	BoundaryEdgeMap;
typedef BoundaryEdgeMap::iterator	BoundaryEdgeIterator;

class cVertex
{
public:
	cVertex()
	{
		mOnHull = false;
		mPoint.x = mPoint.y = mPoint.z = 0.0f;
	}
public:
	bool			mOnHull;			//Indicate whether or not the point is the extreme point of the convex polyhedron.
	cPoint			mPoint;				//The location information of this vertex.
};

class cOutsideSet
{
public:
	cOutsideSet(){}
	~cOutsideSet()
	{
		mVertexList.clear();
	}
public:
	VertexList	mVertexList;			//Container of outside point set.
};

class cPlane
{
public:
	cPlane(){}
	~cPlane(){}
	bool initPlane(const cPoint& p0, const cPoint& p1, const cPoint& p2)
	{
		mNormal	= (p1 - p0).cross(p2 - p0);
		if( mNormal.isZero() )
			return false;
		//It's not necessary to normalize the vector here.
		//normal.normalize();
		mD		= -mNormal.dot(p0);
		return true;
	}
	bool isValid() const
	{
		return !mNormal.isZero();
	}
	SrReal distance(const cPoint& point) const
	{
		return mNormal.dot(point) + mD;
	}
	bool isOnPositiveSide(const cPoint& p) const
	{
		return GREATER(distance(p),0);
	}

public:
	cVector		mNormal;
	SrReal		mD;
};

class cFacet
{
public:
	cFacet()
	{
		mNeighbor[0]  = NULL;
		mNeighbor[1]  = NULL;
		mNeighbor[2]  = NULL;
		mVertex[0]	  = NULL;
		mVertex[1]	  = NULL;
		mVertex[2]	  = NULL;
		mOutsideSet	  = NULL;
		mVisitFlag	  = FACET_NULL;
		mIterator	  = NULL;

	}
	void clear()
	{
		mNeighbor[0] = NULL;
		mNeighbor[1] = NULL;
		mNeighbor[2] = NULL;
		mVertex[0]	  = NULL;
		mVertex[1]	  = NULL;
		mVertex[2]	  = NULL;
		mOutsideSet	 = NULL;
		mVisitFlag	= FACET_NULL;

	}
	~cFacet()
	{
		if( mIterator )
		{
			delete mIterator;
			mIterator = NULL;
		}
		if( mOutsideSet )
		{
			delete mOutsideSet;
			mOutsideSet = NULL;
		}
	}
	void		initFace(cVertex* p0, cVertex* p1,cVertex* p2)
	{
		mVertex[0] = p0;
		mVertex[1] = p1;
		mVertex[2] = p2;
	}
	void		setNeighbors(cFacet* f0,cFacet* f1,cFacet* f2)
	{
		mNeighbor[0] = f0;
		mNeighbor[1] = f1;
		mNeighbor[2] = f2;
	}
	const VertexIterator furthestVertex()const
	{
		ASSERT(mOutsideSet!=NULL);
		cPlane plane;
		ASSERT(plane.initPlane(mVertex[0]->mPoint,mVertex[1]->mPoint,mVertex[2]->mPoint));
		VertexIterator iter = mOutsideSet->mVertexList.begin();
		VertexIterator iterVertex;
		SrReal maxDist = 0 , dist;
		for( ; iter!=mOutsideSet->mVertexList.end();iter++ )
		{
			dist = plane.mNormal.dot((*iter)->mPoint) + plane.mD;
			if(LESS(maxDist, dist))
			{
				maxDist = dist;
				iterVertex = iter;
			}
		}
		return iterVertex;
	}
public:
	cOutsideSet*	mOutsideSet;			//The outside point set.
	cVertex*		mVertex[3];				//The point information of this facet.
	cFacet*			mNeighbor[3];			//Indicate the neighbor facets of this one.
	unsigned char	mVisitFlag;				//Indicate the flag in the process of determining the visible face set.
	FacetIterator*	mIterator;				//Indicate the location in the pending face list.
};

class cEdge
{
public:
	cEdge()
	{
		mPoint[0] = NULL;
		mPoint[1] = NULL;
		mNeighbor[0] = NULL;
		mNeighbor[1] = NULL;
	}
	void setEndpoint(cVertex* p0, cVertex* p1)
	{
		mPoint[0] = p0;
		mPoint[1] = p1;
	}
	void setNeighbor( cFacet* f0, cFacet* f1)
	{
		mNeighbor[0] = f0;
		mNeighbor[1] = f1;
	}
public:
	cVertex*		mPoint[2];				//The point information of this edge.
	cFacet*			mNeighbor[2];			//The neighbor facets of this edge.
};

#endif