#ifndef QUICK_HULL_H_
#define QUICK_HULL_H_
#include "DataStructure.h"

typedef struct _tFacet
{
	int			mFInx[3];
	int			mVInx[3];
}tFacet;

typedef struct
{
	SrPoint3D*	mVertes;
	tFacet*		mFacet;
	int			mNumVertes;
	int			mNumFacet;
}tHull;


class QuickHull
{
public:
	bool quickHull(SrPoint3D* points, int numPoint, tHull* resultHull);

private:
	bool collinear(const cPoint&p0, const cPoint& p1,const cPoint& p2);
	bool coplanar(const cPoint&p0, const cPoint& p1,const cPoint& p2,const cPoint& p3);
	void deallocate(FacetList& ftList);
	void findVisibleFacet(cVertex* furPoint,cFacet* f,FacetList& visibleSet,BoundaryEdgeMap& boudaryMap);
	void constructNewFacets(cVertex* point,FacetList& newFacetList,BoundaryEdgeMap& boundary);
	void determineOutsideSet(FacetList& facetList, VertexList& allVertex);
	void updateFacetPendList(FacetList& facetPendList , FacetList& newFacetList,cFacet*& head);
	void partitionOutsideSet(FacetList& facetPendList,FacetList& newFacetList,VertexList& allVertex,cFacet*& head);
	void gatherOutsideSet(FacetList& facetPendList, FacetList& visFacetList,VertexList& visOutsideSet);
	void quickHullScan(FacetList& facetPendList,cFacet*& head);
	bool initTetrahedron(VertexList& vertexes,FacetList& tetrahedron);
	void recursiveExport(cFacet* ,std::map<cVertex*,int>&,std::map<cFacet*,int>&,int&,int&,std::list<cFacet*>&);
	void exportHull(cFacet* head, tHull* hull);
};


#endif