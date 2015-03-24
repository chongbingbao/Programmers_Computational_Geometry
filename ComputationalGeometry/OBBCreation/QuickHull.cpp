#include "QuickHull.h"


bool QuickHull::quickHull(SrPoint3D* points, int numPoint, tHull* resultHull)
{
	// If the first and last point are equal the collinearity test some lines below will always be true.
	int i , sizePoint;
	for( i=numPoint-1 ; i>0 ; i-- )
		if( points[i].x!=points[0].x ||
			points[i].y!=points[0].y ||
			points[i].z!=points[0].z )
			break;

	sizePoint = i+1;
	if( sizePoint <= 3 )
		return false;

	//Copy all the vertexes into the vertex list.
	VertexList vertexList;
	cVertex* newVertex;
	VertexIterator vertexIter ;
	cVertex** buffer = new cVertex*[sizePoint];
	for( i=0 ; i<sizePoint ; i++ )
	{
		newVertex	 =  new cVertex;
		newVertex->mPoint.x	 = points[i].x;
		newVertex->mPoint.y	 = points[i].y;
		newVertex->mPoint.z	 = points[i].z;
		vertexList.push_back(newVertex);
		buffer[i] = newVertex;
	}
	cFacet* head;
	FacetList facetPendList , newFacetList;
	//Initialize the first tetrahedron.
	if( !initTetrahedron(vertexList,newFacetList) )
	{//If it fails, free the storage allocated to the vertex structure.
		for( i=0 ; i<sizePoint ; i++ )
			delete buffer[i];
		delete []buffer;
		return false;
	}

	partitionOutsideSet(facetPendList,newFacetList,vertexList,head);

	// If there exist no vertexes outside the hull, the tetrahedron is the hull.Or else, go into the if-exp.
	if( !facetPendList.empty() )
	{
		quickHullScan(facetPendList,head);
	}

	//Export the facets and vertexes to the tHull structure.
	//Free the storage of the facets.
	exportHull(head,resultHull);

	//Free the storage of the vertexes.
	for( i=0 ; i<sizePoint ; i++ )
		delete buffer[i];
	delete []buffer;

	return true;
}

bool QuickHull::collinear(const cPoint&p0, const cPoint& p1,const cPoint& p2)
{
	cVector normal = (p1 - p0).cross(p2 - p0);
	if( EQUAL(normal.magnitudeSquared(),0) )
		return true;
	return false;
}

bool QuickHull::coplanar(const cPoint&p0, const cPoint& p1,const cPoint& p2,const cPoint& p3)
{
	SrVector3 normal = (p1 - p0).cross(p2 - p0);
	if( EQUAL(normal.dot(p3 - p0),0) )
		return true;
	return false;
}

void QuickHull::deallocate(FacetList& ftList)
{
	FacetIterator fcIter;
	for( fcIter = ftList.begin(); fcIter!=ftList.end(); fcIter++ )
	{
		cFacet* vlue = *fcIter;
		delete vlue;
	}
	ftList.clear();
}

void QuickHull::findVisibleFacet(cVertex* furPoint,cFacet* f,FacetList& visibleSet,BoundaryEdgeMap& boudaryMap)
{
	f->mVisitFlag = FACET_VISITED;
	visibleSet.push_back(f);
	FacetIterator facetIter = visibleSet.begin();
	cEdge* boundaryEdge = NULL;
	int i ;
	cPlane plane;

	for( ; facetIter!=visibleSet.end(); facetIter++ )
	{
		for( i=0 ; i<3 ; i++ )
		{
			cFacet* neighbor = (*facetIter)->mNeighbor[i];
			if( neighbor->mVisitFlag==FACET_NULL )
			{
				neighbor->mVisitFlag = FACET_VISITED;
				ASSERT(plane.initPlane(neighbor->mVertex[0]->mPoint,neighbor->mVertex[1]->mPoint,neighbor->mVertex[2]->mPoint));
				if( plane.isOnPositiveSide(furPoint->mPoint) )
				{

					visibleSet.push_back(neighbor);
				}
				else
				{
					neighbor->mVisitFlag = FACET_BORDER;
					boundaryEdge = new cEdge;
					boundaryEdge->setNeighbor(*facetIter,neighbor);
					boundaryEdge->setEndpoint((*facetIter)->mVertex[i],(*facetIter)->mVertex[(i+1)%3]);
					boudaryMap.insert(std::make_pair((*facetIter)->mVertex[i],boundaryEdge));

				}
			}
			else if( neighbor->mVisitFlag==FACET_BORDER )
			{
				boundaryEdge = new cEdge();
				boundaryEdge->setNeighbor(*facetIter,neighbor);
				boundaryEdge->setEndpoint((*facetIter)->mVertex[i],(*facetIter)->mVertex[(i+1)%3]);
				boudaryMap.insert(std::make_pair((*facetIter)->mVertex[i],boundaryEdge));
			}
		}
	}
}

void QuickHull::constructNewFacets(cVertex* point,FacetList& newFacetList,BoundaryEdgeMap& boundary)
{
	ASSERT(boundary.size()>=3);
	BoundaryEdgeIterator current = boundary.begin();

	//The boundary edges are closed.
	current = boundary.begin();

	cEdge* edge = current->second;
	int i;
	while (!boundary.empty())
	{
		edge = current->second;
		cFacet* facet = new cFacet();
		//Clear the mVisitFlag. Because it's set FACET_BORDER in findVisibleFacet().
		edge->mNeighbor[1]->mVisitFlag = FACET_NULL;
		//Update neighbor facet of the invisible facet , at least one edge of which belong to the boundary.
		for( i=0 ; i<3 ; i++ )
		{
			if( edge->mNeighbor[1]->mNeighbor[i] == edge->mNeighbor[0] )
			{
				ASSERT( edge->mNeighbor[1]->mNeighbor[i]->mVisitFlag == FACET_VISITED);
				edge->mNeighbor[1]->mNeighbor[i] = facet;
				break;
			}
		}
		facet->mVertex[0]	 = point;
		facet->mVertex[1]	 = edge->mPoint[0];
		facet->mVertex[2]	 = edge->mPoint[1];
		facet->mNeighbor[1] = edge->mNeighbor[1];
		newFacetList.push_back(facet);
		boundary.erase(current);
		current = boundary.find(edge->mPoint[1]);
		delete edge;

	}
	//Update the neighbor facets of new facets.
	FacetIterator curFacet = newFacetList.begin();
	FacetIterator lastFacet = newFacetList.end();
	lastFacet --;
	for( ; curFacet!=newFacetList.end() ; lastFacet = curFacet, curFacet ++)
	{
		(*curFacet)->mNeighbor[0] = *lastFacet;
		(*lastFacet)->mNeighbor[2]= *curFacet;
	}
}

void QuickHull::determineOutsideSet(FacetList& facetList, VertexList& allVertex)
{
	FacetIterator facetIter;
	VertexIterator vertexIter,tempVertexIter;
	cPlane plane;
	for( facetIter = facetList.begin(); facetIter != facetList.end(); facetIter ++ )
	{
		ASSERT(plane.initPlane((*facetIter)->mVertex[0]->mPoint,(*facetIter)->mVertex[1]->mPoint,(*facetIter)->mVertex[2]->mPoint));
		for( vertexIter = allVertex.begin() ; vertexIter!=allVertex.end() ; )
		{
			if( plane.isOnPositiveSide((*vertexIter)->mPoint) )
			{//Update the outside vertex set of every new facet.
				tempVertexIter = vertexIter;
				tempVertexIter ++;
				if( !(*facetIter)->mOutsideSet )
				{
					(*facetIter)->mOutsideSet = new cOutsideSet();
				}
				(*facetIter)->mOutsideSet->mVertexList.splice((*facetIter)->mOutsideSet->mVertexList.end(),allVertex,vertexIter);
				vertexIter = tempVertexIter;
			}
			else
			{
				vertexIter ++;
			}
		}
	}
}

void QuickHull::updateFacetPendList(FacetList& facetPendList , FacetList& newFacetList,cFacet*& head)
{
	//If there exist new facets with nonempty outside vertex set, push them back into the pending facet list.
	FacetIterator facetIter;
	FacetIterator tempFacetIter;
	for( facetIter = newFacetList.begin(); facetIter != newFacetList.end();  )
	{
		if( (*facetIter)->mOutsideSet )
		{
			tempFacetIter = facetIter;
			tempFacetIter ++;
			facetPendList.splice(facetPendList.end(),newFacetList,facetIter);
			facetIter = facetPendList.end();
			facetIter --;
			if( !(*facetIter)->mIterator )
			{
				(*facetIter)->mIterator = new FacetIterator;
			}
			*(*facetIter)->mIterator = facetIter;
			facetIter = tempFacetIter;
		}
		else
		{
			head = (*facetIter);
			facetIter ++;
		}
	}
}

void QuickHull::partitionOutsideSet(FacetList& facetPendList,FacetList& newFacetList,VertexList& allVertex,cFacet*& head)
{
	// For each facet, look at each unassigned point and decide if it belongs to the outside set of this facet.
	determineOutsideSet(newFacetList,allVertex);
	// Add all the facets with non-empty outside sets to the set of facets for further consideration
	updateFacetPendList(facetPendList,newFacetList,head);
}

void QuickHull::gatherOutsideSet(FacetList& facetPendList, FacetList& visFacetList,VertexList& visOutsideSet)
{
	FacetIterator facetIter;
	FacetIterator tempFacetIter;
	for( facetIter = visFacetList.begin(); facetIter!=visFacetList.end(); )
	{
		//Copy the outside set of the visible facet into the visOutsideSet.
		cOutsideSet*& outsideSet = (*facetIter)->mOutsideSet;

		tempFacetIter = facetIter;
		tempFacetIter ++;
		if( outsideSet )
		{
			visOutsideSet.splice(visOutsideSet.end(),outsideSet->mVertexList,outsideSet->mVertexList.begin(),outsideSet->mVertexList.end());
		}
		//If some facets in the visible set exist in the pending list, remove them.
		if( (*facetIter)->mIterator /*&& *(*facetIter)->mIterator!=facetPendList.end()*/ )
		{
			facetPendList.erase(*(*facetIter)->mIterator);
		}
		//The visible triangles are unuseful.So deallocate them.
		facetIter = tempFacetIter;
	}
}

void QuickHull::quickHullScan(FacetList& facetPendList,cFacet*& head)
{
	FacetList		visFacetList;
	VertexList		visOutsideSet;
	FacetList		newFaceList;
	FacetIterator	facetIter;
	BoundaryEdgeMap boundary; 
	cPlane plane;

	while( !facetPendList.empty() )
	{
		ASSERT(visOutsideSet.empty());
		ASSERT(visFacetList.empty());
		ASSERT(boundary.empty());
		ASSERT(newFaceList.empty());

		FacetIterator f = facetPendList.begin();
		cFacet* facet = *f;
		//There must be at least one vertex.
		VertexIterator furVertexIter = facet->furthestVertex( );
		cVertex* furVertex = *furVertexIter;
		facet->mOutsideSet->mVertexList.erase(furVertexIter);

		//Find the visible facet set by the furthest vertex .
		findVisibleFacet(furVertex,facet,visFacetList,boundary);

		ASSERT(!visFacetList.empty());
		ASSERT(!boundary.empty());

		gatherOutsideSet(facetPendList,visFacetList,visOutsideSet);

		constructNewFacets(furVertex,newFaceList,boundary);
		ASSERT(!newFaceList.empty());

		partitionOutsideSet(facetPendList,newFaceList,visOutsideSet,head);

		//Free the storage of the visible facet set.
		deallocate(visFacetList);
		visOutsideSet.clear();
		newFaceList.clear();
	}
}

bool QuickHull::initTetrahedron(VertexList& vertexes,FacetList& tetrahedron)
{
	//Check weather or not all the vertexes are collinear.
	VertexIterator ptIndex1 = vertexes.begin();
	VertexIterator ptIndex2 = ptIndex1;
	VertexIterator ptIndex3 = vertexes.end();
	ptIndex2 ++;
	ptIndex3 --;
	while( ptIndex2 != ptIndex3 && collinear((*ptIndex1)->mPoint,(*ptIndex2)->mPoint,(*ptIndex3)->mPoint))
		ptIndex2 ++;
	//All the vertexes are collinear.
	if( ptIndex2 == ptIndex3 )
		return false;

	// Find the vertexes that have minimum value, maximum value in the x-dimension and minimum value
	// in the y-dimension.
	VertexIterator minx = vertexes.begin(), maxx = vertexes.begin(), miny = vertexes.begin();
	VertexIterator vertexIter = vertexes.begin();
	for( ; vertexIter!=vertexes.end(); vertexIter ++ )
	{
		if( (*vertexIter)->mPoint.x < (*minx)->mPoint.x )
			minx = vertexIter;
		else if( (*vertexIter)->mPoint.x > (*maxx)->mPoint.x )
			maxx = vertexIter;
		else if( (*vertexIter)->mPoint.y < (*miny)->mPoint.y )
			miny = vertexIter;
	}
	//If the three maximum and maximum vertexes found above aren't collinear, initialize the first tetrahedron by using them.
	if( !collinear((*minx)->mPoint,(*maxx)->mPoint,(*miny)->mPoint) ) 
	{
		ptIndex1 = minx;
		ptIndex2 = maxx;
		ptIndex3 = miny;
	}
	cPlane plane;
	ASSERT(plane.initPlane((*ptIndex1)->mPoint,(*ptIndex2)->mPoint,(*ptIndex3)->mPoint));

	//Find the vertexes that have minimum and maximum distance from the plane.
	SrReal minDist = plane.distance((*vertexes.begin())->mPoint);
	SrReal maxDist = minDist;
	vertexIter = vertexes.begin();
	VertexIterator  minPtIndex = vertexIter, maxPtIndex = vertexIter;
	vertexIter++;
	for(  ; vertexIter!=vertexes.end() ; vertexIter++ )
	{
		SrReal dist = plane.distance((*vertexIter)->mPoint);
		if( dist<minDist )
		{
			minDist = dist;
			minPtIndex = vertexIter;
		}
		if( dist>maxDist )
		{
			maxDist = dist;
			maxPtIndex = vertexIter;
		}
	}
	VertexIterator extIndex = maxPtIndex;
	if( coplanar((*ptIndex1)->mPoint,(*ptIndex2)->mPoint,(*ptIndex3)->mPoint,(*extIndex)->mPoint) ) 
	{
		//swap initP0 and  initP2. It's important for the direction of normal of the constructed plane.
		cVertex tmp		= *(*ptIndex1);
		*(*ptIndex1)	= *(*ptIndex3);
		*(*ptIndex3)	= tmp;
		extIndex = minPtIndex;
		if( coplanar((*ptIndex1)->mPoint,(*ptIndex2)->mPoint,(*ptIndex3)->mPoint,(*extIndex)->mPoint) ) 
		{// All of the vertexes are on the same plane.
			return false;
		}
	}

	//Initialize the first tetrahedron.
	cFacet* f0 = new cFacet();
	cFacet* f1 = new cFacet();
	cFacet* f2 = new cFacet();
	cFacet* f3 = new cFacet();

	f0->initFace(*ptIndex1,*ptIndex3,*ptIndex2);
	f1->initFace(*ptIndex1,*ptIndex2,*extIndex);
	f2->initFace(*ptIndex1,*extIndex,*ptIndex3);
	f3->initFace(*ptIndex2,*ptIndex3,*extIndex);

	f0->setNeighbors(f2,f3,f1);
	f1->setNeighbors(f0,f3,f2);
	f2->setNeighbors(f1,f3,f0);
	f3->setNeighbors(f0,f2,f1);

	vertexes.erase(ptIndex1);
	vertexes.erase(ptIndex2);
	vertexes.erase(ptIndex3);
	vertexes.erase(extIndex);

	tetrahedron.push_back(f0);
	tetrahedron.push_back(f1);
	tetrahedron.push_back(f2);
	tetrahedron.push_back(f3);

	return true;
}

void QuickHull::recursiveExport(cFacet* head,std::map<cVertex*,int>& vMap,std::map<cFacet*,int>& fMap, int& vId, int& fId,std::list<cFacet*>& fList)
{
	head->mVisitFlag = FACET_VISITED;
	fList.push_back(head);
	fMap.insert(std::make_pair(head,fId++));
	int i;
	for( i=0 ; i<3 ; i++ )
	{
		if( vMap.find(head->mVertex[i])==vMap.end() ) 
		{
			vMap.insert(std::make_pair(head->mVertex[i] , vId++));
		}
	}
	for( i=0 ; i<3 ; i++ )
	{
		if( head->mNeighbor[i]->mVisitFlag != FACET_VISITED )
		{
			recursiveExport(head->mNeighbor[i],vMap,fMap,vId,fId,fList);
		}
	}
}

void QuickHull::exportHull(cFacet* head, tHull* hull)
{
	std::map<cVertex*,int> vMap;
	std::map<cFacet*,int>  fMap;
	std::list<cFacet*> fList;
	int vId = 0, fId = 0;

	recursiveExport(head,vMap,fMap,vId,fId,fList);

	hull->mNumVertes = vMap.size();
	hull->mVertes	 = new SrPoint3D[hull->mNumVertes];

	std::map<cVertex*,int>::iterator mapIter;
	for( mapIter = vMap.begin() ; mapIter!=vMap.end() ; mapIter ++ )
	{
		ASSERT(mapIter->second<hull->mNumVertes);
		hull->mVertes[ mapIter->second ].x = mapIter->first->mPoint.x;
		hull->mVertes[ mapIter->second ].y = mapIter->first->mPoint.y;
		hull->mVertes[ mapIter->second ].z = mapIter->first->mPoint.z;
	}

	hull->mNumFacet = fList.size();
	hull->mFacet = new tFacet[ hull->mNumFacet ];

	std::list<cFacet*>::iterator fIter;
	int v0, v1, v2, fIndx = 0;
	cFacet* tempFacet;
	for( fIter = fList.begin() ; fIter!=fList.end() ;  )
	{
		v0 = vMap[(*fIter)->mVertex[0]];
		v1 = vMap[(*fIter)->mVertex[1]];
		v2 = vMap[(*fIter)->mVertex[2]];
		ASSERT(v0<hull->mNumVertes);
		ASSERT(v1<hull->mNumVertes);
		ASSERT(v2<hull->mNumVertes);

		hull->mFacet[fIndx].mVInx[0] = v0;
		hull->mFacet[fIndx].mVInx[1] = v1;
		hull->mFacet[fIndx].mVInx[2] = v2;

		hull->mFacet[fIndx].mFInx[0] = fMap[(*fIter)->mNeighbor[0]];
		hull->mFacet[fIndx].mFInx[1] = fMap[(*fIter)->mNeighbor[1]];
		hull->mFacet[fIndx].mFInx[2] = fMap[(*fIter)->mNeighbor[2]];

		fIndx ++;
		tempFacet = *fIter;
		fIter ++;
		delete tempFacet;
	}
}
