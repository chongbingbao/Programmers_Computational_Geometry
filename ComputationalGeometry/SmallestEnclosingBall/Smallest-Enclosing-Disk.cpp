/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/15
****************************************************************************/
#include "../include/SrGeometricTools.h"
#include "../include/SrDataType.h"

#include <list>
#include <math.h>
#include <iostream>
#include <algorithm>
#include "Miniball.hpp"

/**
\brief 3D sphere class.

This is a 3D sphere class with public data members.
It includes center and radius.
*/
class SrSphere3D
{
public:
	/**
	\brief Default constructor. Set center to (0,0) and radius to 0.
	*/
	SrSphere3D()
	{
		mCenter.set(0,0,0);
		mRadius = -1.0;
	}
	/**
	\brief The circle is initialized by center and radius.
	*/
	SrSphere3D(const SrPoint3D& ct ,SrReal rd)
	{
		mCenter = ct;
		mRadius = rd;
	}
public:
	SrPoint3D	mCenter;
	SrReal		mRadius;
};

typedef std::list<SrPoint3D>			VertexList;
typedef std::list<SrPoint3D>::iterator	VertexIterator;



void createSphere(SrSphere3D& sphere)
{
	sphere.mCenter.set(0,0,0);
	sphere.mRadius = 0.0;
}

void createSphere(const SrPoint3D& p1,SrSphere3D& sphere)
{
	sphere.mCenter = p1;
	sphere.mRadius = 0.0;
}

void createSphere(const SrPoint3D& p0,const SrPoint3D& p1,SrSphere3D& sphere)
{
	sphere.mCenter = (p0 + p1) / 2;
	sphere.mRadius = ((p0 - p1) / 2.0).magnitude();
}

void createSphere(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D& p2,SrSphere3D& sphere)
{
	SrVector3 e0 = p1 - p0 , e1 = p2 - p0;
	SrReal	a = e0.dot(e0), b = e0.dot(e1), c = e1.dot(e1);
	SrReal	d = a*c - b*b;
	if( EQUAL(d,0) )
		return;
	SrReal s = (a - b)*c/(2*d), t = (c - b)*a/(2*d);
	sphere.mCenter = p0 + s*e0 + t*e1;
	sphere.mRadius = (p0 - sphere.mCenter).magnitude();
}

void createSphere(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D& p2,const SrPoint3D& p3,SrSphere3D& sphere)
{
	SrVector3 v1 = p1 - p0 , v2 = p2 - p0 , v3 = p3 - p0;
	SrReal V = v1.dot(v2.cross(v3));
	//Check that the three points are not on the same plane.
	if( EQUAL(V,0) )
		return;
	V*=2.0;
	SrReal L1 = v1.magnitudeSquared() , L2 = v2.magnitudeSquared() , L3 = v3.magnitudeSquared();
	sphere.mCenter.x = (p0.x + ( (v2.y*v3.z - v3.y*v2.z)*L1 - (v1.y*v3.z - v3.y*v1.z)*L2 + (v1.y*v2.z - v2.y*v1.z)*L3 ) / V);
	sphere.mCenter.y = (p0.y + (-(v2.x*v3.z - v3.x*v2.z)*L1 + (v1.x*v3.z - v3.x*v1.z)*L2 - (v1.x*v2.z - v2.x*v1.z)*L3 ) / V);
	sphere.mCenter.z = (p0.z + ( (v2.x*v3.y - v3.x*v2.y)*L1 - (v1.x*v3.y - v3.x*v1.y)*L2 + (v1.x*v2.y - v2.x*v1.y)*L3 ) / V);
	sphere.mRadius = (sphere.mCenter - p0).magnitude();
}

void createSphere(SrPoint3D* support, int numPoint ,SrSphere3D& sphere)
{
	switch(numPoint)
	{
	case 0:
		createSphere(sphere);
		break;
	case 1:
		createSphere(support[0],sphere);
		break;
	case 2:
		createSphere(support[0],support[1],sphere);
		break;
	case 3:
		createSphere(support[0],support[1],support[2],sphere);
		break;
	case 4:
		createSphere(support[0],support[1],support[2],support[3],sphere);
		break;
	}
}


bool isOut(const SrPoint3D& point,const SrSphere3D& sphere)
{
	if( GREATER((sphere.mCenter - point).magnitudeSquared(), sphere.mRadius*sphere.mRadius) )
		return true;
	return false;
}

void smallestEnclosingSphere(SrPoint3D*	sp,int nsp,VertexList& vertexList,const VertexIterator& end,SrSphere3D& sphere)
{
	createSphere(sp, nsp,sphere);
	if( nsp==4 )
		return;
	VertexIterator iterator = vertexList.begin();
	for( ; end != iterator ; )
	{
		const SrPoint3D& p = *iterator;
		if( isOut(p,sphere) )
		{
			sp[ nsp ] = p;
			smallestEnclosingSphere( sp, nsp + 1 , vertexList , iterator , sphere);
			vertexList.splice(vertexList.begin(),vertexList,iterator++ );
		}
		else
		{
			iterator++;
		}
	}
}



const SrSphere3D createBoundingSphere(const SrPoint3D* point, int numPoint)
{
	ASSERT(numPoint>0);
	SrPoint3D supportPoint[4];
	VertexList vertexList;
	SrPoint3D* buffer = new SrPoint3D[numPoint];
	int i;
	for( i=0 ; i<numPoint ; i++ )
		buffer[i] = point[i];
	std::random_shuffle(buffer,buffer + numPoint);
	std::copy(buffer,buffer + numPoint, std::back_inserter(vertexList));

	delete []buffer;
	SrSphere3D sphere;
	smallestEnclosingSphere(supportPoint,0,vertexList,vertexList.end(),sphere);
	return sphere;
}

const SrSphere3D test_SmallestEnclosingSphere_Gaertner(const SrPoint3D* point, int n)
{
	typedef double mytype;            // coordinate type
	int             d = 3;            // dimension

	mytype** ap = new mytype*[n];
	for (int i=0; i<n; ++i) 
	{
		mytype* p = new mytype[d];
		p[0] = point[i].x;
		p[1] = point[i].y;
		p[2] = point[i].z;
		ap[i]=p;
	}

	// define the types of iterators through the points and their coordinates
	// ----------------------------------------------------------------------
	typedef mytype* const* PointIterator; 
	typedef const mytype* CoordIterator;

	// create an instance of Miniball
	// ------------------------------
	typedef Miniball::Miniball <Miniball::CoordAccessor<PointIterator, CoordIterator> >		MB;
	MB mb (d, ap, ap+n);
	SrSphere3D sphere;
	const mytype* center = mb.center(); 
	sphere.mCenter.x = *center;
	sphere.mCenter.y = *(++center);
	sphere.mCenter.z = *(++center);
	sphere.mRadius = sqrt(mb.squared_radius());

	// clean up
	for (int i=0; i<n; ++i)
		delete[] ap[i];
	delete[] ap;

	return sphere;
}

const SrSphere3D  test_SmallestEnclosingSphere_TwinklingStar(const SrPoint3D* point, int n)
{

	return createBoundingSphere(point,n);
}

void test_SmallestEnclosingSphere()
{
	int	n = 100000 , i;
	SrPoint3D * point = new SrVector3[n];
	for ( i=0; i<n; ++i) 
	{
		point[i].x = rand();
		point[i].y = rand();
		point[i].z = rand();
	}


	printf("******************************************************\n");
	printf("gaertner's smallest enclosing sphere\n");
	double mTime = clock();
	SrSphere3D s1 = test_SmallestEnclosingSphere_Gaertner(point,n);
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("Ceneter:(%f,%f,%f)	Radius:%f	Time:	%f\n",s1.mCenter.x,s1.mCenter.y,s1.mCenter.z,s1.mRadius,mTime);

	printf("******************************************************\n");
	printf("My smallest enclosing sphere\n");
	mTime = clock();
	SrSphere3D s2 = test_SmallestEnclosingSphere_TwinklingStar(point,n);
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("Ceneter:(%f,%f,%f)	Radius:%f	Time:	%f\n",s2.mCenter.x,s2.mCenter.y,s2.mCenter.z,s2.mRadius,mTime);

	ASSERT(EQUAL(s1.mRadius,s2.mRadius));
	ASSERT(EQUAL(s1.mCenter.x,s2.mCenter.x) && EQUAL(s1.mCenter.y,s2.mCenter.y) && EQUAL(s1.mCenter.z,s2.mCenter.z) );

	delete []point;
}

int main( )
{
	test_SmallestEnclosingSphere();
	return 0;
}