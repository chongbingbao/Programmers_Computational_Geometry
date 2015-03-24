/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/19
****************************************************************************/
/************************************************************************		
\description	两种方法，判断三角形与有向包围盒是否重叠
****************************************************************************/
#include "../include/SrGeometricTools.h"
#include "../include/SrDataType.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

/**
\brief 3D oriented bounding box (OBB) class.
*/
class SrOBBox3D
{
public:
	SrOBBox3D()
	{
		mCenter = SrVector3D(0,0,0);
		mAxis[0] = mAxis[1] = mAxis[2] = SrVector3D(0,0,0);
		mHalfLength[0]	= mHalfLength[1] = mHalfLength[2] = 0;
	}
	SrOBBox3D(const SrPoint3D& c,const SrVector3D* pAxis,const SrReal* pHalfLength)
	{
		mCenter = c;
		mAxis[0] = pAxis[0];
		mAxis[1] = pAxis[1];
		mAxis[2] = pAxis[2];

		mHalfLength[0] = pHalfLength[0];
		mHalfLength[1] = pHalfLength[1];
		mHalfLength[2] = pHalfLength[2];
	}
	/**
	\brief  The OBB is valid if one of the three axis is unit length, 
			the length of them is greater than 0, and the axis should 
			be perpendicular to each other.
	*/
	bool	isValid() const
	{
		int i;
		if( UNEQUAL(mAxis[0].dot(mAxis[1]),0) ||
			UNEQUAL(mAxis[0].dot(mAxis[2]),0)||
			UNEQUAL(mAxis[1].dot(mAxis[2]),0))
			return false;
		for( i=0 ; i<3 ; i++ )
		{
			if( LEQUAL(mHalfLength[i],0) )
				return false;
			if( UNEQUAL(mAxis[i].magnitudeSquared(),1.0) )
				return false;
		}
		return true;
	}

public:
	SrPoint3D	mCenter;
	SrVector3D	mAxis[3];
	SrReal		mHalfLength[3];
};


/**
\brief 3D triangle class.
This is a 3D triangle class with public data members.
*/

class SrTriangle3D
{
public:
	/**
	\brief Default constructor, endpoints is set to (0,0).
	*/
	SrTriangle3D()
	{
		mPoint[0] = mPoint[1] = mPoint[1] = SrVector3D(0,0,0);
	}
	/**
	\brief The line is initialized by three points.
	*/
	SrTriangle3D(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D& p2)
	{
		mPoint[0] = p0;
		mPoint[1] = p1;
		mPoint[2] = p2;
	}
	bool	isValid() const
	{
		SrVector3D norm = (mPoint[1] - mPoint[0]).cross(mPoint[2] - mPoint[0]);
		if( EQUAL(norm.x,0) && EQUAL(norm.y,0) && EQUAL(norm.z,0) )
			return false;
		return true;
	}
	/**
	\brief  Judge whether or not the segment hits the triangle.
	\return true				if intersecting
			false				if not.
	*/
	bool segmentHitTest(const SrPoint3D& point1, const SrPoint3D& point2)const
	{
		SrVector3D ret;
		SrVector3D direction = point2 - point1;
		if(!linearIntersectTriangle(point1,direction,ret))
			return false;
		//check t. 0<= t <=1
		if( LESS(ret.z,0) || GREATER(ret.z,1) )
			return false;
		return true;
	}
public:
	SrPoint3D	mPoint[3];

private:
	/*
	--------------------------------------------------------------------------
	Devised by Moller and Trumbore,1997. <Fast,minimum storage ray-triangle 
	intersection>
	Any point in a triangle can be defined in terms of its position relative
	to the triangle’s vertices:
			Qu,v = (1-u-v)V0 + uV1 + vV2 ,0<=u<=1,0<=v<=1,0<=u+v<=1
	For the linear component–triangle intersection,
			P + t*^d = Qu,v
	which can be expanded and applied Cramer’s rule to:
			|t|							| |P-V0  V1-V0  V2-V0| |
			|u| = 1/|-^d  V1-V0  V2-V0| | |-^d   P-V0   V2-V0| |
			|v|							| |-^d   V1-V0   P-V0| |
										

											| ((P-V0)×(V1-V0))·(V2-V0) |
				= 1/(^d×(V2-v0))·(V1-V0)	|    (^d×(V2-V0))·(P-V0)   |
											|    ((P-V0)×(V1-V0))·^d	 |
	--------------------------------------------------------------------------
	*/
	bool linearIntersectTriangle(const SrPoint3D& base,const SrVector3D& direction,SrVector3D& result)const
	{
		SrReal u,v,tmp;
		SrVector3D e1,e2,p,s,q;
		e1 = mPoint[1] - mPoint[0];
		e2 = mPoint[2] - mPoint[0];
		p = direction.cross(e2);
		tmp = p.dot(e1);
		//If the line is perpendicular to the normal of triangle.
		if( EQUAL(tmp,0) )
		{
			//The line is on the plane.
			p = e1.cross(e2);
			if( EQUAL(p.dot(base - mPoint[0]),0) )
				return true;
			//The line is parallel to the plane.
			return false;
		}
		s=base - mPoint[0];
		u = p.dot(s)/tmp;
		if( LESS(u,0) || GREATER(u,1) )
			return false;
		q = s.cross(e1);
		v = q.dot(direction)/tmp;
		if( LESS(v,0) || GREATER(v,1) || GREATER(u+v,1) )
			return false;

		result.x = u;
		result.y = v;
		result.z = e2.dot(q)/tmp;
		return true;
	}
};


void GetMinMax(SrReal t0, SrReal t1, SrReal t2,SrReal& minValue,SrReal& maxValue)
{
	minValue = maxValue = t0;
	if( LESS(t1,minValue) )		minValue = t1;
	if( GREATER(t1,maxValue))	maxValue = t1;
	if( LESS(t2,minValue) )		minValue = t2;
	if( GREATER(t2,maxValue) )	maxValue = t2;
}

void GetMinMax(SrReal t0, SrReal t1,SrReal& minValue,SrReal& maxValue)
{
	if( LESS(t0,t1) )
	{
		minValue = t0;
		maxValue = t1;
	}
	else
	{
		minValue = t1;
		maxValue = t0;
	}
}

bool IsSeparatingAxis(SrReal p0, SrReal p1,SrReal r)
{
	SrReal mn,mx;
	GetMinMax(p0,p1,mn,mx);
	if( LESS(mx,-r) || GREATER(mn,r) )
		return true;
	return false;
}

/*
	The expression "n[dot]v" is the dot product of the two vectors.
	The expression "n[cross]v" is the cross production of the two vectors.
	Let n = axis[cross](v(i+1)-v(i)), pi = n[dot]v(i).
	For edge v(1)-v(0), p0 == p1.
	For edge v(2)-v(1), p1 == p2.
	For edge v(0)-v(2), p2 == p0.
*/
bool TestAxisCrossEdge(const SrPoint3D* v,const SrVector3D& e,const SrOBBox3D& obb)
{
	SrReal fex , fey , fez , r;
	fex = fabs(e.x);
	fey = fabs(e.y);
	fez = fabs(e.z);
	SrReal p0 , p2;
	//e and Axis X
	//n = (1,0,0)[cross]e = (0, -e.z, e.y)
	p0 = -e.z*v[0].y + e.y*v[0].z;
	p2 = -e.z*v[2].y + e.y*v[2].z;
	r = fez*obb.mHalfLength[1] + fey*obb.mHalfLength[2];
	if( IsSeparatingAxis(p0,p2,r) )
		return false;
	//e and Axis Y
	//n = (0,1,0)[cross]e = (e.z,0,-e.x)
	p0 = e.z*v[0].x - e.x*v[0].z;
	p2 = e.z*v[2].x - e.x*v[2].z;
	r = fez*obb.mHalfLength[0] + fex*obb.mHalfLength[2];
	if( IsSeparatingAxis(p0,p2,r) )
		return false;
	//e and Axis Z
	//n = (0,0,1)[cross]e = (-e.y,e.x,0)
	p0 = -e.y*v[0].x + e.x*v[0].y;
	p2 = -e.y*v[2].x + e.x*v[2].y;
	r = fey*obb.mHalfLength[0] + fex*obb.mHalfLength[1];
	if( IsSeparatingAxis(p0,p2,r) )
		return false;
	return true;
}


/*
\brief	基于轴分离理论，判断三角形与包围盒是否重叠。
\return	true	重叠
		false	不重叠
*/
bool TriangleIntersectionOBB_SeparatingAxisMethod(const SrTriangle3D& triangle,const SrOBBox3D& obb)
{
	int i;
	SrVector3D v[3] ;
	SrPoint3D p;
	for( i=0 ; i<3 ; i++ )
	{ 
		p = triangle.mPoint[i] - obb.mCenter;
		v[i].x = obb.mAxis[0].dot(p);
		v[i].y = obb.mAxis[1].dot(p);
		v[i].z = obb.mAxis[2].dot(p);
	}
	SrReal mn,mx;
	//Test the 9 tests first
	//Edge e0
	const SrVector3D e0 = v[1] - v[0];
	SrPoint3D u[3];
	u[0] = v[0]; u[1] = v[1]; u[2] = v[2];
	if( !TestAxisCrossEdge(v,e0,obb) )
		return false;
	//Edge e1
	u[0] = v[1]; u[1] = v[2]; u[2] = v[0];
	const SrVector3D e1 = v[2] - v[1];
	if( !TestAxisCrossEdge(u,e1,obb) )
		return false;
	//Edge e2
	const SrVector3D e2 = v[2] - v[0];
	u[0] = v[2]; u[1] = v[0]; u[2] = v[1];
	if( !TestAxisCrossEdge(u,e2,obb) )
		return false;
	//Test overlap in the {x,y,z}-directions.
	//Find min, max of the triangle each direction, 
	//and test for overlap in that direction.
	//Axis X
	GetMinMax(v[0].x,v[1].x,v[2].x,mn,mx);
	if( GREATER(mn,obb.mHalfLength[0]) || LESS(mx,-obb.mHalfLength[0]) )
		return false;
	//Axis Y
	GetMinMax(v[0].y,v[1].y,v[2].y,mn,mx);
	if( GREATER(mn,obb.mHalfLength[1]) || LESS(mx,-obb.mHalfLength[1]) )
		return false;
	//Axis Z
	GetMinMax(v[0].z,v[1].z,v[2].z,mn,mx);
	if( GREATER(mn,obb.mHalfLength[2]) || LESS(mx,-obb.mHalfLength[2]) )
		return false;

	//Test if the box intersects the plane of the triangle.
	//Compute the plane equation of triangle: normal*x+d=0.
	SrVector3D normal = (v[1] - v[0]).cross(v[2] - v[0]);
	SrReal d = - normal.dot(v[0]);
	SrPoint3D minPoint,maxPoint;
	for( i=0 ; i<3 ; i++ )
	{
		if( GREATER(normal[i],0) )
		{
			minPoint[i] = -obb.mHalfLength[i];
			maxPoint[i] =  obb.mHalfLength[i];
		}
		else
		{
			minPoint[i] =  obb.mHalfLength[i];
			maxPoint[i] = -obb.mHalfLength[i];
		}
	}
	SrReal t;
	t = normal.dot(minPoint) + d;
	if( GREATER(t,0) )
		return false;
	t = normal.dot(maxPoint) + d;
	if( LESS(t , 0) )
		return false;

	return true;
}

/*
\brief	判断三角形的三个顶点中，是否存在至少一个点在AABB内。
\return	true	存在
		false	不存在
*/
bool TestVertexInsideAABB(const SrPoint3D* v,const SrPoint3D& minAABB,const SrPoint3D& maxAABB)
{
	int i;
	for( i=0 ; i<3 ; i++ )
	{
		if( GEQUAL(v[i].x,minAABB.x)&&LEQUAL(v[i].x,maxAABB.x)&&
			GEQUAL(v[i].y,minAABB.y)&&LEQUAL(v[i].y,maxAABB.y)&&
			GEQUAL(v[i].z,minAABB.z)&&LEQUAL(v[i].z,maxAABB.z))
			return true;
	}
	return false;
}

/*
\brief	判断线段与AABB是否相交。
\return	true	相交
		false	不相交
*/
bool TestSegmentIntersectionAABB(const SrPoint3D& p0,const SrPoint3D& p1, const SrOBBox3D& obb)
{
	SrPoint3D m = (p0 + p1) / 2.0;
	SrVector3D w = m - p0;
	SrReal X = fabs(w.x), Y = fabs(w.y) , Z = fabs(w.z);
	if( GREATER(fabs(m.x),X+obb.mHalfLength[0]) )
		return false;
	if( GREATER(fabs(m.y),Y+obb.mHalfLength[1]) )
		return false;
	if( GREATER(fabs(m.z),Z+obb.mHalfLength[2]) )
		return false;
	if( GREATER(fabs(m.y*w.z - m.z*w.y),obb.mHalfLength[1]*Z + obb.mHalfLength[2]*Y) )
		return false;
	if( GREATER(fabs(m.x*w.z - m.z*w.x),obb.mHalfLength[0]*Z + obb.mHalfLength[2]*X) )
		return false;
	if( GREATER(fabs(m.x*w.y - m.y*w.x),obb.mHalfLength[0]*Y + obb.mHalfLength[1]*X) )
		return false;
	return true;
}

/*
\brief	判断三角形与包围盒是否重叠，主要包括3个步骤
		（1）检测三角形的顶点是否在包围盒内；
		（2）检测三角形的边是否与长方体相交；
		（3）检测包围盒的四条对角线是否与三角形相交。
		参考：Douglas Voorhies. "Triangle-cube intersection." Graphics Gems III. Academic Press Professional, Inc., 1992.
\return	true	重叠
		false	不重叠
*/
bool TriangleIntersectionOBB_Voorhies(const SrTriangle3D& triangle,const SrOBBox3D& obb)
{
	int i;
	SrVector3D v[3];
	for( i=0 ; i<3 ; i++ )
	{
		v[i].x = obb.mAxis[0].dot(triangle.mPoint[i] - obb.mCenter);
		v[i].y = obb.mAxis[1].dot(triangle.mPoint[i] - obb.mCenter);
		v[i].z = obb.mAxis[2].dot(triangle.mPoint[i] - obb.mCenter);
	}

	SrPoint3D minAABB , maxAABB;
	minAABB.x = -obb.mHalfLength[0];
	minAABB.y = -obb.mHalfLength[1];
	minAABB.z = -obb.mHalfLength[2];

	maxAABB.x =  obb.mHalfLength[0];
	maxAABB.y =  obb.mHalfLength[1];
	maxAABB.z =  obb.mHalfLength[2];

	if( TestVertexInsideAABB(v,minAABB,maxAABB) )
		return true;

	if( TestSegmentIntersectionAABB(v[1],v[0],obb) )
		return true;
	if( TestSegmentIntersectionAABB(v[2],v[1],obb) )
		return true;
	if( TestSegmentIntersectionAABB(v[0],v[2],obb) )
		return true;

	SrTriangle3D tri;
	tri.mPoint[0] = v[0];
	tri.mPoint[1] = v[1];
	tri.mPoint[2] = v[2];

	SrPoint3D diagonal1 , diagonal2;
	diagonal1 = minAABB;diagonal2 = maxAABB;
	if( tri.segmentHitTest(diagonal1,diagonal2) )
		return true;

	diagonal1.y = maxAABB.y;diagonal2.y = minAABB.y;
	if( tri.segmentHitTest(diagonal1,diagonal2) )
		return true;

	diagonal1 = minAABB;diagonal2 = maxAABB;
	diagonal1.z = maxAABB.z;diagonal2.z = minAABB.z;
	if( tri.segmentHitTest(diagonal1,diagonal2) )
		return true;

	diagonal1.y = maxAABB.y;diagonal2.y = minAABB.y;
	if( tri.segmentHitTest(diagonal1,diagonal2) )
		return true;
	return false;
}

/*
\brief 随机生成三角形数据，测试两种三角形与OBB相交测试算法的正确性，并测试两种算法的效率。
*/
void Test_TriangleIntersectionOBB()
{
	int numTri = 100000;
	SrTriangle3D* triangle = new SrTriangle3D[numTri];
	int i , range = 2000;
	SrPoint3D p0 , p1, p2;
	for( i=0 ; i<numTri ; i++ )
	{
		do 
		{
			triangle[i].mPoint[0].x = rand() % range;
			triangle[i].mPoint[0].y = rand() % range;
			triangle[i].mPoint[0].z = rand() % range;

			triangle[i].mPoint[1].x = rand() % range;
			triangle[i].mPoint[1].y = rand() % range;
			triangle[i].mPoint[1].z = rand() % range;

			triangle[i].mPoint[2].x = rand() % range;
			triangle[i].mPoint[2].y = rand() % range;
			triangle[i].mPoint[2].z = rand() % range;
		} while (!triangle[i].isValid());
	}
	SrVector3D axis[3];
	axis[0] = SrVector3D(1,0,0);
	axis[1] = SrVector3D(0,1,0);
	axis[2] = SrVector3D(0,0,1);
	SrReal halfLen[3] = {250,250,250};
	SrPoint3D center = SrVector3D(750,750,750);
	SrOBBox3D obb(center,axis,halfLen);

	int status0 , status1;
	//测试算法的正确性
	for( i=0 ; i<numTri ; i++ )
	{
		status0 = TriangleIntersectionOBB_SeparatingAxisMethod(triangle[i],obb);
		status1 = TriangleIntersectionOBB_Voorhies(triangle[i],obb);
		if( status0!=status1 )
		{
			status0 = TriangleIntersectionOBB_SeparatingAxisMethod(triangle[i],obb);
			status1 = TriangleIntersectionOBB_Voorhies(triangle[i],obb);
		}
		ASSERT(status0 == status1);
	}

	double mTime ;
	printf("三角形与OBB相交检测:分离轴方法，时间开销(s)：");
	mTime = clock();
	for( i=0 ; i<numTri ; i++ )
	{
		TriangleIntersectionOBB_SeparatingAxisMethod(triangle[i],obb);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("%f\n",mTime);

	printf("三角形与OBB相交检测:Voorhies方法，时间开销(s)：");
	mTime = clock();
	for( i=0 ; i<numTri ; i++ )
	{
		TriangleIntersectionOBB_Voorhies(triangle[i],obb);
	}
	mTime = (clock() - mTime) / CLOCKS_PER_SEC;
	printf("%f\n",mTime);

	delete []triangle;
}

int main( )
{
	Test_TriangleIntersectionOBB();
	return 0;
}