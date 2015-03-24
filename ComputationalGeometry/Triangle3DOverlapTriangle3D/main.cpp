/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/04/20
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"

#include <time.h>
#include <stdio.h>

#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D
#define Point3D		SrPoint3D
#define Vector3D	SrVector3D


#define EPS			0.000001f
#define INSIDE		0x01
#define OUTSIDE		0x02
#define ON_EDGE		0x03

#define AXIS_X		0x01
#define AXIS_Y		0x02
#define AXIS_Z		0x04

/**
\brief 3D triangle class.
This is a 3D triangle class with public data members.
*/
class SrTriangle3D
{
public:
	SrTriangle3D()
	{
		mPoint[0].set(0,0,0);
		mPoint[1].set(0,0,0);
		mPoint[2].set(0,0,0);
	}
	SrTriangle3D(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D& p2)
	{
		mPoint[0] = p0;
		mPoint[1] = p1;
		mPoint[2] = p2;
	}
	/**
	\brief The triangle is valid if the three points are not on a line.
	*/
	bool isValid()
	{
		SrVector3D norm = (mPoint[1]-mPoint[0]).cross(mPoint[2]-mPoint[0]);
		if( EQUAL(norm.x,0)&&EQUAL(norm.y,0)&&EQUAL(norm.z,0) )
			return false;
		return true;
	}
	/**
	\brief  Judge whether the point is in the triangle or not.
	\return SR_POSITION_ON		 if the point is on the edge; 
			SR_POSITION_INSIDE  if inside the triangle and on the same plane; 
			SR_POSITION_OUTSIDE if outside the triangle and on the same plane.
			SR_POSITION_INVALID if not on the same plane.
	*/
	int		pointLocation(const SrPoint3D& p) const
	{
		//Judge whether the three points determine a plane or not.
		SrVector3D norm = (mPoint[1] - mPoint[0]).cross(mPoint[2] - mPoint[0]);
		if(EQUAL(norm.x,0) && EQUAL(norm.y,0) && EQUAL(norm.z,0))
			return SR_POSITION_INVALID;
		SrReal d = -norm.dot(mPoint[0]);
		if( EQUAL(norm.dot(p) + d, 0) )
			return SR_POSITION_INVALID;

		SrReal flag1 = norm.dot((mPoint[1]-mPoint[0]).cross(p-mPoint[0]));
		if( LESS(flag1,0) )
			return SR_POSITION_OUTSIDE;
		SrReal flag2 = norm.dot((mPoint[2]-mPoint[1]).cross(p-mPoint[1]));
		if( LESS(flag2,0) )
			return SR_POSITION_OUTSIDE;
		SrVector3D tmp = (mPoint[0]-mPoint[2]).cross(p-mPoint[2]);
		SrReal flag3 = norm.dot((mPoint[0]-mPoint[2]).cross(p-mPoint[2]));
		if( LESS(flag3,0) )
			return SR_POSITION_OUTSIDE;
		if( EQUAL(flag1,0) || EQUAL(flag2,0) || EQUAL(flag3,0) )
			return SR_POSITION_ON;
		return SR_POSITION_INSIDE;
	}
	/**
	\brief  Judge whether or not the segment hits the triangle.
	\return SR_OVERLAPPING		 if the segment is on the plane where the triangle lies;
			SR_PARALLEL			 if the segment is parallel to the triangle.
			SR_SEPARATING		 if the segment misses the triangle;
			SR_INTERSECTING		 if the segment intersects with the triangle and intersection point is not on the edge.
	*/
	int segmentHitTest(const SrPoint3D& p0, const SrPoint3D& p1)const
	{
		SrVector3D ret;
		SrVector3D direction = p1 - p0;
		int retFlag = linearIntersectTriangle(p0,direction,ret);
		if( retFlag!=SR_INTERSECTING )
			return retFlag;
		//check t. 0<= t <=1
		if( LESS(ret.z,0) || GREATER(ret.z,1) )
			return SR_DISJOINT;
		//check whether the intersection point is on the edge.
		return SR_INTERSECTING;
	}
public:
	SrPoint3D mPoint[3];
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
	int linearIntersectTriangle(const SrPoint3D& base,const SrVector3D& direction,SrVector3D& result)const
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
				return SR_OVERLAPPING;
			//The line is parallel to the plane.
			return SR_PARALLEL;
		}
		s=base - mPoint[0];
		u = p.dot(s)/tmp;
		if( LESS(u,0) || GREATER(u,1) )
			return SR_DISJOINT;
		q = s.cross(e1);
		v = q.dot(direction)/tmp;
		if( LESS(v,0) || GREATER(v,1) || GREATER(u+v,1) )
			return SR_DISJOINT;

		result.x = u;
		result.y = v;
		result.z = e2.dot(q)/tmp;
		return SR_INTERSECTING;
	}
};

#define  Triangle3D			SrTriangle3D


/*
*\brief  在二维空间上，判断两条线段是否相交.
*\return true		若相交.
		 false		若不相交.
*/
bool Seg2DIntersectSeg2D(const Point2D& p0, const Point2D& p1, const Point2D& q0, const Point2D& q1 )
{
	SrVector2D A = p1 - p0;
	SrVector2D B = q0 - q1;
	SrVector2D C = p0 - q0;
	SrReal a = B.y*C.x - B.x*C.y;
	SrReal b;
	SrReal denom = A.y*B.x - A.x*B.y;
	if( GREATER( denom,0 ) )
	{
		if( LESS(a,0) || GREATER(a,denom) )
			return false;
		b = A.x*C.y - A.y*C.x;
		if( LESS(b,0) || GREATER(b,denom) )
			return false;
		return true;
	}
	else if (LESS( denom,0 ))
	{
		if( GREATER(a,0) || LESS(a,denom) )
			return false;
		b = A.x*C.y - A.y*C.x;
		if( GREATER(b,0) || LESS(b,denom) )
			return false;
		return true;
	}
	else
	{
		if( !EQUAL(a,0) )
			return false;

		SrReal denom , t0, t1;
		if( GREATER(fabs(A.x),fabs(A.y)) )
		{
			t0 = q0.x - p0.x;
			t1 = q1.x - p0.x;
			denom = A.x;
		}
		else
		{
			t0 = q0.y - p0.y;
			t1 = q1.y - p0.y;
			denom = A.y;
		}
		if( LESS(denom,0) )
		{
			denom = -denom;
			t0    = -t0;
			t1	  = -t1;
		}
		SrReal tmp;
		if( t0>t1 )
		{
			tmp = t0;
			t0 = t1;
			t1 = tmp;
		}
		if( GREATER(t0,denom) || LESS(t1,0) )
			return false;
		else if( EQUAL(t1,0) || EQUAL(t0,1) )
			return true;

		return true;
	}
	return false;
}

/**
\brief  在二维空间上，判断一个点与三角形的关系
\return ON_EDGE		若点在三角形的边上; 
		INSIDE		若点在三角形内部; 
		OUTSIDE		若点在三角形外部.
*/
int		PointTriangleRelation(const Point2D* tri, const Point2D& p) 
{
	Real d0 = (tri[1] - tri[0]).cross(p - tri[0]);
	Real d1 = (tri[2] - tri[1]).cross(p - tri[1]);
	Real d2 = (tri[0] - tri[2]).cross(p - tri[2]);
	Real s0 = d0*d1, s1 = d1*d2;

	if( LESS(s0,0) || LESS(s1,0) )
		return OUTSIDE;
	else if( EQUAL(s0,0) || EQUAL(s1,0))
		return ON_EDGE;
	return INSIDE;
}
/**
\brief  在二维空间上，三角形与三角形的重叠检测
\return true		若两三角形重叠;
		false		否则.
*/
bool Tri2DOverlapTestTri2D(const Point2D* tri1, const Point2D* tri2)
{
	int i , j;
	for( i = 0 ; i < 3 ; i++ )
	{
		for( j = 0 ; j < 3 ; j++ )
		{
			if( Seg2DIntersectSeg2D(tri1[i],tri1[(i + 1) % 3],tri2[j],tri2[(j + 1) % 3]) )
				return true;
		}
	}
	if(PointTriangleRelation(tri1,tri2[0]) != OUTSIDE)
		return true;
	else if(PointTriangleRelation(tri2,tri1[0]) != OUTSIDE )
		return true;
	return false;
}

int GetMaximumAxis(const Vector3D& normal)
{
	Vector3D tm = Vector3D(fabs(normal.x),fabs(normal.y),fabs(normal.z));
	if( GEQUAL(tm.x,tm.y) && GEQUAL(tm.x,tm.z) )
		return AXIS_X;
	else if( GEQUAL(tm.y,tm.x) && GEQUAL(tm.y,tm.z) )
		return AXIS_Y;
	return AXIS_Z;
}

/**
\brief  把三维的点，移除一个分量，得到二维的点
*/
void Point3DToPoint2D(const Point3D& point,Point2D& result,int axis)
{
	if(axis == AXIS_X)
	{
		result.x = point.y;
		result.y = point.z;
	}
	else if(axis == AXIS_Y)
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

void Point3DToPoint2D(const SrPoint3D*point,int number,SrPoint2D* result,int axisRemoved)
{
	int i = 0;
	for( i=0 ; i<number ; i++ )
	{
		Point3DToPoint2D(point[i],result[i],axisRemoved);
	}
}

bool Tri3DCoplanarTri3D(const SrTriangle3D& tri1,const SrTriangle3D& tri2,int axis)
{
	Point2D tmpTri1[3],tmpTri2[3];
	Point3DToPoint2D(tri1.mPoint,3,tmpTri1,axis);
	Point3DToPoint2D(tri2.mPoint,3,tmpTri2,axis);
	return Tri2DOverlapTestTri2D(tmpTri1,tmpTri2);
}

void ComputeInterval(const Triangle3D& triangle,const Vector3D& direction,int maxAxis,Real* distTriToPlane,Real* t)
{
	Real p[3], tm;
	int indx[3];
	//sort the dist by using the index table.
	int minIndex = 0 , tmpIndx;
	if( LESS(distTriToPlane[1],distTriToPlane[minIndex]) )
		minIndex = 1;
	if( LESS(distTriToPlane[2],distTriToPlane[minIndex]) )
		minIndex = 2;
	int tmp1 = (minIndex + 1)%3, tmp2 = (minIndex + 2)%3;
	indx[0] = minIndex;
	if( LESS(distTriToPlane[tmp1],distTriToPlane[tmp2]) )
	{
		indx[1] = tmp1;
		indx[2] = tmp2;
	}
	else
	{
		indx[1] = tmp2;
		indx[2] = tmp1;;
	}

	if( EQUAL(distTriToPlane[indx[0]],0) && EQUAL(distTriToPlane[indx[1]],0) )
	{
		t[0] = triangle.mPoint[indx[0]][maxAxis];
		t[1] = triangle.mPoint[indx[1]][maxAxis];
	}
	else if( EQUAL(distTriToPlane[indx[1]],0) && EQUAL(distTriToPlane[indx[2]],0) )
	{
		t[0] = triangle.mPoint[indx[1]][maxAxis];
		t[1] = triangle.mPoint[indx[2]][maxAxis];
	}
	else
	{
		if( LESS(distTriToPlane[indx[1]],0) )
		{
			tmpIndx = indx[0];
			indx[0] = indx[2];
			indx[2] = tmpIndx;
		}
		p[0] = triangle.mPoint[indx[0]][maxAxis];
		p[1] = triangle.mPoint[indx[1]][maxAxis];
		p[2] = triangle.mPoint[indx[2]][maxAxis];

		t[0] = p[1] + (p[0] - p[1])*distTriToPlane[indx[1]] / (distTriToPlane[indx[1]] - distTriToPlane[indx[0]]);
		t[1] = p[2] + (p[0] - p[2])*distTriToPlane[indx[2]] / (distTriToPlane[indx[2]] - distTriToPlane[indx[0]]);
	}

	if( GREATER(t[0] , t[1]) )
	{
		tm	 = t[0];
		t[0] = t[1];
		t[1] = tm;
	}
}

/*
	间隔重叠法，判断两个三角形是否重叠
*/

bool Tri3DOverlapTestTri3D_Moller(const Triangle3D& tri1,const Triangle3D& tri2)
{
	Vector3D tri1Normal,tri2Normal;
	Real d, d0, d1;

	//初始化三角形tri1所在的平面
	tri1Normal = (tri1.mPoint[1] - tri1.mPoint[0]).cross(tri1.mPoint[2] - tri1.mPoint[0]);
	d = -tri1Normal.dot(tri1.mPoint[0]);

	Real distTri2ToPlane[3] , distTri1ToPlane[3];
	//三角形tri2上的三个顶点到三角形tri1所在的平面的有符号距离
	distTri2ToPlane[0] = tri1Normal.dot(tri2.mPoint[0]) + d;
	distTri2ToPlane[1] = tri1Normal.dot(tri2.mPoint[1]) + d;
	distTri2ToPlane[2] = tri1Normal.dot(tri2.mPoint[2]) + d;

	//判断有符号距离是否都是正数或者是负
	d0 = distTri2ToPlane[0]*distTri2ToPlane[1];
	d1 = distTri2ToPlane[1]*distTri2ToPlane[2];
	if( GREATER(d0,0) && GREATER(d1,0) )
		return false;

	if( EQUAL(distTri2ToPlane[0],0) && EQUAL(distTri2ToPlane[1],0) && EQUAL(distTri2ToPlane[2],0) )
	{//The two triangles are coplanar.
		int axis = GetMaximumAxis(tri1Normal);
		return Tri3DCoplanarTri3D(tri1,tri2,axis);
	}
	//初始化三角形tri2所在的平面
	tri2Normal = (tri2.mPoint[1] - tri2.mPoint[0]).cross(tri2.mPoint[2] - tri2.mPoint[0]);
	d = -tri2Normal.dot(tri2.mPoint[0]);

	//三角形tri1上的三个顶点到三角形tri2所在的平面的有符号距离
	distTri1ToPlane[0] = tri2Normal.dot(tri1.mPoint[0]) + d;
	distTri1ToPlane[1] = tri2Normal.dot(tri1.mPoint[1]) + d;
	distTri1ToPlane[2] = tri2Normal.dot(tri1.mPoint[2]) + d;

	//判断有符号距离是否都是正数
	d0 = distTri1ToPlane[0]*distTri1ToPlane[1];
	d1 = distTri1ToPlane[1]*distTri1ToPlane[2];
	if( GREATER(d0,0) && GREATER(d1,0) )
		return false;

	//相交直线的方向向量
	Vector3D direction = tri1Normal.cross(tri2Normal);
	Real t0[2] , t1[2];
	int maxAxis = 0 ;
	Vector3D tmpDirection = Vector3D(fabs(direction.x),fabs(direction.y),fabs(direction.z));
	if( GREATER(tmpDirection.x,tmpDirection.y) )
	{
		if( GREATER(tmpDirection.z , tmpDirection.x)  )
			maxAxis = 2;
		else
			maxAxis = 0;
	}
	else
	{
		if( GREATER(tmpDirection.z , tmpDirection.y) )
			maxAxis = 2;
		else
			maxAxis = 1;
	}

	ComputeInterval(tri1,direction,maxAxis,distTri1ToPlane,t0);
	ComputeInterval(tri2,direction,maxAxis,distTri2ToPlane,t1);

	if( GREATER(t0[0] , t1[1]) || LESS(t0[1] , t1[0]) )
		return false;
	return true;
}


Real Determinant(const Point3D& p0,const Point3D& p1,const Point3D& p2,const Point3D& p3)
{
	return (p3-p0).dot((p1-p0).cross(p2-p0));
}

void ReorderVertex(Real* distTriToPlane, int* index)
{
	index[0] = 0;index[1] = 1;index[2] = 2;
	//sort the dist by using the index table.
	int sole = 0 ;
	if( LESS(distTriToPlane[1],distTriToPlane[sole]) )
		sole = 1;
	if( LESS(distTriToPlane[2],distTriToPlane[sole]) )
		sole = 2;
	int tmp1 = (sole + 1) % 3, tmp2 = (sole + 2) % 3;

	if( EQUAL(distTriToPlane[sole],0) )
	{
		if( EQUAL(distTriToPlane[tmp1],0) )
			sole = tmp2;
		else if( EQUAL(distTriToPlane[tmp2],0) )
			sole = tmp1;
	}
	else
	{
		if( LESS(distTriToPlane[tmp1],0) )
			sole = tmp2;
		else if( LESS(distTriToPlane[tmp2],0) )
			sole = tmp1;
	}

	index[0] =  sole;
	index[1] = (sole + 1) % 3;
	index[2] = (sole + 2) % 3;
}


void Transposition(Real*distTri1ToPlane,int *tri1Index,
				   Real*distTri2ToPlane,int* tri2Index)
{
	int tmpIndex;
	if( EQUAL(distTri1ToPlane[tri1Index[0]],0) )
	{
		if( GREATER(distTri1ToPlane[tri1Index[1]] , 0) )
		{
			tmpIndex = tri2Index[1];
			tri2Index[1] = tri2Index[2];
			tri2Index[2] = tmpIndex;
		}
	}
	else
	{
		if( LESS(distTri1ToPlane[tri1Index[0]] , 0) )
		{
			tmpIndex = tri2Index[1];
			tri2Index[1] = tri2Index[2];
			tri2Index[2] = tmpIndex;
		}
	}

	if( EQUAL(distTri2ToPlane[tri2Index[0]],0) )
	{
		if( GREATER(distTri2ToPlane[tri2Index[1]] , 0) )
		{
			tmpIndex = tri1Index[1];
			tri1Index[1] = tri1Index[2];
			tri1Index[2] = tmpIndex;
		}
	}
	else
	{
		if( LESS(distTri2ToPlane[tri2Index[0]] , 0) )
		{
			tmpIndex = tri1Index[1];
			tri1Index[1] = tri1Index[2];
			tri1Index[2] = tmpIndex;
		}
	}
}

bool Tri3DOverlapTestTri3D_Guigue(const Triangle3D& tri1,const Triangle3D& tri2)
{
	Vector3D tri1Normal,tri2Normal;
	tri1Normal = (tri1.mPoint[1] - tri1.mPoint[0]).cross(tri1.mPoint[2] - tri1.mPoint[0]);
	Real distTri2ToPlane[3] , distTri1ToPlane[3];
	distTri2ToPlane[0] = tri1Normal.dot(tri2.mPoint[0] - tri1.mPoint[0]);
	distTri2ToPlane[1] = tri1Normal.dot(tri2.mPoint[1] - tri1.mPoint[0]);
	distTri2ToPlane[2] = tri1Normal.dot(tri2.mPoint[2] - tri1.mPoint[0]);
	Real d0 = distTri2ToPlane[0]*distTri2ToPlane[1];
	Real d1 = distTri2ToPlane[1]*distTri2ToPlane[2];
	if( GREATER(d0,0) && GREATER(d1,0) )
		return false;

	if( EQUAL(distTri2ToPlane[0],0) && EQUAL(distTri2ToPlane[1],0) && EQUAL(distTri2ToPlane[2],0) )
	{//The two triangles are coplanar.
		int axis = GetMaximumAxis(tri1Normal);
		return Tri3DCoplanarTri3D(tri1,tri2,axis);
	}
	tri2Normal = (tri2.mPoint[1] - tri2.mPoint[0]).cross(tri2.mPoint[2] - tri2.mPoint[0]);
	distTri1ToPlane[0] = tri2Normal.dot(tri1.mPoint[0] - tri2.mPoint[0]);
	distTri1ToPlane[1] = tri2Normal.dot(tri1.mPoint[1] - tri2.mPoint[0]);
	distTri1ToPlane[2] = tri2Normal.dot(tri1.mPoint[2] - tri2.mPoint[0]);

	d0 = distTri1ToPlane[0]*distTri1ToPlane[1];
	d1 = distTri1ToPlane[1]*distTri1ToPlane[2];
	if( GREATER(d0,0) && GREATER(d1,0) )
		return false;

	int indexTri1[3] , indexTri2[3];
	ReorderVertex(distTri1ToPlane,indexTri1);
	ReorderVertex(distTri2ToPlane,indexTri2);
	Transposition(distTri1ToPlane,indexTri1,distTri2ToPlane,indexTri2);

	Real d;
	d = Determinant(tri2.mPoint[indexTri2[0]],tri2.mPoint[indexTri2[1]],tri1.mPoint[indexTri1[0]],tri1.mPoint[indexTri1[1]]);
	if( GREATER(d,0) )
		return false;
	d = Determinant(tri2.mPoint[indexTri2[0]],tri2.mPoint[indexTri2[2]],tri1.mPoint[indexTri1[2]],tri1.mPoint[indexTri1[0]]);
	if( GREATER(d,0) )
		return false;
	return true;
}

bool Tri3DOverlapTestTri3D_Naive(const SrTriangle3D& tri1,const SrTriangle3D& tri2)
{
	SrVector3D tri1Normal,tri2Normal;
	SrReal d;
	tri1Normal = (tri1.mPoint[1] - tri1.mPoint[0]).cross(tri1.mPoint[2] - tri1.mPoint[0]);
	d = -tri1Normal.dot(tri1.mPoint[0]);

	SrReal distTri2ToPlane[3],distTri1ToPlane[3];
	distTri2ToPlane[0] = tri1Normal.dot(tri2.mPoint[0]) + d;
	distTri2ToPlane[1] = tri1Normal.dot(tri2.mPoint[1]) + d;
	distTri2ToPlane[2] = tri1Normal.dot(tri2.mPoint[2]) + d;
	SrReal d0 = distTri2ToPlane[0] * distTri2ToPlane[1];
	SrReal d1 = distTri2ToPlane[1] * distTri2ToPlane[2];
	//The three vertex of triangle tri2 is on the same side of the plane initialized by triangle tri1.
	if( GREATER(d0,0) && GREATER(d1,0) )
		return false;

	if( EQUAL(distTri2ToPlane[0],0) && EQUAL(distTri2ToPlane[1],0) && EQUAL(distTri2ToPlane[2],0) )
	{//The two triangles are coplanar.
		int axis = GetMaximumAxis(tri1Normal);
		return Tri3DCoplanarTri3D(tri1,tri2,axis);
	}

	tri2Normal = (tri2.mPoint[1] - tri2.mPoint[0]).cross(tri2.mPoint[2] - tri2.mPoint[0]);
	d = -tri2Normal.dot(tri2.mPoint[0]);
	distTri1ToPlane[0] = tri2Normal.dot(tri1.mPoint[0]) + d;
	distTri1ToPlane[1] = tri2Normal.dot(tri1.mPoint[1]) + d;
	distTri1ToPlane[2] = tri2Normal.dot(tri1.mPoint[2]) + d;

	d0 = distTri1ToPlane[0]*distTri1ToPlane[1];
	d1 = distTri1ToPlane[1]*distTri1ToPlane[2];

	if( GREATER(d0,0) && GREATER(d1,0) )
		return false;
	int i;
	for( i=0 ; i<3 ; i++ )
	{
		if( tri2.segmentHitTest(tri1.mPoint[i], tri1.mPoint[(i+1)%3]) == SR_INTERSECTING )
			return true;
	}
	for( i=0 ; i<3 ; i++ )
	{
		if( tri1.segmentHitTest(tri2.mPoint[i], tri2.mPoint[(i+1)%3])==SR_INTERSECTING )
			return true;
	}
	return false;
}


void ComputeIntersection(const SrTriangle3D& triangle,SrReal* distTriToPlane,const SrVector3D& normal,SrReal d,SrPoint3D* outPoint)
{
	int indx[3];
	//sort the dist by using the index table.
	int minIndex = 0 , tmpIndx;
	if( LESS(distTriToPlane[1],distTriToPlane[minIndex]) )
		minIndex = 1;
	if( LESS(distTriToPlane[2],distTriToPlane[minIndex]) )
		minIndex = 2;
	int tmp1 = (minIndex + 1)%3, tmp2 = (minIndex + 2)%3;
	indx[0] = minIndex;
	if( LESS(distTriToPlane[tmp1],distTriToPlane[tmp2]) )
	{
		indx[1] = tmp1;
		indx[2] = tmp2;
	}
	else
	{
		indx[1] = tmp2;
		indx[2] = tmp1;;
	}

	if( EQUAL(distTriToPlane[indx[1]],0) )
	{
		if( EQUAL(distTriToPlane[indx[0]],0) )
		{
			outPoint[0] = triangle.mPoint[indx[0]];
			outPoint[1] = triangle.mPoint[indx[1]];
			return;
		}
		else if( EQUAL(distTriToPlane[indx[2]],0) )
		{
			outPoint[0] = triangle.mPoint[indx[1]];
			outPoint[1] = triangle.mPoint[indx[2]];
			return;
		}
	}
	else if( LESS(distTriToPlane[indx[1]],0) )
	{
		tmpIndx = indx[0];
		indx[0] = indx[2];
		indx[2] = tmpIndx;
	}
	SrVector3D direction = triangle.mPoint[indx[1]] - triangle.mPoint[indx[0]];
	SrReal denom = direction.dot(normal);
	SrReal t = -(d + normal.dot(triangle.mPoint[indx[0]])) / denom;
	outPoint[0] = triangle.mPoint[indx[0]] + t*direction;

	direction = triangle.mPoint[indx[2]] - triangle.mPoint[indx[0]];
	denom = direction.dot(normal);
	t = -(d + normal.dot(triangle.mPoint[indx[0]])) / denom;
	outPoint[1] = triangle.mPoint[indx[0]] + t*direction;
}
bool Tri3DOverlapTestTri3D_ERIT(const SrTriangle3D& tri1,const SrTriangle3D& tri2)
{
	SrVector3D tri1Normal,tri2Normal;
	SrReal tri1d,tri2d;
	tri1Normal = (tri1.mPoint[1] - tri1.mPoint[0]).cross(tri1.mPoint[2] - tri1.mPoint[0]);
	tri1d = -tri1Normal.dot(tri1.mPoint[0]);

	SrReal distTri2ToPlane[3] , distTri1ToPlane[3];
	distTri2ToPlane[0] = tri1Normal.dot(tri2.mPoint[0]) + tri1d;
	distTri2ToPlane[1] = tri1Normal.dot(tri2.mPoint[1]) + tri1d;
	distTri2ToPlane[2] = tri1Normal.dot(tri2.mPoint[2]) + tri1d;

	SrReal d0 = distTri2ToPlane[0] * distTri2ToPlane[1];
	SrReal d1 = distTri2ToPlane[1] * distTri2ToPlane[2];
	//The three vertex of triangle tri2 is on the same side of the plane initialized by triangle tri1.
	if( GREATER(d0,0) && GREATER(d1,0) )
	{
		//printf("first\n");
		return false;
	}

	if( EQUAL(distTri2ToPlane[0],0) && EQUAL(distTri2ToPlane[1],0) && EQUAL(distTri2ToPlane[2],0) )
	{//The two triangles are coplanar.
		int axis = GetMaximumAxis(tri1Normal);
		return Tri3DCoplanarTri3D(tri1,tri2,axis);
	}
	tri2Normal = (tri2.mPoint[1] - tri2.mPoint[0]).cross(tri2.mPoint[2] - tri2.mPoint[0]);
	tri2d = -tri2Normal.dot(tri2.mPoint[0]);
	distTri1ToPlane[0] = tri2Normal.dot(tri1.mPoint[0]) + tri2d;
	distTri1ToPlane[1] = tri2Normal.dot(tri1.mPoint[1]) + tri2d;
	distTri1ToPlane[2] = tri2Normal.dot(tri1.mPoint[2]) + tri2d;

	d0 = distTri1ToPlane[0]*distTri1ToPlane[1];
	d1 = distTri1ToPlane[1]*distTri1ToPlane[2];

	if( GREATER(d0,0) && GREATER(d1,0) )
	{
		//printf("GREATER(d0,0) && GREATER(d1,0)\n");
		return false;
	}

	SrPoint3D inPoint[2];
	SrPoint2D point[3];
	SrPoint2D p0, p1;
	ComputeIntersection(tri2,distTri2ToPlane,tri1Normal,tri1d,inPoint);
	int axis = GetMaximumAxis(tri1Normal) , i;
	Point3DToPoint2D(inPoint[0],p0,axis);
	Point3DToPoint2D(inPoint[1],p1,axis);
	Point3DToPoint2D(tri1.mPoint,3,point,axis);
	int status = PointTriangleRelation(point,p0);
	if( status != OUTSIDE )
		return true;
	for( i=0 ; i<3 ; i++ )
	{
		if(Seg2DIntersectSeg2D(p0,p1,point[i],point[(i+1)%3]))
			return true;
	}
	//printf("return false;\n");
	return false;
}

void testTriangle3dIntersectTriangle3d_Guigue(SrTriangle3D* tri1,SrTriangle3D* tri2,int numCase)
{
	int i;
	for( i=0 ; i<numCase ; i++ )
	{
		int status1 = Tri3DOverlapTestTri3D_Guigue(tri1[i],tri2[i]);
		int status2 = Tri3DOverlapTestTri3D_Naive(tri1[i],tri2[i]);
		ASSERT(status1==status2);
	}
	double seconds = clock();
	for( i=0 ; i<numCase ; i++ )
	{
		Tri3DOverlapTestTri3D_Guigue(tri1[i],tri2[i]);
	}
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("Time:	%.6lf\n",seconds);
}

void testTriangle3dIntersectTriangle3d_Naive(SrTriangle3D* tri1,SrTriangle3D* tri2,int numCase)
{
	int i;
	double seconds = clock();
	for( i=0 ; i<numCase ; i++ )
	{
		Tri3DOverlapTestTri3D_Naive(tri1[i],tri2[i]);
	}
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("Time:	%.6lf\n",seconds);
}

void testTriangle3dIntersectTriangle3d_Moller(SrTriangle3D* tri1,SrTriangle3D* tri2,int numCase)
{
	int i;
	//Test the correctness of moller algorithms
	for( i=0 ; i<numCase ; i++ )
	{
		bool status1 = Tri3DOverlapTestTri3D_Moller(tri1[i],tri2[i]);
		bool status2 = Tri3DOverlapTestTri3D_Naive(tri1[i],tri2[i]);
		ASSERT(status1==status2);
	}
	double seconds = clock();
	for( i=0 ; i<numCase ; i++ )
	{
		Tri3DOverlapTestTri3D_Moller(tri1[i],tri2[i]);
	}
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("Time:	%.6lf\n",seconds);
}

void testTriangle3dIntersectTriangle3d_ERIT(SrTriangle3D* tri1,SrTriangle3D* tri2,int numCase)
{

	int i;
	for( i=0 ; i<numCase ; i++ )
	{
		bool status1 = Tri3DOverlapTestTri3D_ERIT(tri1[i],tri2[i]);
		bool status2 = Tri3DOverlapTestTri3D_Naive(tri1[i],tri2[i]);
		if(status1 != status2)
			printf("%d,%d\n",status1,status2);
		ASSERT(status1 == status2);
	}
	double seconds = clock();
	for( i=0 ; i<numCase ; i++ )
	{
		Tri3DOverlapTestTri3D_ERIT(tri1[i],tri2[i]);
	}
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("Time:	%.6lf\n",seconds);
}


void testTriangle3dIntersectTriangle3d()
{
	int numCase = 100000;
	SrTriangle3D* tri1 = new SrTriangle3D[numCase];
	SrTriangle3D* tri2 = new SrTriangle3D[numCase];
	int i , j;

	for( i=0 ; i<numCase ; i++ )
	{
		do 
		{
			for( j=0 ; j<3 ; j++ )
			{
				tri1[i].mPoint[j].x = rand()%50;
				tri1[i].mPoint[j].y = rand()%50;
				tri1[i].mPoint[j].z = rand()%50;

			}
		} while (!tri1[i].isValid());

		do 
		{
			for( j=0 ; j<3 ; j++ )
			{
				tri2[i].mPoint[j].x = rand()%100;
				tri2[i].mPoint[j].y = rand()%100;
				tri2[i].mPoint[j].z = rand()%100;

			}
		} while (!tri2[i].isValid());
	}

	printf("Naive Algorithm:\n");
	testTriangle3dIntersectTriangle3d_Naive(tri1,tri2,numCase);
	printf("Moller Algorithm:\n");
	testTriangle3dIntersectTriangle3d_Moller(tri1,tri2,numCase);
	printf("Guigue Algorithm:\n");
	testTriangle3dIntersectTriangle3d_Guigue(tri1,tri2,numCase);
	printf("ERIT Algorithm:\n");
	testTriangle3dIntersectTriangle3d_ERIT(tri1,tri2,numCase);

	delete []tri1;
	delete []tri2;
}


int main( )
{
	testTriangle3dIntersectTriangle3d();
	return 0;
}