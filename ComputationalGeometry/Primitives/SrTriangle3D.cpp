/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/04/16
****************************************************************************/
#include "SrTriangle3D.h"
#include "SrPlane3D.h"
#include "SrPrimitive.h"
#include "SrTriangle2D.h"

SrTriangle3D::SrTriangle3D()
{
	mPoint[0] = mPoint[1] = mPoint[1] = SrVector3D(0,0,0);
}

SrTriangle3D::SrTriangle3D(const SrPoint3D& p0,const SrPoint3D& p1,const SrPoint3D& p2)
{
	mPoint[0] = p0;
	mPoint[1] = p1;
	mPoint[2] = p2;
}


SrTriangle3D::~SrTriangle3D()
{

}


/*
--------------------------------------------------------------------------
Reference:<Geometric Tools for computer Graphics> 10.3.2
We define a triangle T with vertices {V0, V1, V2} parametrically
					T (s,t) = B + s^e0 + t^e1
  for (s, t)¡ÊD={(s,t):s¡Ê[0,1],t¡Ê[0,1],s+t¡Ü1},B=V0,e0=V1-V0,and e1=V2-V0
The minimum distance is computed by locating the values (~s,~t)¡ÊD 
corresponding to the point P',the closest point on the triangle to P.
	Q(s,t) = ||T(s,t)-P||^2 = as^2+2bst+ct^2+2ds+2et+f
a=^e0¡¤^e0,b=^e0¡¤^e1,c=^e0¡¤^e1,d=^e0¡¤(B-P),e=^e1¡¤(B-P),f=(B-P)¡¤(B-P)
For Q, ac-b^2>0.Since Q is a continuously differentiable function, the minimum 
occurs either at an interior point of D where the gradient 
			grad(Q) = 2(as + bt + d, bs + ct + e) = (0, 0),Then
			(~s,~t)=( (be-cd)/(ac-b^2) , (bd-ad)/(ac-b^2) )
Based on region that (~s,~t) belong to,the algorithm computes the minimum distance.  
						   2	
						  \ |
						   \|
							|\
						   3| \
							|  \
							| 0 \ 1
						 ___|____\_____
						  4 |  5  \ 6
--------------------------------------------------------------------------
*/
SrReal	SrTriangle3D::toPointDistance(const SrPoint3D& p) const
{
	SrVector3D e0 = mPoint[1] - mPoint[0], e1 = mPoint[2] - mPoint[0];
	SrVector3D dif = mPoint[0] - p;
	SrReal a = e0.dot(e0);
	SrReal b = e0.dot(e1);
	SrReal c = e1.dot(e1);
	SrReal d = e0.dot(dif);
	SrReal e = e1.dot(dif);
	SrReal f = dif.dot(dif);
	SrReal det = a*c - b*b;
	SrReal s = b*e - c*d;
	SrReal t = b*d - a*e;
	SrReal tmp0,tmp1;
	SrReal numer , denom;
	if( LEQUAL(s+t,det) )
	{
		if( LESS(s,0) )
		{
			if( LESS(t,0) )
			{//region 4
				//It's similar to region 2.
				if( LESS(d,0) )
				{
					t = 0;
					s = -d>=a?1:-d/a;
				}
				else
				{
					s = 0;
					t = (GEQUAL(e,0)?0:(GEQUAL(-e,c)?1:-e/c));
				}
			}
			else
			{//region 3
				//F(t)=Q(0,t)=ct^2+et+f,F'(t)/2=ct+e
				//t = -e/c
				s = 0;
				t = (GEQUAL(e,0)?0:(GEQUAL(-e,c)?1:-e/c));
			}
		}
		else
		{
			if( LESS(t,0) )
			{//region 5
				//It's similar to region 3.
				t = 0;
				s = (GEQUAL(d,0)?0:(GEQUAL(-d,a)?1:-d/a));
			}
			else
			{//region 0
				SrReal invDet = 1 / det;
				s *= invDet;
				t *= invDet;
			}
		}
	}
	else
	{
		if( LESS(s,0) )
		{//region 2
			//Grad(Q) = 2(as+bt+d,bs+ct+e)
			//(0,-1)*Grad(Q(0,1)) = (0,-1)*(b+d,c+e) = -(c+e)
			//(1,-1)*Grad(Q(0,1)) = (1,-1)*(b+d,c+e) = (b+d)-(c+e)
			//min on edge s+t=1 if (1,-1)*Grad(Q(0,1))<0
			//min on edge s = 0 otherwise
			tmp0 = b + d;
			tmp1 = c + e;
			if( GREATER(tmp1,tmp0) )
			{
				numer = tmp1 - tmp0;
				denom = a - 2*b + c;
				s = (GEQUAL(numer,denom)?1:numer/denom);
				t = 1-s;
			}
			else
			{
				s = 0;
				t = (LEQUAL(tmp1,0)?1:(GEQUAL(e,0)?0:-e/c));
			}
		}
		else if( LESS(t,0) )
		{//region 6
			//It's similar to region 2.
			tmp0 = b + e;
			tmp1 = a + d;
			if( GREATER(tmp1,tmp0) )
			{
				numer = c + e - b - d;
				if( LEQUAL(numer,0) )
					s = 0;
				else
				{
					denom = a - 2*b + c;
					s = (GEQUAL(numer,denom)?1:numer/denom);
				}
				t = 1-s;
			}
			else
			{
				t = 0;
				s = (GEQUAL(d,0)?0:(GEQUAL(-d,a)?1:-d/a));
			}

		}
		else
		{//region 1
			//F(s) = Q(s,1-s),F'(s)/2 = (a-2b+c)s+(b-c+d-e) = 0,then
			//s = (c + e - b -d)/(a-2b+c). a-2b+c=||^e0-^e1||^2>0.
			numer = c + e - b - d;
			if( LEQUAL(numer,0) )
				s = 0;
			else
			{
				denom = a - 2*b + c;
				s = (GEQUAL(numer,denom)?1:numer/denom);
			}
			t = 1 - s;
		}
	}
	//The distance may be less than 0 because of floating imprecision.
	SrReal distance = s*(a*s + 2*b*t + 2*d) + t*(c*t + 2*e) + f;
	if( EQUAL(distance,0) )
		distance = 0;
	return SrMath::sqrt(distance);
}

bool	SrTriangle3D::isValid() const
{
	SrVector3D norm = normal();
	if( EQUAL(norm.x,0) && EQUAL(norm.y,0) && EQUAL(norm.z,0) )
		return false;
	return true;
}


SrReal	SrTriangle3D::area() const
{
	SrVector3D norm = normal();
	SrVector3D areaTotal = mPoint[2].cross(mPoint[0]);
	int i;
	for( i=0 ; i<2 ; i++ )
	{
		areaTotal += mPoint[i].cross(mPoint[i+1]);
	}
	norm.normalize();
	return norm.dot(areaTotal)*0.5;
}

const SrVector3D SrTriangle3D::normal()const
{
	SrVector3D norm = (mPoint[1] - mPoint[0]).cross(mPoint[2] - mPoint[0]);
	return norm;
}

int		SrTriangle3D::pointLocation(const SrPoint3D& p) const
{
	SrPlane3D plane;
	//Judge whether the three points determine a plane or not.
	ASSERT(plane.initPlane(mPoint[0],mPoint[1],mPoint[2]));
	if( !plane.isOnPlane(p) )
		return SR_POSITION_INVALID;

	SrReal flag1 = plane.mNormal.dot((mPoint[1]-mPoint[0]).cross(p-mPoint[0]));
	if( LESS(flag1,0) )
		return SR_POSITION_OUTSIDE;
	SrReal flag2 = plane.mNormal.dot((mPoint[2]-mPoint[1]).cross(p-mPoint[1]));
	if( LESS(flag2,0) )
		return SR_POSITION_OUTSIDE;
	SrVector3D tmp = (mPoint[0]-mPoint[2]).cross(p-mPoint[2]);
	SrReal flag3 = plane.mNormal.dot((mPoint[0]-mPoint[2]).cross(p-mPoint[2]));
	if( LESS(flag3,0) )
		return SR_POSITION_OUTSIDE;
	if( EQUAL(flag1,0) || EQUAL(flag2,0) || EQUAL(flag3,0) )
		return SR_POSITION_ON;

	return SR_POSITION_INSIDE;
}

SrReal	SrTriangle3D::perimeter() const
{
	return (mPoint[1]-mPoint[0]).magnitude() + 
		   (mPoint[2]-mPoint[1]).magnitude() +
		   (mPoint[0]-mPoint[2]).magnitude();
}

void SrTriangle3D::rotateX(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mPoint[0].set(mPoint[0].x,mPoint[0].y*c-mPoint[0].z*s,mPoint[0].y*s+mPoint[0].z*c);
	mPoint[1].set(mPoint[1].x,mPoint[1].y*c-mPoint[1].z*s,mPoint[1].y*s+mPoint[1].z*c);
	mPoint[2].set(mPoint[2].x,mPoint[2].y*c-mPoint[2].z*s,mPoint[2].y*s+mPoint[2].z*c);
}                                                                              

void SrTriangle3D::rotateY(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mPoint[0].set(mPoint[0].x*c+mPoint[0].z*s,mPoint[0].y,-mPoint[0].x*s+mPoint[0].z*c);
	mPoint[1].set(mPoint[1].x*c+mPoint[1].z*s,mPoint[1].y,-mPoint[1].x*s+mPoint[1].z*c);
	mPoint[2].set(mPoint[2].x*c+mPoint[2].z*s,mPoint[2].y,-mPoint[2].x*s+mPoint[2].z*c);
}

void SrTriangle3D::rotateZ(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mPoint[0].set(mPoint[0].x*c-mPoint[0].y*s,mPoint[0].x*s+mPoint[0].y*c,mPoint[0].z);
	mPoint[1].set(mPoint[1].x*c-mPoint[1].y*s,mPoint[1].x*s+mPoint[1].y*c,mPoint[1].z);
	mPoint[2].set(mPoint[2].x*c-mPoint[2].y*s,mPoint[2].x*s+mPoint[2].y*c,mPoint[2].z);
}

void SrTriangle3D::translate(const SrVector3D& offset)
{
	mPoint[0].set(mPoint[0].x+offset.x,mPoint[0].y+offset.y,mPoint[0].z+offset.z);
	mPoint[1].set(mPoint[1].x+offset.x,mPoint[1].y+offset.y,mPoint[1].z+offset.z);
	mPoint[2].set(mPoint[2].x+offset.x,mPoint[2].y+offset.y,mPoint[2].z+offset.z);
}

int SrTriangle3D::lineHitTest(const SrLine3D& line,SrPoint3D& /*[OUT]*/ result)const
{
	SrVector3D ret;
	int retFlag = linearIntersectTriangle(line.mBase,line.mDirection,ret);
	if( retFlag!=SR_INTERSECTING )
		return retFlag;
	result = line.mBase + ret.z*line.mDirection;
	//check whether the intersection point is on the edge.
	//if( EQUAL(ret.x+ret.y-1.0,0) || 
	//	EQUAL(ret.x,0) || 
	//	EQUAL(ret.y,0) )
	//	return SR_INTERSECTING_EDGE;
	return SR_INTERSECTING;
}

int SrTriangle3D::rayHitTest(const SrRay3D& ray,SrPoint3D& /*[OUT]*/ result)const
{
	SrVector3D ret;
	int retFlag = linearIntersectTriangle(ray.mBase,ray.mDirection,ret);
	if( retFlag!=SR_INTERSECTING )
		return retFlag;
	//check t. t>=0
	if( LESS(ret.z,0) )
		return SR_DISJOINT;
	result = ray.mBase + ret.z*ray.mDirection;
	//check whether the intersection point is on the edge.
	//if( EQUAL(ret.x+ret.y-1.0,0) || 
	//	EQUAL(ret.x,0) || 
	//	EQUAL(ret.y,0) )
	//	return SR_INTERSECTING_EDGE;

	return SR_INTERSECTING;
}

int SrTriangle3D::segmentHitTest(const SrSegment3D& segment,SrPoint3D& /*[OUT]*/ result)const
{
	SrVector3D ret;
	SrVector3D direction = segment.mPoint2 - segment.mPoint1;
	int retFlag = linearIntersectTriangle(segment.mPoint1,direction,ret);
	if( retFlag!=SR_INTERSECTING )
		return retFlag;
	//check t. 0<= t <=1
	if( LESS(ret.z,0) || GREATER(ret.z,1) )
		return SR_DISJOINT;
	result = segment.mPoint1 + ret.z*direction;
	//check whether the intersection point is on the edge.
	//if( EQUAL(ret.x+ret.y-1.0,0) || 
	//	EQUAL(ret.x,0) || 
	//	EQUAL(ret.y,0) )
	//	return SR_INTERSECTING_EDGE;
	return SR_INTERSECTING;
}

/*
--------------------------------------------------------------------------
Devised by Moller and Trumbore,1997. <Fast,minimum storage ray-triangle 
intersection>
Any point in a triangle can be defined in terms of its position relative
to the triangle¡¯s vertices:
		Qu,v = (1-u-v)V0 + uV1 + vV2 ,0<=u<=1,0<=v<=1,0<=u+v<=1
For the linear component¨Ctriangle intersection,
		P + t*^d = Qu,v
which can be expanded and applied Cramer¡¯s rule to:
		|t|							| |P-V0  V1-V0  V2-V0| |
		|u| = 1/|-^d  V1-V0  V2-V0| | |-^d   P-V0   V2-V0| |
		|v|							| |-^d   V1-V0   P-V0| |
									

										| ((P-V0)¡Á(V1-V0))¡¤(V2-V0) |
			= 1/(^d¡Á(V2-v0))¡¤(V1-V0)	|    (^d¡Á(V2-V0))¡¤(P-V0)   |
										|    ((P-V0)¡Á(V1-V0))¡¤^d	 |
--------------------------------------------------------------------------
*/
int SrTriangle3D::linearIntersectTriangle(const SrPoint3D& base,const SrVector3D& direction,SrVector3D& result)const
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

void SrTriangle3D::computeInterval(const SrTriangle3D& triangle,const SrVector3D& direction,int maxAxis,SrReal* distTriToPlane,SrReal* t)const
{
	SrReal p[3] ;
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
		t[0] = direction[maxAxis] * triangle.mPoint[indx[0]][maxAxis];
		t[1] = direction[maxAxis] * triangle.mPoint[indx[1]][maxAxis];
	}
	else if( EQUAL(distTriToPlane[indx[1]],0) && EQUAL(distTriToPlane[indx[2]],0) )
	{
		t[0] = direction[maxAxis] * triangle.mPoint[indx[1]][maxAxis];
		t[1] = direction[maxAxis] * triangle.mPoint[indx[2]][maxAxis];
	}
	else
	{
		if( LESS(distTriToPlane[indx[1]],0) )
		{
			tmpIndx = indx[0];
			indx[0] = indx[2];
			indx[2] = tmpIndx;
		}
		p[0] = direction[maxAxis] * triangle.mPoint[indx[0]][maxAxis];
		p[1] = direction[maxAxis] * triangle.mPoint[indx[1]][maxAxis];
		p[2] = direction[maxAxis] * triangle.mPoint[indx[2]][maxAxis];

		t[0] = p[1] + (p[0] - p[1])*distTriToPlane[indx[1]] / (distTriToPlane[indx[1]] - distTriToPlane[indx[0]]);
		t[1] = p[2] + (p[0] - p[2])*distTriToPlane[indx[2]] / (distTriToPlane[indx[2]] - distTriToPlane[indx[0]]);
	}

	if( GREATER(t[0] , t[1]) )
	{
		SrReal tempSwap ;
		tempSwap = t[0];
		t[0]	 = t[1];
		t[1]	 = tempSwap;
	}
}
