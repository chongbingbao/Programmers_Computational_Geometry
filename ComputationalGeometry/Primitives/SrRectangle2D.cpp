/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/05/09
****************************************************************************/
#include "SrRectangle2D.h"
#include "SrPrimitive.h"

SrRectangle2D::SrRectangle2D()
{
	mCenter = SrVector2D(0,0);
	mAxis[0] = mAxis[1] = SrVector2D(0,0);
	mHalfLength[0] = mHalfLength[1] = -1;
}

SrRectangle2D::SrRectangle2D(const SrPoint2D& center,const SrPoint2D* axis,const SrReal* halfLength)
{
	mCenter = center;
	mAxis[0] = axis[0];
	mAxis[1] = axis[1];
	mHalfLength[0] = halfLength[0];
	mHalfLength[1] = halfLength[1];
}

SrRectangle2D::~SrRectangle2D()
{

}

bool SrRectangle2D::isValid() const
{
	//The axis is unit length.
	if( UNEQUAL(mAxis[0].magnitudeSquared(),1.0) || EQUAL(mAxis[1].magnitudeSquared(),1.0))
		return false;
	//The half length of the two axises shouldn't be less than or equal to zero.
	if( LEQUAL(mHalfLength[0],0) || LEQUAL(mHalfLength[1],0) )
		return false;
	//One axis is perpendicular to the other one.
	if( UNEQUAL(mAxis[0].dot(mAxis[1]),0) ) 
		return false;
	return true;
}

SrReal SrRectangle2D::area() const
{
	return 4*mHalfLength[0]*mHalfLength[1];
}

SrReal SrRectangle2D::toPointDistanceSquared(const SrPoint2D& p) const
{
	SrVector2D  delta = p - mCenter;
	SrReal dist = 0.0;

	SrReal s1 = mAxis[0].dot(delta), s2 = mAxis[1].dot(delta);
	SrReal s1Pe1 = s1 + mHalfLength[0];
	if( LESS(s1Pe1,0) )
	{
		dist += s1Pe1*s1Pe1;
	}
	else
	{
		s1Pe1 = s1 - mHalfLength[0];
		if( GREATER(s1Pe1,0) )
		{
			dist += s1Pe1*s1Pe1;
		}
	}
	SrReal s2Pe2 = s2 + mHalfLength[1];
	if( LESS(s2Pe2,0) )
	{
		dist += s2Pe2*s2Pe2;
	}
	else
	{
		s2Pe2 = s2 - mHalfLength[1];
		if( GREATER(s2Pe2,0) )
		{
			dist += s2Pe2*s2Pe2;
		}
	}
	return dist;
}

SrReal SrRectangle2D::toPointDistance(const SrPoint2D& p) const
{
	return SrMath::sqrt(toPointDistanceSquared(p));
}

/*
只要判断该点的横坐标和纵坐标是否夹在矩形的左右边和上下边之间。
例如：矩形四个顶点P1，P2，P3，P4，判断P是否包含在矩形中，
只需要判断：|P2P|×|P2P1|*|P3P|×|P3P4|<=0 and |P1P|×|P1P4|*|P2P|×|P2P3|<=0
*/
int	SrRectangle2D::pointLocation(const SrPoint2D& p) const
{
	SrVector2D delta[2];
	delta[0] = mHalfLength[0]*mAxis[0];
	delta[1] = mHalfLength[1]*mAxis[1];
	SrPoint2D p0 = mCenter - delta[0] - delta[1];
	SrPoint2D p1 = mCenter - delta[0] + delta[1];
	SrPoint2D p2 = mCenter + delta[0] - delta[1];
	SrPoint2D p3 = mCenter + delta[0] + delta[1];
	SrReal tmp1 = (p-p0).cross(p1-p0) * (p - p2).cross(p3-p2);
	if( GREATER(tmp1,0) )
		return SR_POSITION_OUTSIDE;
	SrReal tmp2 = (p-p0).cross(p2-p0) * (p - p1).cross(p3-p1);
	if( GREATER(tmp2,0) )
		return SR_POSITION_OUTSIDE;
	if( EQUAL(tmp2,0) || EQUAL(tmp1,0) )
		return SR_POSITION_ON;
	return SR_POSITION_INSIDE;
}

SrReal SrRectangle2D::perimeter() const
{
	return 2*(mHalfLength[0] + mHalfLength[1]);
}

void SrRectangle2D::rotate(SrReal angle)
{
	SrF64 c = SrMath::cos(angle*SrPiF64/180.0) , s =SrMath::sin(angle*SrPiF64/180.0);
	mCenter.set( mCenter.x*c-mCenter.y*s , mCenter.x*s+mCenter.y*c);
	mAxis[0].set( mAxis[0].x*c-mAxis[0].y*s , mAxis[0].x*s+mAxis[0].y*c);
	mAxis[1].set( mAxis[1].x*c-mAxis[1].y*s , mAxis[1].x*s+mAxis[1].y*c);
}

void SrRectangle2D::translate(const SrVector2D& offset)
{
	mCenter.set( mCenter.x + offset.x , mCenter.y + offset.y);
}

bool SrRectangle2D::hitTest(const SrRay2D& ray,SrPoint2D& result) const
{
	//ASSERT(ray.isValid());
	//ASSERT(isValid());
	//SrPoint2D edge[4][2],p4 = mPoint[1] + mPoint[2] - mPoint[0];
	//SrPoint2D temp,nearPoint;
	//edge[0][0] = mPoint[0]; edge[0][1] = mPoint[1];
	//edge[1][0] = mPoint[0]; edge[1][1] = mPoint[2];
	//edge[2][0] = mPoint[2]; edge[2][1] = p4;
	//edge[3][0] = mPoint[1]; edge[3][1] = p4;

	//int hitState , i;
	//SrReal tMin = SR_MAX_F32 , tTemp1 , tTemp2;
	//for( i=0 ; i<4 ; i++ )
	//{
	//	hitState = ray.intersectSegment(SrSegment2D(edge[i][0],edge[i][1]),temp);
	//	if( hitState==LINEAR_OVERLAP )
	//	{//Overlapping.
	//		if( fabs(ray.mDirection.x)>fabs(ray.mDirection.y) )
	//		{
	//			tTemp1 = (edge[i][0].x - ray.mBase.x) / ray.mDirection.x;
	//			tTemp2 = (edge[i][1].x - ray.mBase.x) / ray.mDirection.x;
	//			if( tTemp1<tTemp2 && tTemp1<tMin )
	//			{
	//				tMin = tTemp1;
	//				nearPoint = edge[i][0];
	//			}
	//			else if( tTemp2<tTemp1 && tTemp2<tMin )
	//			{
	//				tMin = tTemp2;
	//				nearPoint = edge[i][1];
	//			}
	//		}
	//		else
	//		{
	//			tTemp1 = (edge[i][0].y - ray.mBase.y) / ray.mDirection.y;
	//			tTemp2 = (edge[i][1].y - ray.mBase.y) / ray.mDirection.y;
	//			if( tTemp1<tTemp2 && tTemp1<tMin )
	//			{
	//				tMin = tTemp1;
	//				nearPoint = edge[i][0];
	//			}
	//			else if( tTemp2<tTemp1 && tTemp2<tMin )
	//			{
	//				tMin = tTemp2;
	//				nearPoint = edge[i][1];
	//			}
	//		}
	//	}
	//	else if( hitState==LINEAR_INTERSECT ||  hitState==LINEAR_INTERSECT_ENDPOINT )
	//	{//Intersecting.
	//		if( fabs(ray.mDirection.x)>fabs(ray.mDirection.y) )
	//		{
	//			tTemp1 = (temp.x - ray.mBase.x) / ray.mDirection.x;
	//		}
	//		else
	//		{
	//			tTemp1 = (temp.y - ray.mBase.y) / ray.mDirection.y;
	//		}
	//		if( tTemp1<tMin )
	//		{
	//			tMin = tTemp1;
	//			nearPoint = temp;
	//		}
	//	}
	//}
	//if( tMin != SR_MAX_F32 )
	//{
	//	if( tMin<=0 )
	//		result = ray.mBase;
	//	else
	//		result = nearPoint;
	//	return true;
	//}
	return false;
}

