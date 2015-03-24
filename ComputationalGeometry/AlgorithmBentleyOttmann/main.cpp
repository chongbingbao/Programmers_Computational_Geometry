/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/04
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"

#include <stdio.h>
#include <algorithm>
#include <stdlib.h>
#include <list>
#include <set>
#include <queue>
#include <map>
using namespace std;

typedef SrPoint2D		Point2D;
typedef SrVector2D		Vector2D;
typedef SrReal			Real;

#define  EPS			SR_EPS

#define POINT_TYPE_LEFT			0x01
#define POINT_TYPE_RIGHT		0x02
#define	POINT_TYPE_INTER		0x04


/*
\brief 斜率
*/
#define SLOPE_INVALID		0x00
#define	SLOPE_INFINITY		0x01
#define SLOPE_NORMAL		0x02

class Slope
{
private:
	int	 mType;
	Real mValue;
public:
	Slope()
	{
		mType = SLOPE_INVALID;
		mValue = 0;
	}
	void make(const Point2D& p0, const Point2D& p1)
	{
		if( EQUAL(p0.x,p1.x) )
		{
			mType = SLOPE_INFINITY;
		}
		else
		{
			mType  = SLOPE_NORMAL;
			mValue = (p1.y - p0.y) / (p1.x - p0.x);
		}
	}
	int getType()const
	{
		return mType;
	}
	Real getValue()const
	{
		return mValue;
	}
	bool operator < (const Slope& s) const
	{
		if( mType==SLOPE_INFINITY ) 
			return false;
		return (s.mType==SLOPE_INFINITY || LESS(mValue,s.mValue));
	}
};

/*
\brief 线段
*/
class Segment
{
private:
	Point2D	mPoint1;
	Point2D	mPoint2;
	Slope	mSlope;
public:
	Segment()
	{
		mPoint1 = Point2D(0,0);
		mPoint2 = Point2D(0,0);
	}
	Segment(const Point2D& p1, const Point2D& p2)
	{
		ASSERT(init(p1,p2));
	}
	bool init(const Point2D& p1, const Point2D& p2)
	{
		if( LESS(p1.x , p2.x) || (EQUAL(p1.x, p2.x)&& LESS(p1.y , p2.y)))
		{
			mPoint1 = p1;
			mPoint2 = p2;
		}
		else
		{
			mPoint1 = p2;
			mPoint2 = p1;
		}
		mSlope.make(mPoint1,mPoint2);
		return true;
	}
	bool isLeftEnd(const Point2D& p) const
	{
		return (EQUAL(mPoint1.x,p.x) && EQUAL(mPoint1.y,p.y));
	}
	bool isRightEnd(const Point2D& p) const
	{
		return (EQUAL(mPoint2.x,p.x) && EQUAL(mPoint2.y,p.y));
	}
	bool isOnSegment(const Point2D& p) const
	{//设点p在线段上
		return (!isLeftEnd(p)) && (!isRightEnd(p));
	}
	bool less(const Segment& s,const Point2D& p)const
	{
		Real h1 = height(p), h2 = s.height(p);
		return (LESS(h1 , h2) || ((EQUAL(h1, h2) && mSlope < s.mSlope)));
	}
	Real height(const Point2D& p)const
	{
		if( mSlope.getType()==SLOPE_INFINITY )
		{
			Real py = p.y;
			Real ly = mPoint1.y;
			Real ry = mPoint2.y;
			if(LESS(py , ly))
				return ly;
			else if(LESS(ry , py))
				return ry;
			return py;
		}

		return (mPoint2.y - mPoint1.y)*(p.x - mPoint1.x)/(mPoint2.x - mPoint1.x) + mPoint1.y;
	}
	bool intersect(const Segment& segment,Point2D& result)const
	{
		Vector2D direction1 = mPoint2 - mPoint1;
		Vector2D direction2 = segment.mPoint2 - segment.mPoint1;

		Real dir1Square = direction1.magnitudeSquared();
		Real dir2Square = direction2.magnitudeSquared();

		Real kcross = direction1.cross(direction2);
		Vector2D e = segment.mPoint1 - mPoint1;

		if( (kcross*kcross) > EPS * EPS * dir1Square * dir2Square  )
		{//The intersection angle is not 0 based on relative error.||Cross(d1,d2)||^2/ (||d1||^2*||d2||^2)<=sin(a).
			Real s = e.cross(direction2) / kcross;
			if(LESS(s, 0) || GREATER(s, 1))
				return false;
			Real t = e.cross(direction1) / kcross;
			if(LESS(t , 0) || GREATER(t, 1))
				return false;
			result = mPoint1 + s*direction1;

			return true;
		}
		return false;
	}
	Point2D getLeftPoint()const
	{
		return mPoint1;
	}
	Point2D getRightPoint()const
	{
		return mPoint2;
	}
};

class Event
{
private:
	Point2D	mPoint;
public:
	Event()
	{
		mPoint = Point2D(0,0);
	}
	Event(const Point2D& p)
	{
		mPoint = p;
	}
	Point2D getPoint()const
	{
		return mPoint;
	}
	bool operator < (const Event& b)const
	{
		if( EQUAL(mPoint.x, b.mPoint.x) )
			return LESS(mPoint.y, b.mPoint.y);
		return LESS(mPoint.x,b.mPoint.x);
	}
};


struct sCompare
{
	Point2D* mComp;
	sCompare(Point2D* p):mComp(p){}
	bool operator()(Segment*a, Segment*b)
	{
		bool result = a->less(*b,*mComp);
		return a->less(*b,*mComp);
	}
};

//存储线段的平衡二叉树
typedef set<Segment*,sCompare>		SegBST;
typedef SegBST::iterator			SegBSTIterator;

typedef vector<Segment*>			SegVec;
typedef SegVec::iterator			SegVecIterator;

typedef map<Event, vector<Segment*> >	PriorityQueue;
typedef PriorityQueue::iterator			PriorityQueueIterator;

class EventQueue:public PriorityQueue
{
	typedef pair<typename PriorityQueue::iterator,bool> InsertStatus;
	typedef pair<Event,SegVec > EventType;
public:
	void push(const Event& key)
	{
		SegVec v;
		insert(EventType(key,v));
	}
	void push(const Event& key, Segment* value)
	{
		SegVec v;
		v.push_back(value);
		InsertStatus status = insert(EventType(key,v));
		if( !status.second )
			status.first->second.push_back(value);
	}
	Event topEvent()
	{
		return begin()->first;
	}
	SegVec& topSegVec()
	{
		return begin()->second;
	}
	void pop()
	{
		erase(begin());
	}
	bool isExist(const Event& key)const
	{
		return find(key) != end();
	}
};

void GetLeftInSets(const Point2D& p, SegBST& tree, SegVec& rightSet, SegVec& inSet)
{
	if( tree.empty() )
		return;
	Segment s(p,p);
	SegBSTIterator it = tree.upper_bound(&s);
	SegBSTIterator tmpBSTIt;
	it = tree.upper_bound(&s);
	reverse_iterator<SegBSTIterator> rit(it);

	while( rit != tree.rend() && fabs((*rit)->height(p) - p.y) < EPS )
	{
		if( (*rit)->isOnSegment(p) )
			inSet.push_back(*rit);
		else if( (*rit)->isRightEnd(p) )
			rightSet.push_back(*rit);
		rit ++;
	}
}

void FindNeighbors(const Point2D&p,SegBST& btree,Segment*& above,Segment*& below)
{
	Segment s(p,p);
	SegBSTIterator it = btree.upper_bound(&s);

	if( it==btree.end() )
		above = NULL;
	else
		above = *it;
	if( it==btree.begin() )
		below = NULL;
	else
		below = *--it;
}


void ComputeNewEvents(Segment* s0,Segment* s1,const Event& p,EventQueue& q)
{
	Point2D inter;
	if( s0&&s1&&s1->intersect(*s0,inter) )
	{
		Event evt(inter);
		const Point2D point = p.getPoint();

		if( p < evt && !q.isExist(evt) )
		{
			q.push(evt);
		}
	}
}

void FindEndSeg(SegVec& v , const Point2D& p,Segment*&min, Segment*& max)
{
	SegVecIterator it = v.begin();
	min = *v.begin();
	max = min;
	it ++;
	while( it != v.end() )
	{
		if( (*it)->less(*min,p) )
			min = *it;
		if( max->less(**it,p) )
			max = *it;
		it++;
	}
}

Segment* FindUpperNeighbor(Segment* s, SegBST& btree)
{
	//printf("findUpperNeighbor\n");
	SegBSTIterator it = btree.find(s);
	++it;
	if( it==btree.end())
		return NULL;
	else
		return *it;
}

Segment* FindLowerNeighbor(Segment* s, SegBST& btree)
{
	//printf("findLowerNeighbor");
	SegBSTIterator it = btree.find(s);
	if( it==btree.begin() )
		return NULL;
	else
		return *(--it);
}

void SegVecUnion(SegVecIterator& begin,SegVecIterator& last,SegVecIterator& result)
{
	SegVecIterator it;
	for( it = begin ; it!=last; it++,result++ )
	{
		*result = *it;
	}
}

void BentleyOttmannIntersection(const Segment* segs,int numSegs)
{
	//把线段的端点信息存入优先队列中
	EventQueue evtQueue;
	int i;
	for( i=0 ; i<numSegs ; i++ )
	{
		const Point2D l = segs[i].getLeftPoint();
		const Point2D r = segs[i].getRightPoint();
		Event evtLeft(l);
		Event evtRight(r);
		evtQueue.push(evtLeft,(Segment*const)(segs + i));
		evtQueue.push(evtRight);
	}
	//初始化存储线段的平衡二叉树
	Point2D swp(0,0);
	sCompare compare(&swp);
	SegBST segBSTree(compare);
	int step = 0;
	int count = 0;

	while(!evtQueue.empty())
	{
		step += 1;
		printf("第%d次迭代\n",step);
		Event event		= evtQueue.topEvent();
		SegVec leftSet	= evtQueue.topSegVec();
		evtQueue.pop();

		Point2D tmpPoint = swp;

		Point2D evtPoint	= event.getPoint();
		//更新扫描线
		swp.set(evtPoint.x,evtPoint.y);
		printf("swp");

		SegVec rightSet,inSet;
		GetLeftInSets(evtPoint,segBSTree,rightSet,inSet);
		
		int lSize = leftSet.size();
		int rSize = rightSet.size();
		int iSize = inSet.size();
		if(lSize + rSize + iSize > 1)
		{
			printf("(%f,%f)\n",evtPoint.x,evtPoint.y);
		}

		SegBSTIterator tmpBSTIt;
		SegVecIterator tmpSegIt;

		swp = tmpPoint;
		//移除rightSet和inSet
		SegVecIterator segIt;
		for( segIt = rightSet.begin() ; segIt!=rightSet.end() ; segIt++ )
			segBSTree.erase(*segIt);
		for( segIt = inSet.begin() ; segIt!=inSet.end() ; segIt++ )
			segBSTree.erase(*segIt);

		

		swp.set(evtPoint.x,evtPoint.y);
		//插入leftSet和inSet中的线段集
		for( segIt = leftSet.begin() ; segIt!=leftSet.end() ; segIt++ )
			segBSTree.insert(*segIt);
		for( segIt = inSet.begin() ; segIt!=inSet.end() ; segIt++ )
			segBSTree.insert(*segIt);

		if( lSize + iSize==0 )
		{
			Segment* sa,*sb;
			FindNeighbors(evtPoint,segBSTree,sa,sb);
			ComputeNewEvents(sa,sb,event,evtQueue);
		}
		else
		{
			SegVec v(lSize + iSize);
			SegVecUnion(leftSet.begin(),leftSet.end(),v.begin());
			SegVecUnion(inSet.begin(),inSet.end(),v.begin());
			Segment* bot,*top;
			FindEndSeg(v,evtPoint,bot,top);
			Segment* lowerBot = FindLowerNeighbor(bot,segBSTree);
			Segment* upperTop = FindUpperNeighbor(top,segBSTree);
			ComputeNewEvents(bot,lowerBot,event,evtQueue);
			ComputeNewEvents(top,upperTop,event,evtQueue);
		}
	}
}

void TestTentley()
{
	int num = 10;
	Segment* segs = new Segment[num];
	Point2D p1, p2;

	p1.x = 2; p1.y = 1; p2.x = 3; p2.y = 7;
	segs[0].init(p1,p2);
	p1.x = 3; p1.y = 7; p2.x = 6;p2.y = 1;
	segs[1].init(p1,p2);
	p1.x = 4; p1.y = 0; p2.x = 6;p2.y = 1;
	segs[2].init(p1,p2);
	p1.x = 2; p1.y = 1; p2.x = 4;p2.y = 0;
	segs[3].init(p1,p2);
	p1.x = 2; p1.y = 1; p2.x = 6;p2.y = 10;
	segs[4].init(p1,p2);
	p1.x = 1; p1.y = 2; p2.x = 10;p2.y = 10;
	segs[5].init(p1,p2);
	p1.x = 6; p1.y = 1; p2.x = 6;p2.y = 10;
	segs[6].init(p1,p2);

	BentleyOttmannIntersection(segs,7);

	delete []segs;
}

int main ()
{
	TestTentley();
	return 0;
}