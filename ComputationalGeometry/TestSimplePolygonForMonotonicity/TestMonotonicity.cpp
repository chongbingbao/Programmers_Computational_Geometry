/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/11/10
****************************************************************************/
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

#include "SrGeometricTools.h"
#include "SrDataType.h"

#include <math.h>
#include <stdio.h>
#include <list>
using namespace std;

#define Real		SrReal
#define Point2D		SrPoint2D
#define Vector2D	SrVector2D

#define EPS				SR_EPS
#define PI				SrPiF64

#define	HALF_CIRCLE		(PI)
#define FULL_CIRCLE		(2 * PI)


//a0 到 a1逆时针方向上的夹角
#define CCW_ANGLE(a0,a1)	(((a1)-(a0) < 0)?(a1) - (a0) + FULL_CIRCLE:(a1) - (a0))

//a0 到 a1逆时针方向上的夹角
Real	CCWAngle(Real a0, Real a1)
{
	if(a1 - a0 < 0)	return a1 - a0 + FULL_CIRCLE;
	return a1 - a0;
}

Real	ClampAngle(Real angle)
{
	if(angle < 0)			return angle + FULL_CIRCLE;
	if(angle >= FULL_CIRCLE) return angle - FULL_CIRCLE;
	return angle;
}

Real	ComputeAngle(const Vector2D& v0 , const Vector2D& v1)
{
	Real len0 = v0.magnitude();
	Real len1 = v1.magnitude();
	Real angle = asin(v0.cross(v1) / (len0 * len1));
	if(v0.dot(v1) > 0)	return angle;
	if(angle > 0)		return HALF_CIRCLE - angle;
	return -(HALF_CIRCLE + angle);
}

// 若a0在a1的顺时针方向上，return -1;
// 若a0在a1的逆时针方向上，return 1;
int IsCCWAngle(Real a0 , Real a1)
{
	Real span = CCWAngle(a0,a1);
	if(span < HALF_CIRCLE)	return -1;
	if(span > HALF_CIRCLE)	return 1;
	return 0;
}

class Node
{
public:
	Node()
	{
		mNext	= NULL;
		mPre	= NULL;
		mAngle	= 0;
		mCount	= 0;
	}
	Node* next()
	{
		return mNext;
	}
	Node* pre()
	{
		return mPre;
	}
	void erase()
	{
		mPre->mNext = mNext;
		mNext->mPre = mPre;
	}
	void insert_back(Node* node)
	{
		node->mNext = mNext;
		node->mPre	= this;
		mNext->mPre = node;
		mNext		= node;
	}
	void insert_front(Node* node)
	{
		node->mNext = this;
		node->mPre  = this->mPre;
		mPre->mNext = node;
		mPre		= node;
	}
	void setNext(Node* node)
	{
		mNext = node;
	}
	void setPre(Node* node)
	{
		mPre = node;
	}
public:
	Real	mAngle;
	int		mCount;
private:
	Node*	mNext;
	Node*	mPre;
};


void Print(Node* head)
{
	Node* current = head;
	while(current->mAngle != 0)
		current = current->next();
	Node* t = current;
	do 
	{
		printf("(%.2f,%d),  ",current->mAngle * 180.0 / HALF_CIRCLE,current->mCount);
		current = current->next();
	} while (current != t);
	printf("\n");
}


/*
\param[in]  head	双向循环链表中的节点，它的角度值小于等于polar
\param[in]	polar	介于[0,2*PI)，表示极角坐标
\param[in]  angle	介于(0,PI)，表示夹角
\return		返回链表中的一个节点，该节点的角度值小于等于polar
*/
Node* InsertCCW(Node* head, Real  polar, Real angle)
{
	Node* pCurr, *node;
	Real t = angle + polar;
	pCurr = head;
	if(t > FULL_CIRCLE)
	{
		pCurr = InsertCCW(pCurr,polar,FULL_CIRCLE - polar);
		pCurr = pCurr->next();
		pCurr = InsertCCW(pCurr,0,t - FULL_CIRCLE);
		node = pCurr;
	}
	else
	{	
		printf("			%f\n",head->mAngle * 180.0 / HALF_CIRCLE);
		Node* pNext = pCurr->next();		
		while(pNext->mAngle < t)
		{
			pCurr->mCount = (pCurr->mCount < 2 ? pCurr->mCount + 1 : 2);
			pCurr = pNext;
			pNext = pCurr->next();
		}
		if(fabs(pNext->mAngle - t) >= EPS )
		{
			if(pCurr->mCount < 2)
			{
				node = new Node();
				node->mCount = pCurr->mCount;
				node->mAngle = t;

				pCurr->mCount = (pCurr->mCount < 2 ? pCurr->mCount + 1 : 2);
				pCurr->insert_back(node);
			}
			else
			{
				node = pCurr;
			}
		}
		else
		{
			pCurr->mCount = (pCurr->mCount < 2 ? pCurr->mCount + 1 : 2);
			node = pNext;
		}
		//合并计数器值相同的数
		if( head != node)
		{
			pCurr = head;
			pNext = pCurr->next();
			while(pNext != node)
			{
				Node* tmp = pNext->next();
				if(pCurr->mCount == pNext->mCount)
				{
					pNext->erase();
					delete pNext;
				}
				pNext = tmp;
			}
		}
		printf("Print The List\n");
		Print(node);
	}
	return node;
}

/*
\param[in]  head	节点的角度值小于等于angle
\param[in]	polar	介于[0,2*PI)，表示极角坐标
\param[in]  angle	介于(0,PI)，表示夹角
\return		返回角度值小于等于angle的节点
*/
Node* InsertCW(Node* head, Real  polar, Real angle)
{

	Node* pCurr, *node;
	Real sum = angle + polar;

	pCurr = head;
	if(sum < 0)
	{
		if(fabs(polar) >= EPS)		// polar != 0
			pCurr = InsertCW(pCurr,polar,-polar);
		pCurr = pCurr->pre();
		node = InsertCW(pCurr,FULL_CIRCLE, sum);

	}
	else
	{
		Node* pNext = NULL;
		Node* pPre;
		while(pCurr->mAngle > sum)
		{
			if(pNext != NULL)
				pCurr->mCount = (pCurr->mCount < 2 ? pCurr->mCount + 1 : 2);
			pNext = pCurr;
			pCurr = pCurr->pre();
		}
		if(fabs(pCurr->mAngle - sum) >= EPS )
		{//pCurr->mAngle一定不为0
			if(pCurr->mCount < 2)
			{
				node = new Node();
				node->mAngle = sum;
				node->mCount = (pCurr->mCount < 2 ? pCurr->mCount + 1 : 2);

				pCurr->insert_back(node);
			}
			else
			{
				node = pCurr;
			}
		}
		else
		{
			pCurr->mCount = (pCurr->mCount < 2 ? pCurr->mCount + 1 : 2);
			node = pCurr;
		}
		if(head != node)
		{
			pCurr = head;
			pPre  = pCurr->pre();
			while( pPre != node )
			{
				if( pCurr->mCount == pPre->mCount && pCurr->mAngle != 0)
				{
					pCurr->erase();
					delete pCurr;
				}
				pCurr = pPre;
				pPre  = pCurr->pre();
			}
		}
		printf("Print The List\n");
		Print(node);
	}
	return node;
}


void Deallocate(Node* head)
{
	Node* current, *next;
	current = head;
	current = current->next();
	while (current != head)
	{
		next = current->next();
		delete current;
		current = next;
	}
	delete head;
}

bool IsMonotone(Node* head)
{
	printf("Is Monotone.\n");
	Node* current = head;
	bool flag = false;
	int i = 0;
	do 
	{
		if(current->mCount == 1)
			flag = true;;
		printf("%d(%.f,%d)  ", ++i, current->mAngle * 180.0 / HALF_CIRCLE, current->mCount);
		current = current->next();
	} while (current != head);
	printf("\n");

	return flag;
}

bool CheckMonotone(const Point2D* V, int n)
{
	int i;
	Real a = 0, angle = 0;
	Vector2D e0, e1;
	Node*	head;
	head = new Node();
	head->mAngle = 0;
	head->mCount = 0;
	head->setNext(head);
	head->setPre(head);

	Node*	tail;
	tail = new Node();
	tail->mAngle = FULL_CIRCLE;
	tail->mCount = 0;
	head->insert_back(tail);

	Node *curr = tail;

	for( i = 0 ; i < n ; ++i )
	{
		e0	  = V[(i + 1) % n] - V[i];
		e1	  = V[(i + 2) % n] - V[(i +1) % n];
		a     = ComputeAngle(e0,e1);
//		printf("%d	%.f,%.f\n",i,a*180.0/HALF_CIRCLE,angle*180.0/HALF_CIRCLE);
		if(a > 0)
		{
			if(curr->mAngle == FULL_CIRCLE)
				curr = curr->next();
			curr = InsertCCW(curr,angle,a);
		}
		else if(a < 0)
		{
			curr = InsertCW(curr,angle,a);
		}
		angle = ClampAngle(a + angle);
	}
	bool r = IsMonotone(head);

	Deallocate(head);

	return r;
	return true;
}

typedef list<Point2D>		PointList;
typedef PointList::iterator	PointListItertor;	

int Compare(const Point2D& p0 , const Point2D& p1, const Vector2D& d)
{
	Real d0, d1, c0, c1;
	d0 = d.dot(p0);
	d1 = d.dot(p1);
	if(d0 < d1)	return  1;
	if(d0 > d1)	return -1;
	c0 = d.cross(p0);
	c1 = d.cross(p1);
	if(c0 < c1)	return 1;
	if(c0 > c1) return -1;
	return 0;
}

void HeapSort(Point2D * points, int numPoint , const Vector2D& d)
{
	Point2D* heap = new Point2D[numPoint + 1];
	int i , j , n = numPoint , child;
	for( i = 0 ; i < numPoint ; i ++ ) 
		heap[i + 1] = points[i];
	//build a minimum heap.
	Point2D y;
	for(i=numPoint/2 ; i>=1; i--)
	{
		y = heap[i];
		child = 2 * i;
		while(child <= numPoint)
		{
			if(child < numPoint && Compare(heap[child],heap[child + 1], d) < 0 )	/*heap[child]>heap[child+1]*/
				child++;
			if(Compare(y,heap[child],d) >= 0)
				break;
			heap[child / 2] = heap[child];
			child *= 2;
		}
		heap[child / 2]=y;
	}
	//Pop the head of heap and update the minimum heap.
	int indx = 0;
	for(j = 1;j <= numPoint ; j ++)
	{
		y = heap[n--];
		points[indx++] = heap[1];
		i = 1,child = 2;
		while(child <= n)
		{
			if(child < n && Compare(heap[child],heap[child + 1], d) < 0)
				child++;
			if(Compare(y,heap[child],d) >= 0)
				break;
			heap[i] = heap[child];
			i = child;
			child *= 2;
		}
		heap[i] = y;
	}
	delete []heap;
}

/*
\brief	根据给定点集，生成沿着d方向的单调多边形，设单调多边形是逆时针顺序的，
		生成的单调多边形可能出现多点共线的情况
\param[in]	p, n		规定输入的点集
\param[in]	d			规定直线的方向
\param[out] q, m		输出的单调多边形
*/
void GenMonotonePolygon(const Point2D* p, int n, const Vector2D& d, Point2D*& q, int &m)
{
	int i;
	Point2D* P = new Point2D[n];
	for( i = 0 ; i < n ; ++ i )
		P[i] = p[i];

	//对点集排序
	HeapSort(P,n,d);

	//排除重复点
	int k = 1;
	//Real s0 = d.dot(P[0]);
	//Real s1 = 0;
	for( i = 1 ; i < n ; ++ i )
		if(P[i] != P[i-1] )
			P[k++] = P[i];

	//对排完序的点集，按照线段P[0]-P[k-1]，分成上、下两个点集
	Point2D begin = P[0];
	Point2D end	  = P[k - 1];
	Vector2D b	  = P[k - 1] - P[0];
	PointList top, bott;
	for( i = 1 ; i < k - 1 ; ++ i )
	{
		if(P[0] == P[n - 1])
			continue;
		Real s = b.cross(P[i] - P[0]);
		if(s > 0)
			top.push_front(P[i]);
		else if(s < 0)
			bott.push_back(P[i]);
	}
	//归并上、下两个点集，得到单调多边形
	PointListItertor it;
	m = top.size() + bott.size() + 2;
	q = new Point2D[m];

	q[0] = P[0];
	for( i = 1, it = bott.begin() ; it != bott.end() ; ++ it, ++ i )
		q[i] = *it;

	q[i++] = P[n-1];
	for( it = top.begin() ; it != top.end() ; ++ it, ++ i )
		q[i] = *it;
	delete []P;
}

void Test()
{
	int n = 1000,m, i;
	Point2D* p,* q;
	Vector2D d(1,1);

	p = new Point2D[n];
	for( i = 0 ; i < n ; ++ i )
	{
		p[i].x = rand() % 500;
		p[i].y = rand() % 500;
	}
	GenMonotonePolygon(p,n,d,q,m);

	printf("%d\n",CheckMonotone(q,m));
	delete []p;
	delete []q;
}

int main()
{
	Test();
	_CrtDumpMemoryLeaks();
	return 0;
}