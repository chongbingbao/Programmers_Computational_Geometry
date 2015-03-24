/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/05/15
****************************************************************************/
#include "../include/SrVector3.h"
#include "../include/SrGeometricTools.h"
#include "../include/SrDataType.h"

#include <time.h>
#include <stdio.h>

const int INSIDE = 0; // 0000
const int LEFT = 1;   // 0001
const int RIGHT = 2;  // 0010
const int BOTTOM = 4; // 0100
const int TOP = 8;    // 1000



// Compute the bit code for a point (x, y) using the clip rectangle
// bounded diagonally by (xMin, yMin), and (xMax, yMax)
int computeOutCode(const SrPoint2D& p,const SrVector2D& minW,const SrVector2D& maxW)
{
	int code;
	code = INSIDE;				 // initialised as being inside of clip window
	if ( LESS(p.x,minW.x) )           // to the left of clip window
		code |= LEFT;
	else if ( GREATER(p.x,maxW.x))      // to the right of clip window
		code |= RIGHT;
	if ( LESS(p.y , minW.y) )           // below the clip window
		code |= BOTTOM;
	else if ( GREATER(p.y , maxW.y) )      // above the clip window
		code |= TOP;
	return code;
}

// Cohen¨CSutherland clipping algorithm clips a line from
// P0 = (x0, y0) to P1 = (x1, y1) against a rectangle with 
// diagonal from (xMin, yMin) to (xMax, yMax).
bool cohenSutherland(const SrPoint2D& inP1, const SrPoint2D& inP2,
					 const SrVector2D& minRect,const SrVector2D& maxRect,
					 SrPoint2D& resultP1,SrPoint2D& resultP2)
{

	if( (LESS(inP1.x,minRect.x)&&LESS(inP2.x,minRect.x))||
		(GREATER(inP1.x,maxRect.x)&&GREATER(inP2.x,maxRect.x))||
		(LESS(inP1.y,minRect.y)&&LESS(inP2.y,minRect.y))||
		(GREATER(inP1.y,maxRect.y)&&GREATER(inP2.y,maxRect.y)) )
		return false;

	SrPoint2D p1 = inP1;
	SrPoint2D p2 = inP2;
	// compute outcodes for P0, P1, and whatever point lies outside the clip rectangle
	int outcode0 = computeOutCode(p1,minRect,maxRect);
	int outcode1 = computeOutCode(p2,minRect,maxRect);
	bool accept = false;

	while (true) 
	{
		if (!(outcode0 | outcode1)) 
		{ // Bitwise OR is 0. Trivially accept and get out of loop
			accept = true;
			break;
		} 
		else if (outcode0 & outcode1) 
		{ // Bitwise AND is not 0. Trivially reject and get out of loop
			break;
		} 
		else 
		{
			// failed both tests, so calculate the line segment to clip
			// from an outside point to an intersection with clip edge
			SrReal x, y;
			// At least one endpoint is outside the clip rectangle; pick it.
			int outcodeOut = outcode0 ? outcode0 : outcode1;
			// Now find the intersection point;
			// use formulas y = y0 + slope * (x - x0), x = x0 + (1 / slope) * (y - y0)
			if (outcodeOut & TOP) 
			{           // point is above the clip rectangle
				x = p1.x + (p2.x - p1.x) * (maxRect.y - p1.y) / (p2.y - p1.y);
				y = maxRect.y;
			} 
			else if (outcodeOut & BOTTOM) 
			{ // point is below the clip rectangle
				x = p1.x + (p2.x - p1.x) * (minRect.y - p1.y) / (p2.y - p1.y);
				y = minRect.y;
			} 
			else if (outcodeOut & RIGHT) 
			{  // point is to the right of clip rectangle
				y = p1.y + (p2.y - p1.y) * (maxRect.x - p1.x) / (p2.x - p1.x);
				x = maxRect.x;
			} 
			else if (outcodeOut & LEFT) 
			{   // point is to the left of clip rectangle
				y = p1.y + (p2.y - p1.y) * (minRect.x - p1.x) / (p2.x - p1.x);
				x = minRect.x;
			}
			// Now we move outside point to intersection point to clip
			// and get ready for next pass.
			if (outcodeOut == outcode0) 
			{
				p1.x = x;
				p1.y = y;
				outcode0 = computeOutCode(p1,minRect,maxRect);
			} 
			else 
			{
				p2.x = x;
				p2.y = y;
				outcode1 = computeOutCode(p2,minRect,maxRect);
			}
		}
	}
	if (accept) 
	{
		resultP1 = p1;
		resultP2 = p2;
		return true;
	}
	return false;
}

bool clip(SrReal p,SrReal q,SrReal& t0,SrReal& t1)
{
	SrReal r;
	if( LESS(p,0) && LESS(q,0) )
	{
		r = q/p;
		if( GREATER(r,t1) )
			return false;
		else if( GREATER(r,t0) )
			t0 = r;
	}
	else if( GREATER(p,0) && LESS(q,p) )
	{
		r = q/p;
		if( LESS(r,t0) )
			return false;
		else if( LESS(r,t1) )
			t1 = r;
	}
	else if( LESS(q,0) )
		return false;
	return true;
}

// Liang-Barsky clipping algorithm clips a line from
// P0 = (x0, y0) to P1 = (x1, y1) against a rectangle with 
// diagonal from (xMin, yMin) to (xMax, yMax).
bool LiangBarsky(const SrPoint2D& inP1, const SrPoint2D& inP2,const SrVector2D& minRect,const SrVector2D& maxRect,SrPoint2D& resultP1,SrPoint2D& resultP2)
{
	if( (LESS(inP1.x,minRect.x)&&LESS(inP2.x,minRect.x))||
		(GREATER(inP1.x,maxRect.x)&&GREATER(inP2.x,maxRect.x))||
		(LESS(inP1.y,minRect.y)&&LESS(inP2.y,minRect.y))||
		(GREATER(inP1.y,maxRect.y)&&GREATER(inP2.y,maxRect.y)) )
		return false;

	SrReal t0, t1;
	t0 = 0.0;
	t1 = 1.0;
	SrReal deltaX , deltaY;

	deltaX = inP2.x - inP1.x;
	if( clip(-deltaX,inP1.x - minRect.x,t0,t1) && clip(deltaX,maxRect.x - inP1.x,t0,t1))
	{
		deltaY = inP2.y - inP1.y;
		if( clip(-deltaY,inP1.y - minRect.y,t0,t1) && clip(deltaY,maxRect.y - inP1.y,t0,t1) )
		{
			if( LEQUAL(t1,1) )
			{
				resultP2.x = inP1.x + t1*deltaX;
				resultP2.y = inP1.y + t1*deltaY;
			}
			if( GEQUAL(t0,0) )
			{
				resultP1.x = inP1.x + t0*deltaX;
				resultP1.y = inP1.y + t0*deltaY;
			}
			return true;
		}
	}
	return false;
}

void testClipSegment()
{
	int numPoint = 100000, range = 100;
	SrPoint2D* point1,*point2;
	point1 = new SrPoint2D[numPoint];
	point2 = new SrPoint2D[numPoint];
	int i;
	for( i=0 ; i<numPoint ; i++ )
	{
		do 
		{
			point1[i].x = rand()%range;
			point1[i].y = rand()%range;

			point2[i].x = rand()%range;
			point2[i].y = rand()%range;
		} while ((EQUAL(point1[i].x,point2[i].x)&&EQUAL(point1[i].y,point2[i].y)));
	}
	SrPoint2D res1[2] , res2[2];
	SrPoint2D minRect = SrPoint2D(20,20);
	SrPoint2D maxRect = SrPoint2D(50,50);
	/*for( i=0 ; i<numPoint ; i++ )
	{
		bool status1 = cohenSutherland(point1[i],point2[i],minRect,maxRect,res1[0],res1[1]);
		bool status2 = LiangBarsky(point1[i],point2[i],minRect,maxRect,res2[0],res2[1]);
		if( status1 != status2 )
		{
			int t ;
			t = 10;
			cohenSutherland(point1[i],point2[i],minRect,maxRect,res1[0],res1[1]);
			LiangBarsky(point1[i],point2[i],minRect,maxRect,res2[0],res2[1]);
		}
		ASSERT(EQUAL(res1[0].x,res2[0].x)&&EQUAL(res1[0].y,res2[0].y));
		ASSERT(EQUAL(res1[1].x,res2[1].x)&&EQUAL(res1[1].y,res2[1].y));
		ASSERT(status1==status2);
	}*/

	double seconds = clock();
	for( i=0 ; i<numPoint ; i++ )
	{
		cohenSutherland(point1[i],point2[i],minRect,maxRect,res1[0],res1[1]);
	}
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("%.6lf\n",seconds);

	seconds = clock();
	for( i=0 ; i<numPoint ; i++ )
	{
		LiangBarsky(point1[i],point2[i],minRect,maxRect,res1[0],res1[1]);
	}
	seconds = (clock() - seconds)/CLOCKS_PER_SEC;
	printf("%.6lf\n",seconds);

}

int main()
{

	testClipSegment();
	return 0;
}