/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/04/11
****************************************************************************/
#ifndef SR_GEOMETRIC_TOOLS_H_
#define SR_GEOMETRIC_TOOLS_H_
/** \addtogroup foundation
  @{
*/
#include <float.h>


#define SR_X_AXIS						0x01
#define SR_Y_AXIS						0x02
#define SR_Z_AXIS						0x03

#define SR_PLANE_BACK					0x01
#define SR_PLANE_FRONT					0x02
#define SR_PLANE_INTERSECTING			0x03
#define SR_PLANE_TANGENT				0x04

#define SR_INVALID						0x00
#define SR_DISJOINT						0x01
#define SR_OVERLAPPING					0x02
#define SR_PARALLEL						0x03
#define SR_INTERSECTING					0x04
#define SR_INTERSECTING_EDGE			0x05

#define SR_POSITION_INVALID				0x00
#define SR_POSITION_ON					0x01
#define SR_POSITION_INSIDE				0x02
#define SR_POSITION_OUTSIDE				0x03

#define SR_LINEAR_ON					0x01
#define SR_LINEAR_ABSENT				0x02


#define  SR_EPS_32	0.000001f
#define	 SR_EPS		0.000000000001



#ifdef _DEBUG
#define  ASSERT(exp) assert((exp))
#else
#define ASSERT(exp)
#endif

#define EQUAL(exp1,exp2)	(fabs((exp1)-(exp2))<SR_EPS)
#define LEQUAL(exp1,exp2)	((exp1)-(exp2)<SR_EPS)
#define LESS(exp1,exp2)		((exp1)-(exp2)<=-SR_EPS)
#define GREATER(exp1,exp2)	((exp1)-(exp2)>=SR_EPS)
#define GEQUAL(exp1,exp2)	((exp1)-(exp2)>-SR_EPS)
#define UNEQUAL(exp1,exp2)	(fabs((exp1)-(exp2))>=SR_EPS)

#define ANGLE_EQUAL_ZERO(angle,squareLen1,squareLen2) ((angle)*(angle) < SR_EPS*SR_EPS*(squareLen1)*(squareLen2))
#define ANGLE_LEQUAL_ZERO(angle,squareLen1,squareLen2) ((angle)<=-SR_EPS || ANGLE_EQUAL_ZERO(angle,squareLen1,squareLen2))
#define ANGLE_GEQUAL_ZERO(angle,squareLen1,squareLen2) ((angle)>=SR_EPS || ANGLE_EQUAL_ZERO(angle,squareLen1,squareLen2))

/** @} */
#endif