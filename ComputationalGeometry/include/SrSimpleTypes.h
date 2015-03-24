/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2013/11/22
****************************************************************************/
#ifndef SR_FOUNDATION_SIMPLETYPES_H_
#define SR_FOUNDATION_SIMPLETYPES_H_
/** \addtogroup foundation
  @{
*/

typedef signed int			SrI32;
typedef signed short		SrI16;
typedef signed char			SrI8;

typedef unsigned int		SrU32;
typedef unsigned short		SrU16;


typedef float				SrF32;
typedef double				SrF64;

typedef double				SrReal;


// Type ranges
#define	SR_MAX_I8			0x7f			//max possible sbyte value
#define	SR_MIN_I8			0x80			//min possible sbyte value
#define	SR_MAX_U8			0xff			//max possible ubyte value
#define	SR_MIN_U8			0x00			//min possible ubyte value
#define	SR_MAX_I16			0x7fff			//max possible sword value
#define	SR_MIN_I16			0x8000			//min possible sword value
#define	SR_MAX_U16			0xffff			//max possible uword value
#define	SR_MIN_U16			0x0000			//min possible uword value
#define	SR_MAX_I32			0x7fffffff		//max possible sdword value
#define	SR_MIN_I32			0x80000000		//min possible sdword value
#define	SR_MAX_U32			0xffffffff		//max possible udword value
#define	SR_MIN_U32			0x00000000		//min possible udword value
#define	SR_MAX_F32			3.402823466e+38F			//max possible float value
#define	SR_MIN_F32			(-3.402823466e+38F)		//min possible float value
#define	SR_MAX_F64			1.7976931348623158e+308			//max possible double value
#define	SR_MIN_F64			(-1.7976931348623158e+308)		//min possible double value

#define SR_EPS_F32			FLT_EPSILON		//smallest number not zero
#define SR_EPS_F64			DBL_EPSILON		//smallest number not zero


#define SR_INLINE			inline




/** @} */
#endif
