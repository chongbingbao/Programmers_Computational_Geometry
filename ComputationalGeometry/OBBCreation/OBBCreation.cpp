/************************************************************************		
\link	www.twinklingstar.cn
\author Twinkling Star
\date	2014/06/15
****************************************************************************/
/************************************************************************		
\description	采用协方差矩阵的算法，近似估计给定点集的最小有向包围盒。
****************************************************************************/
#include "SrGeometricTools.h"
#include "SrDataType.h"
#include "QuickHull.h"
#include "gmm/gmm_dense_qr.h"
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
	SrOBBox3D(const SrPoint3D& c,const SrVector3D* pAxis,const float* pHalfLength)
	{
		mCenter = c;

		mAxis[0] = pAxis[0];
		mAxis[1] = pAxis[1];
		mAxis[2] = pAxis[2];

		mHalfLength[0] = pHalfLength[0];
		mHalfLength[1] = pHalfLength[1];
		mHalfLength[2] = pHalfLength[2];
	}

public:
	SrPoint3D	mCenter;
	SrVector3D	mAxis[3];
	SrReal		mHalfLength[3];
};


/*
\brief	采用协方差矩阵的算法，近似估计凸多面体的最小有向包围盒。
\param[in]	hull	存储凸多面体的数据结构，必需是有效凸多面体。
\return	SrOBBox3D对象	返回估计出的最小有向包围盒。
*/
const SrOBBox3D approximateSmallestOBB(tHull* hull)
{
	SrReal *		area = new SrReal[hull->mNumFacet];
	SrPoint3D*		mass = new SrPoint3D[hull->mNumFacet];
	SrVector3 normal;
	SrPoint3D p0,p1,p2,tmp;
	int i;
	for( i=0 ; i<hull->mNumFacet ; i++ )
	{
		p0 = hull->mVertes[hull->mFacet[i].mVInx[0]];
		p1 = hull->mVertes[hull->mFacet[i].mVInx[1]];
		p2 = hull->mVertes[hull->mFacet[i].mVInx[2]];

		normal = (p1 - p0).cross(p2 - p0);
		tmp = p0.cross(p1) + p1.cross(p2) + p2.cross(p0);

		area[i] = tmp.dot(normal)*(SrReal)0.5;
		mass[i] = (p0 + p1 + p2) / (SrReal)3.0;
	}

	SrReal		sumArea = 0.0;
	SrPoint3D	sumMass = SrPoint3D(0,0,0);
	for( i=0 ; i<hull->mNumFacet ; i++ )
	{
		sumArea += area[i];
		sumMass += area[i]*mass[i];
	}
	sumMass = sumMass/sumArea;
	SrPoint3D t = SrPoint3D(0,0,0);

	SrReal covarMatrx[3][3];
	int j , k;
	for( i=0 ; i<3 ; i++ )
	{
		for( j=i ; j<3 ; j++ )
		{
			covarMatrx[i][j] = covarMatrx[j][i] = 0;
			for( k=0 ; k<hull->mNumFacet ; k++ )
			{
				p0 = hull->mVertes[hull->mFacet[i].mVInx[0]];
				p1 = hull->mVertes[hull->mFacet[i].mVInx[1]];
				p2 = hull->mVertes[hull->mFacet[i].mVInx[2]];
				covarMatrx[i][j] += area[k] * (9*mass[k][i]*mass[k][j] + p0[i]*p0[j] + p1[i]*p1[j] + p2[i]*p2[j]);
			}
			covarMatrx[i][j] = sumArea * covarMatrx[i][j] / (SrReal)12.0 - sumMass[i]*sumMass[j];
			covarMatrx[j][i] = covarMatrx[i][j];
		}
	}

	gmm::dense_matrix<SrReal> cMatrix(3,3);

	cMatrix(0,0) = covarMatrx[0][0]; cMatrix(0,1) = covarMatrx[0][1]; cMatrix(0,2) = covarMatrx[0][2];
	cMatrix(1,0) = covarMatrx[1][0]; cMatrix(1,1) = covarMatrx[1][1]; cMatrix(1,2) = covarMatrx[1][2];
	cMatrix(2,0) = covarMatrx[2][0]; cMatrix(2,1) = covarMatrx[2][1]; cMatrix(2,2) = covarMatrx[2][2];

	//采用GMM++软件包中的对称QR算法，计算3*3的协方差矩阵的特征向量
	gmm::dense_matrix<SrReal>	eigvec(3,3);
	std::vector<SrReal>			eigval(3);
	gmm::symmetric_qr_algorithm( cMatrix, eigval, eigvec );

	SrVector3D x( eigvec(0,0), eigvec(1,0), eigvec(2,0) );
	SrVector3D y( eigvec(0,1), eigvec(1,1), eigvec(2,1) );
	SrVector3D z( eigvec(0,2), eigvec(1,2), eigvec(2,2) );
	x.normalize(); y.normalize();z.normalize();

	SrVector3D minLen , maxLen;
	SrVector3D tmpLen;
	minLen.set(x.dot(hull->mVertes[0]),y.dot(hull->mVertes[0]),z.dot(hull->mVertes[0]));
	maxLen = minLen;
	for( i = 1 ; i<hull->mNumVertes ; i++ )
	{
		tmpLen.set(x.dot(hull->mVertes[i]),y.dot(hull->mVertes[i]),z.dot(hull->mVertes[i]));
		minLen.min(tmpLen);
		maxLen.max(tmpLen);
	}


	SrOBBox3D obb;

	obb.mAxis[0] = x;
	obb.mAxis[1] = y;
	obb.mAxis[2] = z;

	obb.mHalfLength[0] = (maxLen.x - minLen.x)*(SrReal)0.5;
	obb.mHalfLength[1] = (maxLen.y - minLen.y)*(SrReal)0.5;
	obb.mHalfLength[2] = (maxLen.z - minLen.z)*(SrReal)0.5;

	obb.mCenter = ((minLen.x + maxLen.x)*(SrReal)0.5)*x + ((minLen.y + maxLen.y)*(SrReal)0.5)*y + ((minLen.z + maxLen.z)*(SrReal)0.5)*z;

	delete []area;
	delete []mass;

	return obb;
}


/************************************************************************		
							用OpenGL，显示创建的有向包围盒
****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>

bool		gMoveScene			= false;
bool		gStopTimer			= false;
bool		gDisplayMesh		= false;


void outputInfo()
{
	printf("************************************\n");
	printf("S:	Stop or start timer        *\n");
	printf("C:	Display the mesh or not    *\n");
	printf("************************************\n");
}



void timerMessage(int id)
{
	glutPostRedisplay();
	if( !gStopTimer )
		glutTimerFunc(200,timerMessage,0);
}

void setNormalLight( const float* lightPosition)
{
	float ambient_lightColor[] = {0.1f,0.1f,0.1f};
	float diffuse_lightColor[] = {0.5f,0.5f,0.5f};
	float specular_lightColor[] = {1.0f,1.0f,1.0f};

	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT1, GL_POSITION, lightPosition); 
	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient_lightColor);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse_lightColor);
	glLightfv(GL_LIGHT1, GL_SPECULAR,specular_lightColor);
}

void drawRectangle(const SrPoint3D& p0,const SrPoint3D& p1, const SrPoint3D& p2,const SrPoint3D& p3)
{
	glBegin(GL_LINE_STRIP);
	glVertex3f(p0.x,p0.y,p0.z);
	glVertex3f(p1.x,p1.y,p1.z);
	glVertex3f(p2.x,p2.y,p2.z);
	glVertex3f(p3.x,p3.y,p3.z);
	glEnd();
}

void drawOBB(const SrVector3* axis,const SrVector3& halfLength, const SrPoint3D& center)
{
	int i ;
	glColor3f(1.0f,1.0f,1.0f);
	SrVector3 ori , u , v;
	for( i=0 ; i<3 ; i++ )
	{
		ori = center + axis[i] * halfLength[i];
		u = axis[(i+1)%3] * halfLength[ (i+1)%3 ];
		v = axis[(i+2)%3] * halfLength[ (i+2)%3 ];
		drawRectangle(ori - u - v, ori + u - v, ori + u + v, ori - u + v);
		ori = center - axis[i] * halfLength[i];
		drawRectangle(ori - u - v, ori + u - v, ori + u + v, ori - u + v);
	}
}



void renderMessage( )
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	glTranslatef(-5.0f,-5.0f,-50.0f);

	float lightPosition[] ={0,0,0};
	setNormalLight(lightPosition);
	QuickHull hull3d;

	int numPoint = 100 , i;

	tHull gHull;
	SrPoint3D* point  = new SrPoint3D[numPoint];

	for( i=0 ; i<numPoint ; i++ )
	{
		point[i].x = (float)(rand() % 100)/10.0f;
		point[i].y = (float)(rand() % 100)/10.0f;
		point[i].z = (float)(rand() % 100)/10.0f;
	}

	//采用三维快速凸包算法，计算给定点集的凸多面体。
	if( hull3d.quickHull(point,numPoint,&gHull) )
	{
		int i , j;
		for( i=0 ; i<gHull.mNumFacet ; i++ )
		{
			if( !gDisplayMesh )
			{
				glColor3f((float)(rand() % 100)/100.0f,(float)(rand() % 100)/100.0f,(float)(rand() % 100)/100.0f);
				glBegin(GL_TRIANGLES);
				for( j=0 ; j<3 ; j++ )
				{

					glVertex3f( gHull.mVertes[gHull.mFacet[i].mVInx[j]].x,
						gHull.mVertes[gHull.mFacet[i].mVInx[j]].y,
						gHull.mVertes[gHull.mFacet[i].mVInx[j]].z);
				}

				glEnd();
			}
			else
			{
				glColor3f(1.0f,1.0f,1.0f);
				glBegin(GL_LINES);
				for( j=0 ; j<3 ; j++ )
				{

					glVertex3f( gHull.mVertes[gHull.mFacet[i].mVInx[j]].x,
						gHull.mVertes[gHull.mFacet[i].mVInx[j]].y,
						gHull.mVertes[gHull.mFacet[i].mVInx[j]].z);
					glVertex3f( gHull.mVertes[gHull.mFacet[i].mVInx[(j+1)%3]].x,
						gHull.mVertes[gHull.mFacet[i].mVInx[(j+1)%3]].y,
						gHull.mVertes[gHull.mFacet[i].mVInx[(j+1)%3]].z);
				}
				glEnd();
			}
		}

		SrOBBox3D obb = approximateSmallestOBB(&gHull);

		drawOBB(obb.mAxis,obb.mHalfLength,obb.mCenter);


		delete[] gHull.mFacet;
		delete[] gHull.mVertes;
	}
	delete []point;
	glFlush();
}




void init()
{
	glClearColor(0.0f,0.0f,0.0f,0.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	float MatAmb[] = {0.4f, 0.4f, 0.4f, 1.0f};				// Material - Ambient Values
	float MatDif[] = {0.2f, 0.6f, 0.9f, 1.0f};				// Material - Diffuse Values
	float MatSpc[] = {0.5f, 0.5f, 0.5f, 1.0f};				// Material - Specular Values
	float MatShn[] = {1.0f};
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glMaterialfv(GL_FRONT, GL_AMBIENT, MatAmb);			// Set Material Ambience
	glMaterialfv(GL_FRONT, GL_DIFFUSE, MatDif);			// Set Material Diffuse
	glMaterialfv(GL_FRONT, GL_SPECULAR, MatSpc);		// Set Material Specular
	glMaterialfv(GL_FRONT, GL_SHININESS, MatShn);		// Set Material Shininess
	glEnable(GL_COLOR_MATERIAL);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT1);

}

void reshapeMessage(GLsizei w,GLsizei h)
{
	glViewport(0,0,w,h);
	float pinf[4][4];
	pinf[0][1] = pinf[0][2] = pinf[0][3] = pinf[1][0] =
		pinf[1][2] = pinf[1][3] = pinf[3][0] = pinf[3][1] =
		pinf[3][3] = 0;
	float nearPlane = 1.0f;
	float farPlane	= 10000.0f;
	float fovy = 45;
	float top = nearPlane*tan(3.1415926*fovy/360.0f);
	float bott = -top;
	float right = top*(float)w/(float)h;
	float left = - right;
	pinf[0][0] = 2*nearPlane / (right - left);
	pinf[1][1] = 2*nearPlane / (top - bott);
	pinf[2][0] = ( right + left ) / (right - left);
	pinf[2][1] = ( top + bott) / (top - bott);
	pinf[2][2] = pinf[2][3] = -1;
	pinf[3][2] = -2*nearPlane;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glLoadMatrixf(&pinf[0][0]);
	glMatrixMode(GL_MODELVIEW);
}



void keyMessage(unsigned char key, int x, int y)
{
	switch(key)
	{
	case 's':
	case 'S':
		gStopTimer = !gStopTimer;
		if( !gStopTimer )
		{
			glutTimerFunc(200,timerMessage,0);
		}
		break;
	case 'C':
	case 'c':
		gDisplayMesh = !gDisplayMesh;
		glutPostRedisplay();
		break;
	}
}

int main(int argc,char ** argv)
{
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_SINGLE|GLUT_RGB);
	glutInitWindowSize(400,400);

	glutCreateWindow("Create OBB");

	init();
	outputInfo();
	glutReshapeFunc(reshapeMessage);
	glutDisplayFunc(renderMessage);
	glutTimerFunc(200,timerMessage,0);
	glutKeyboardFunc(keyMessage);

	glutMainLoop();
	return(0);
}
