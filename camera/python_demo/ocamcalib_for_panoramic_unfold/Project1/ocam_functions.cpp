/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   Copyright (C) 2008 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ocam_functions.h"

//------------------------------------------------------------------------------
int get_ocam_model(struct ocam_model *myocam_model, char *filename)
{
 double *pol        = myocam_model->pol;    // poland invpol are both arrays. for arrays, their names actually equal the poiter of their first element.
 double *invpol     = myocam_model->invpol; // like int array[3]; int *p = a; this equals: int *p = &a[0]
 double *xc         = &(myocam_model->xc);
 double *yc         = &(myocam_model->yc);
 double *c          = &(myocam_model->c);
 double *d          = &(myocam_model->d);
 double *e          = &(myocam_model->e);
 int    *width      = &(myocam_model->width);
 int    *height     = &(myocam_model->height);
 int *length_pol    = &(myocam_model->length_pol);
 int *length_invpol = &(myocam_model->length_invpol);
 FILE *f;
 char buf[CMV_MAX_BUF];
 int i;

 //Open file
 if(!(f=fopen(filename,"r")))
 {
   printf("File %s cannot be opened\n", filename);
   return -1;
 }

 //Read polynomial coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_pol);
 for (i = 0; i < *length_pol; i++)
 {
     fscanf(f," %lf",&pol[i]);
 }

 //Read inverse polynomial coefficients
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_invpol);
 for (i = 0; i < *length_invpol; i++)
 {
     fscanf(f," %lf",&invpol[i]);
 }

 //Read center coordinates
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf\n", xc, yc);

 //Read affine coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf %lf\n", c,d,e);

 //Read image size
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d %d", height, width);

 fclose(f);
 return 0;
}

//------------------------------------------------------------------------------
void cam2world(double point3D[3], double point2D[2], struct ocam_model *myocam_model)
{
 double *pol    = myocam_model->pol;
 double xc      = (myocam_model->xc);
 double yc      = (myocam_model->yc);
 double c       = (myocam_model->c);
 double d       = (myocam_model->d);
 double e       = (myocam_model->e);
 int length_pol = (myocam_model->length_pol);
 double invdet  = 1/(c-d*e); // 1/det(A), where A = [c,d;e,1] as in the Matlab file

 double xp = invdet*(    (point2D[0] - xc) - d*(point2D[1] - yc) );
 double yp = invdet*( -e*(point2D[0] - xc) + c*(point2D[1] - yc) );

 double r   = sqrt(  xp*xp + yp*yp ); //distance [pixels] of  the point from the image center
 double zp  = pol[0];
 double r_i = 1;
 int i;

 for (i = 1; i < length_pol; i++)
 {
   r_i *= r;
   zp  += r_i*pol[i];
 }

 //normalize to unit norm
 double invnorm = 1/sqrt( xp*xp + yp*yp + zp*zp );

 point3D[0] = invnorm*xp;
 point3D[1] = invnorm*yp;
 point3D[2] = invnorm*zp;
}

//------------------------------------------------------------------------------
void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
 double *invpol     = myocam_model->invpol;
 double xc          = (myocam_model->xc);
 double yc          = (myocam_model->yc);
 double c           = (myocam_model->c);
 double d           = (myocam_model->d);
 double e           = (myocam_model->e);
 int    width       = (myocam_model->width);
 int    height      = (myocam_model->height);
 int length_invpol  = (myocam_model->length_invpol);
 double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
 double theta       = atan(point3D[2]/norm);
 double t, t_i;
 double rho, x, y;
 double invnorm;
 int i;

  if (norm != 0)
  {
    invnorm = 1/norm;
    t  = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i*invpol[i];
    }

    x = point3D[0]*invnorm*rho;
    y = point3D[1]*invnorm*rho;

    point2D[0] = x*c + y*d + xc;
    point2D[1] = x*e + y   + yc;
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
  }
}
//------------------------------------------------------------------------------
void create_perspecive_undistortion_LUT( CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf)
{
     int i, j;
     int width = mapx->cols; //New width
     int height = mapx->rows;//New height
     float *data_mapx = mapx->data.fl;
     float *data_mapy = mapy->data.fl;
     float Nxc = height/2.0;
     float Nyc = width/2.0;
     float Nz  = -width/sf;
     double M[3];
     double m[2];

     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {
             M[0] = (i - Nxc);
             M[1] = (j - Nyc);
             M[2] = Nz;
             world2cam(m, M, ocam_model);
             *( data_mapx + i*width+j ) = (float) m[1];
             *( data_mapy + i*width+j ) = (float) m[0];
         }
}

//------------------------------------------------------------------------------
void create_panoramic_undistortion_LUT ( CvMat *mapx, CvMat *mapy, float Rmin, float Rmax, float xc, float yc )
{
     int i, j;
     float theta;
     int width = mapx->width;
     int height = mapx->height;
     float *data_mapx = mapx->data.fl;
     float *data_mapy = mapy->data.fl;
     float rho;

     for (i=0; i<height; i++)
         for (j=0; j<width; j++)
         {
             theta = -((float)j)/width*2*3.1416; // Note, if you would like to flip the image, just inverte the sign of theta
             rho   = Rmax - (Rmax-Rmin)/height*i;
             *( data_mapx + i*width+j ) = yc + rho*sin(theta); //in OpenCV "x" is the
             *( data_mapy + i*width+j ) = xc + rho*cos(theta);
         }
}
void create_panoramic_CUBE(CvMat *mapx, CvMat *mapy, float Rmin, float Rmax, float xc, float yc, int face, struct ocam_model *myocam_model)
{
	int i, j;
	float theta;
	int width = mapx->width;
	int height = mapx->height;
	float *data_mapx = mapx->data.fl;
	float *data_mapy = mapy->data.fl;
	float rho;
	int size_y = 1;//��������ϵ��
	float size_x = 1.732;
	switch (face)
	{
	case 1:
		for (i = 0; i<height; i++)
			for (j = 0; j<width; j++)
			{
				//float z = 0;
				//theta = -((float)j) / width/4 * 2 * 3.1416; // Note, if you would like to flip the image, just inverte the sign of theta
				//rho = Rmax - (Rmax - Rmin) / height*i;
				//*(data_mapx + i*width + j) = yc + rho*sin(theta); //in OpenCV "x" is the
				//*(data_mapy + i*width + j) = xc + rho*cos(theta);
				double point3[3];
				point3[0] = -1;
				point3[1] = -size_y*(1 - 2 * (double)j / (double)width);// 2 - 2 * (double)(width - j) / (double)width;
				point3[2] = -(double)i*size_x / ((double)height);
				double point2[2];
				world2cam(point2, point3, myocam_model);
				*(data_mapx + i*width + j) = point2[1];
				*(data_mapy + i*width + j) = point2[0];
			}
		break;
	case 3:
		for (i = 0; i<height; i++)
			for (j = 0; j<width; j++)
			{
				//theta = -(((float)j) / width / 4+0.25) * 2 * 3.1416; // Note, if you would like to flip the image, just inverte the sign of theta
				//rho = Rmax - (Rmax - Rmin) / height*i;
				//*(data_mapx + i*width + j) = yc + rho*sin(theta); //in OpenCV "x" is the
				//*(data_mapy + i*width + j) = xc + rho*cos(theta);
				double point3[3];
				point3[0] = 1;
				point3[1] = size_y*(1 -  2*(double)j / (double)width);
				point3[2] = -(double)i*size_x / ((double)height);
				double point2[2];
				world2cam(point2, point3, myocam_model);
				*(data_mapx + i*width + j) = point2[1];
				*(data_mapy + i*width + j) = point2[0];
			}
		break;
	case 2:
		for (i = 0; i<height; i++)
			for (j = 0; j<width; j++)
			{
				//theta = -(((float)j) / width / 4 + 0.25) * 2 * 3.1416; // Note, if you would like to flip the image, just inverte the sign of theta
				//rho = Rmax - (Rmax - Rmin) / height*i;
				//*(data_mapx + i*width + j) = yc + rho*sin(theta); //in OpenCV "x" is the
				//*(data_mapy + i*width + j) = xc + rho*cos(theta);
				double point3[3];
				point3[0] = -size_y*(1 - 2 * (double)j / (double)width);
				point3[1] = 1;//(double)2*j / (double)width;//
				point3[2] = -(double)i*size_x / ((double)height);
				double point2[2];
				world2cam(point2, point3, myocam_model);
				*(data_mapx + i*width + j) = point2[1];
				*(data_mapy + i*width + j) = point2[0];
			}
		break;
	case 4:
		for (i = 0; i<height; i++)
			for (j = 0; j<width; j++)
			{
				//theta = -(((float)j) / width / 4 + 0.75) * 2 * 3.1416; // Note, if you would like to flip the image, just inverte the sign of theta
				//rho = Rmax - (Rmax - Rmin) / height*i;
				//*(data_mapx + i*width + j) = yc + rho*sin(theta); //in OpenCV "x" is the
				//*(data_mapy + i*width + j) = xc + rho*cos(theta);
				double point3[3];
				point3[0] = size_y*(1 - 2 * (double)j / (double)width);
				point3[1] = -1;
				point3[2] = -(double)i*size_x / ((double)height);
				double point2[2];
				world2cam(point2, point3, myocam_model);
				*(data_mapx + i*width + j) = point2[1];
				*(data_mapy + i*width + j) = point2[0];
			}
		break;
	}

}
void create_panoramic_CUBE_plus(CvMat *mapx, CvMat *mapy, float Rmin, float Rmax, float xc, float yc, int face, struct ocam_model *myocam_model,float angle[4][2])
{
	int i, j;
	float theta;
	int width = mapx->width;
	int height = mapx->height;
	float *data_mapx = mapx->data.fl;
	float *data_mapy = mapy->data.fl;
	float rho;


	float ang_nowface = angle[face][1] - angle[face][0];
	float size_x = 1.732;
	float size_y = tan(ang_nowface / 2);//��������ϵ��
	float rot_cos = cos(angle[face][0] + ang_nowface / 2);
	float rot_sin = sin(angle[face][0] + ang_nowface / 2);

	for (i = 0; i<height; i++)
		for (j = 0; j<width; j++)
		{
			//float z = 0;
			//theta = -((float)j) / width/4 * 2 * 3.1416; // Note, if you would like to flip the image, just inverte the sign of theta
			//rho = Rmax - (Rmax - Rmin) / height*i;
			//*(data_mapx + i*width + j) = yc + rho*sin(theta); //in OpenCV "x" is the
			//*(data_mapy + i*width + j) = xc + rho*cos(theta);
			double point3_temp[3];
			double point3[3];
			point3_temp[0] = 1;
			point3_temp[1] = -size_y*(1 - 2 * (double)j / (double)width);// 2 - 2 * (double)(width - j) / (double)width;
			point3_temp[2] = -(double)i*size_x / ((double)height);
			point3[0] = point3_temp[0] * rot_cos - point3_temp[1] * rot_sin;
			point3[1] = point3_temp[0] * rot_sin + point3_temp[1] * rot_cos;
			point3[2] = point3_temp[2];
			double point2[2];
			world2cam(point2, point3, myocam_model);
			*(data_mapx + i*width + j) = point2[1];
			*(data_mapy + i*width + j) = point2[0];
		}

}
