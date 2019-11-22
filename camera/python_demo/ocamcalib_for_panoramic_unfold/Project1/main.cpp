/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images

   NOTE, IF YOU WANT TO SPEED UP THE REMAP FUNCTION I STRONGLY RECOMMEND TO INSTALL
   INTELL IPP LIBRARIES ( http://software.intel.com/en-us/intel-ipp/ )
   YOU JUST NEED TO INSTALL IT AND INCLUDE ipp.h IN YOUR PROGRAM

   Copyright (C) 2009 DAVIDE SCARAMUZZA, ETH Zurich
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ocam_functions.h"
#include <opencv2\opencv.hpp>
int main(int argc, char *argv[])
{
	char *result_txt = argv[1];
	const char *out_path = argv[2];
	char *fn = argv[3];

	// std::cout << img_name <<std::endl;
  /* --------------------------------------------------------------------*/
  /* Read the parameters of the omnidirectional camera from the TXT file */
  /* --------------------------------------------------------------------*/

	cv::String pattern;
	// �˺��Ͳ���Ҫ��glob��ȡ�ļ���ֱ��python����ͼ�񴫲ν���
	// pattern = "D:\\11projects\\pal-face\\camera\\python_demo\\ocamcalib_for_panoramic_unfold\\Project1\\newfolder";
	// std::vector<cv::String> fn;
  // //std::cout<< fn << std::endl;
	// cv::glob(pattern, fn, false); // read all the image names into fn
	// char fn = *img_name;
	std::cout << fn << std::endl;

	//cv::Mat image = cv::imread(fn[ii]);
	struct ocam_model o, o_cata, o_cata1; // our ocam_models for the fisheye and catadioptric cameras
										  // get_ocam_model(&o, "./calib_results1.txt");
	//get_ocam_model(&o_cata, "./calib_results0.txt");
	//get_ocam_model(&o_cata1, "./calib_results0.txt");
	get_ocam_model(&o_cata, result_txt);
	get_ocam_model(&o_cata1, result_txt);
	/* --------------------------------------------------------------------*/
	/* Print ocam_model parameters                                         */
	/* --------------------------------------------------------------------*/
	int i;
	/*printf("pol =\n");    for (i = 0; i<o.length_pol; i++) { printf("\t%e\n", o.pol[i]); };    printf("\n");
	printf("invpol =\n"); for (i = 0; i<o.length_invpol; i++) { printf("\t%e\n", o.invpol[i]); }; printf("\n");
	printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n", o.xc, o.yc, o.width, o.height);*/

	/* --------------------------------------------------------------------*/
	/* WORLD2CAM projects 3D point into the image                          */
	/* NOTE!!! The coordinates are expressed according the C convention,   */
	/* that is, from the origin (0,0) instead than from 1 (MATLAB).        */
	/* --------------------------------------------------------------------*/
	//double point3D[3] = { 100 , 200 , -300 };       // a sample 3D point
	//double point2D[2];                              // the image point in pixel coordinates
	//world2cam(point2D, point3D, &o_cata); // The behaviour of this function is the same as in MATLAB

	//								 /* --------------------------------------------------------------------*/
	//								 /* Display re-projected coordinates                                    */
	//								 /* --------------------------------------------------------------------*/
	///*printf("\nworld2cam: pixel coordinates reprojected onto the image\n");
	//printf("m_row= %2.4f, m_col=%2.4f\n", point2D[0], point2D[1]);*/

	///* --------------------------------------------------------------------*/
	///* CAM2WORLD back-projects pixel points on to the unit sphere          */
	///* The behaviour of this function is the same as in MATLAB             */
	///* --------------------------------------------------------------------*/

	//cam2world(point3D, point2D, &o_cata);

	/* --------------------------------------------------------------------*/
	/* Display back-projected normalized coordinates (on the unit sphere)  */
	/* --------------------------------------------------------------------*/
	/*printf("\ncam2world: coordinates back-projected onto the unit sphere (x^2+y^2+z^2=1)\n");
	printf("x= %2.4f, y=%2.4f, z=%2.4f\n", point3D[0], point3D[1], point3D[2]);*/

	/* --------------------------------------------------------------------*/
	/* Allocate space for the unistorted images                            */
	/* --------------------------------------------------------------------*/
	//IplImage *src1         = cvLoadImage("./96.jpg");      // source image 1

	char fn1[256] = { 0 };
	sprintf(fn1, "%s",fn);
	//fn1Ӧ���Ǹ�·����char��
	IplImage *src2 = cvLoadImage(fn1);      // source image 2
	IplImage *src3 = cvLoadImage(fn1); 													 //IplImage *dst_persp   = cvCreateImage( cvGetSize(src1), 8, 3 );   // undistorted perspective and panoramic image

	CvSize size_pan_image = cvSize(3000, 600);// (2070, 345);        // size of the undistorted panoramic image
													  //CvSize size_pan_image = cvSize(1472, 448);        // size of the undistorted panoramic image
	// dst_pan�൱���Ǵ���һ���µĿհ�ͼ�񣬷ֱ���Ϊ��3000,600����λ����Ϊ8��ͨ����3
	IplImage *dst_pan = cvCreateImage(size_pan_image, 8, 3);    // undistorted panoramic image
	IplImage *dst_pan1 = cvCreateImage(size_pan_image, 8, 3);
																//CvMat* mapx_persp = cvCreateMat(src1->height, src1->width, CV_32FC1);
																//CvMat* mapy_persp = cvCreateMat(src1->height, src1->width, CV_32FC1);
	CvMat* mapx_pan = cvCreateMat(dst_pan->height, dst_pan->width, CV_32FC1);
	CvMat* mapy_pan = cvCreateMat(dst_pan->height, dst_pan->width, CV_32FC1);
	CvMat* mapx_pan1 = cvCreateMat(dst_pan1->height, dst_pan1->width, CV_32FC1);
	CvMat* mapy_pan1 = cvCreateMat(dst_pan1->height, dst_pan1->width, CV_32FC1);
	/* --------------------------------------------------------------------  */
	/* Create Look-Up-Table for perspective undistortion                     */
	/* SF is kind of distance from the undistorted image to the camera       */
	/* (it is not meters, it is justa zoom fator)                            */
	/* Try to change SF to see how it affects the result                     */
	/* The undistortion is done on a  plane perpendicular to the camera axis */
	/* --------------------------------------------------------------------  */
	float sf = 4;
	//create_perspecive_undistortion_LUT( mapx_persp, mapy_persp, &o, sf );

	/* --------------------------------------------------------------------  */
	/* Create Look-Up-Table for panoramic undistortion                       */
	/* The undistortoin is just a simple cartesia-to-polar transformation    */
	/* Note, only the knowledge of image center (xc,yc) is used to undisort the image      */
	/* xc, yc are the row and column coordinates of the image center         */
	/* Note, if you would like to flip the image, just inverte the sign of theta in this function */
	/* --------------------------------------------------------------------  */
	float Rmax = 1063.78;  // the maximum radius of the region you would like to undistort into a panorama
	float Rmin = 329.949;   // the minimum radius of the region you would like to undistort into a panorama
	//o_cata.xc = 493;
	//o_cata.yc = 629;
	//o_cata.xc = 2052;
	//o_cata.yc = 3052;
	create_panoramic_undistortion_LUT(mapx_pan, mapy_pan, Rmin, Rmax, o_cata.xc, o_cata.yc);
	create_panoramic_undistortion_LUT(mapx_pan1, mapy_pan1, Rmin, Rmax, o_cata1.xc, o_cata1.yc);
	/* --------------------------------------------------------------------*/
	/* Undistort using specified interpolation method                      */
	/* Other possible values are (see OpenCV doc):                         */
	/* CV_INTER_NN - nearest-neighbor interpolation,                       */
	/* CV_INTER_LINEAR - bilinear interpolation (used by default)          */
	/* CV_INTER_AREA - resampling using pixel area relation. It is the preferred method for image decimation that gives moire-free results. In case of zooming it is similar to CV_INTER_NN method. */
	/* CV_INTER_CUBIC - bicubic interpolation.                             */
	/* --------------------------------------------------------------------*/
	//cvRemap( src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );
	cvRemap(src2, dst_pan, mapx_pan, mapy_pan, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
	cvRemap(src3, dst_pan1, mapx_pan1, mapy_pan1, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
	cvFlip(dst_pan,dst_pan,-1);
	/* --------------------------------------------------------------------*/
	/* Display image                                                       */
	/* --------------------------------------------------------------------*/
	//cvNamedWindow( "Original fisheye camera image", 1 );
	//cvShowImage( "Original fisheye camera image", src1 );

	//cvNamedWindow( "Undistorted Perspective Image", 1 );
	//cvShowImage( "Undistorted Perspective Image", dst_persp );

	//cvNamedWindow("Original Catadioptric camera image", 1);
	//cvShowImage("Original Catadioptric camera image", src2);

	//cvNamedWindow("Undistorted Panoramic Image", 1);
	//cvShowImage("Undistorted Panoramic Image", dst_pan);

	/* --------------------------------------------------------------------*/
	/* Save image                                                          */
	/* --------------------------------------------------------------------*/
	// cvSaveImage("undistorted_perspective.jpg",dst_persp);
	// printf("\nImage %s saved\n","undistorted_perspective.jpg");
	char fn2[256] = { 0 };

	//sprintf(fn2, "D:\\11projects\\pal-face\\camera\\ocamcalib_for_panoramic_unfold\\Project1\\result\\%06d.jpg", iii);
	sprintf(fn2, out_path, 0);
	cvSaveImage(fn2, dst_pan);  //���󱣴������ĺ���ֻ��dst_pan? dst_pan1����û������

	char fn3[256] = { 0 };

	//sprintf(fn3, "%s_2.jpg", fn[ii]);
	//cvSaveImage(fn3, dst_pan1);
	//printf("\nImage %s saved\n", "undistorted_panoramic.jpg");

	/* --------------------------------------------------------------------*/
	/* Wait until key presses                                              */
	/* --------------------------------------------------------------------*/
	cvWaitKey();

	/* --------------------------------------------------------------------*/
	/* Free memory                                                         */
	/* --------------------------------------------------------------------*/
	//cvReleaseImage(&src1);
	cvReleaseImage(&src2);
	//cvReleaseImage(&dst_persp);
	cvReleaseImage(&dst_pan);
	// cvReleaseMat(&mapx_persp);
	// cvReleaseMat(&mapy_persp);
	cvReleaseMat(&mapx_pan);
	cvReleaseMat(&mapy_pan);

	cvReleaseImage(&src3);
	//cvReleaseImage(&dst_persp);
	cvReleaseImage(&dst_pan1);
	// cvReleaseMat(&mapx_persp);
	// cvReleaseMat(&mapy_persp);
	cvReleaseMat(&mapx_pan1);
	cvReleaseMat(&mapy_pan1);



}
