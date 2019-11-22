#include<iostream>
#include <opencv2\opencv.hpp>
#include <math.h>
#include <algorithm>
#include <vector>
using namespace std;
using namespace cv;

int main() {
	Mat input = imread(".\\raw_img.jpg");
	Mat output;
	// int histSize = 255;
	// float range[] = {0, 255};
	// const float *histRange = {range};
	// bool uniform = true;
	// bool accumulate = false;
	// Mat hist;
	// int hist_h = 400;
	// int hist_w = 400;
	// int bin_w = cvRound((double)hist_w/histSize);
	// Mat histImage(hist_w, hist_h, CV_8UC3, Scalar(0,0,0));
	Mat ele1 = getStructuringElement(MORPH_RECT, Size(1,1));
	Mat ele2 = getStructuringElement(MORPH_RECT, Size(7,7));
	Mat ele3 = getStructuringElement(MORPH_RECT, Size(3,3));
	Mat ele4 = getStructuringElement(MORPH_RECT, Size(3,3));
	Mat temp;
	Mat dst = Mat::zeros(input.size(), CV_8UC1);
	int maxOverlapCircle(Vec3f c, Mat &rimg);
	int centerDist(Vec3f c, Vec3f &min_c);
	//
	GaussianBlur(input, input, Size(9, 9), 0, 0);
	cvtColor(input, output, CV_BGR2GRAY);
	cout << output.depth() << endl;
	// // 计算和显示直方图
	// // calcHist(&output, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
	// // normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat());
	// // for (int i = 1; i!=histSize; i++){
	// // 	line(histImage, Point(bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1))),
	// // 			Point(bin_w*(i), hist_h - cvRound(hist.at<float>(i))),
	// // 			Scalar(0, 0, 255), 2, 8, 0);
	// // }
	// //
	// // namedWindow("histogram", CV_WINDOW_AUTOSIZE);
	// // imshow("histogram", histImage);
	// /**************************************/
	// // 二值化
	// GaussianBlur(output, output, Size(15, 15), 0, 0);
	// temp = output.clone();
	// adaptiveThreshold(temp, temp, 255, 0, 0, 3, 0);
	Canny(output, output, 20, 30, 3);
	morphologyEx(output, output, MORPH_OPEN, ele1);
	morphologyEx(output, output, MORPH_CLOSE, ele2);
	// morphologyEx(output, output, MORPH_OPEN, ele3);
	// morphologyEx(output, output, MORPH_CLOSE, ele4);
	morphologyEx(output, output, MORPH_ERODE, ele3);
	// morphologyEx(output, output, MORPH_DILATE, ele4);
	medianBlur(output, output, 9);
	temp = output.clone();
	// namedWindow("original image", 0);
	// imshow("original image", output);
	// waitKey(0);
	//
	// // 删除小面积区域
	// vector<vector<Point>> contours;
	// vector<Vec4i> hierarchy;
	//
	// // findContours函数会对输入图像进行改变，所以最好是clone一个用于找连通域
	// findContours(output.clone(), contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	// cout<<contours.size()<<endl;
	// auto it = contours.begin();
	// while(it != contours.end()){
	// 	if (contourArea(*it)<10){
	// 		// cout<<contourArea(*it)<<endl;
	// 		it = contours.erase(it);
	// 	}
	// 	else{
	// 		it++;
	// 	}
	//
	// }
	// cout<<contours.size()<<endl;
	// drawContours(output, contours, -1, Scalar(255));
	// namedWindow("delete small area", 0);
	// imshow("delete small area", dst);
	// waitKey(0);
	// 霍夫圆变换 */
	vector<Vec3f> circles;
	// 最后两位填入半径的估计范围 310, 350               1, 20, 10, 30, 300, 420
	HoughCircles(output.clone(), circles, CV_HOUGH_GRADIENT, 1, 1, 1, 20, 300, 420);
	vector<int> n_pixels;
	cout << circles.size() << endl;
	for (size_t i = 0; i != circles.size(); i++){
		Vec3f c = circles[i];
		// 画圆
		circle(input, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, 8);
		// 画圆心
		circle(input, Point(c[0], c[1]), 2, Scalar(0,0,255), 3, 8);
		//计算和canny的重合像素数
		n_pixels.push_back(maxOverlapCircle(c, output));
	}
	// cout<<"ent here4"<<endl;
	// 画霍夫变换找到的票数最高的圆,圆度不高的时候不适用
	Vec3f min_cir = circles[0];
	// 画圆
	circle(input, Point(min_cir[0], min_cir[1]), min_cir[2], Scalar(0,255,0), 3, 8);
	// 画圆心
	circle(input, Point(min_cir[0], min_cir[1]), 2, Scalar(0,255,0), 3, 8);
	// 画与实际像素重合最大的圆，在噪声多，特别有同心环时不适用
	// auto maxPosition = max_element(n_pixels.begin(), n_pixels.end());
	// auto p = maxPosition - n_pixels.begin();
	// Vec3f min_cir = circles[p];
	// // 画圆
	// circle(input, Point(min_cir[0], min_cir[1]), min_cir[2], Scalar(0,255,0), 3, 8);
	// // 画圆心
	// circle(input, Point(min_cir[0], min_cir[1]), 2, Scalar(0,255,0), 3, 8);
	cout<<"The center position is ("<<min_cir[0]<<", "<<min_cir[1]<<")"<<endl;
	cout<<"The Min Radius is "<<min_cir[2]<<endl;
	// /********************* 画外面的大圆 ********************/
	// equalizeHist(temp, temp);
	// medianBlur(temp, temp, 15);
	// GaussianBlur(temp, temp, Size(15, 15), 0, 0);
	// // 采用自适应方法阈值分割
	// threshold(temp, temp, 180, 255, THRESH_OTSU);
	// // adaptiveThreshold(temp, temp, 255, 0, 0, 3, 0);
	vector<Vec3f> circles2;
	// 最后两位填入半径的估计范围 1000, 1150
	HoughCircles(temp.clone(), circles2, CV_HOUGH_GRADIENT, 1, 1, 1, 15, 1050, 1150);
	vector<int> center_dist;
	cout << circles2.size() << endl;
	for (size_t i = 0; i != circles2.size(); i++){
		Vec3f c = circles2[i];
		// circle(input, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, 8);
		// // 画圆心
		// circle(input, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, 8);
		//计算圆心和小圆圆心最近的
		center_dist.push_back(centerDist(c, min_cir));
	}
	// cout<<"ent here4"<<endl;
	auto minPosition = min_element(center_dist.begin(), center_dist.end());
	auto d = minPosition - center_dist.begin();
	Vec3f max_cir = circles2[d];
	// 画圆
	circle(input, Point(max_cir[0], max_cir[1]), max_cir[2], Scalar(255,0,0), 3, 8);
	// 画圆心
	circle(input, Point(max_cir[0], max_cir[1]), 2, Scalar(255,0,0), 3, 8);
	cout<<"The center position is ("<<max_cir[0]<<", "<<max_cir[1]<<")"<<endl;
	cout<<"The Max Radius is "<<max_cir[2]<<endl;
	//
	//
	namedWindow("input image", 0);
	imshow("input image", input);
	// namedWindow("output image", 0);
	// imshow("output image", temp);
	waitKey(0);
	return 0;
}

int maxOverlapCircle(Vec3f c, Mat &rimg){
	// cout<<"ent here1"<<endl;
	uchar value;
	int count=0;
	float dist;
	for(int i = 0; i != rimg.rows; i++){
		for (int j = 0; j != rimg.cols; j++){
			value = rimg.at<uchar>(i,j);
			if (value == 255 ){
				dist = sqrt(pow((i-c[0]),2)+pow(j-c[1],2));
				if (abs(dist-c[2])<=5){
					count++;
				}
			}
		}
	}
	// cout<<count<<endl;
	return count;
}

int centerDist(Vec3f c, Vec3f &min_c){
	float dist = sqrt(pow(c[0]-min_c[0], 2)+pow(c[1]-min_c[1], 2));
	return dist;
}
