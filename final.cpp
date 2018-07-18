#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
//#include "opencv2\opencv.hpp"
#include"opencv2/core/core.hpp"
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgcodecs/imgcodecs.hpp"
#include"opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

const float PI = 3.1415926;

void rectifyMap(Mat &mapImg, const int inWidth, const int inHeight,const float* rot, const int outWidth, const int outHeight, const float FOV, const float radius)
{
	float cx = inWidth/2.0;
	float cy = inHeight/2.0;

	float* pMapData = (float*)mapImg.data;
	for (int j = 0; j < outHeight; j++)
	{
		float theta1 = j*PI / outHeight;
		float sinTheta1 = sin(theta1);
		float z1 = cos(theta1);

		for (int i = 0; i < outWidth; i++)
		{
			float fi1 = 2 * PI - i* 2*PI / outWidth;
			float x1 = sinTheta1*cos(fi1);
			float y1 = sinTheta1*sin(fi1);

			//規一化三维座標
			float x2 = rot[0] * x1 + rot[1] * y1 + rot[2] * z1;
			float y2 = rot[3] * x1 + rot[4] * y1 + rot[5] * z1;
			float z2 = rot[6] * x1 + rot[7] * y1 + rot[8] * z1;
			float norm = sqrt(x2*x2 + y2*y2 + z2*z2);
			x2 /= norm;
			y2 /= norm;
			z2 /= norm;

			//球面座標系轉換
			float theta2 = acos(z2)*180/PI;
			float fi2 = atan2(y2, x2);

			if (theta2 <= (FOV / 2) && theta2 >= 0)
			{
				//球面到鱼眼
				float radius2 =radius* theta2 / (FOV / 2);
				float u = (radius2*cos(fi2) + cx);
				float v = (radius2*sin(fi2) + cy);
				if (u >= 0 && u < inWidth - 1 && v >= 0 && v < inHeight - 1)
				{
					pMapData[j*outWidth * 2 + 2 * i + 0] = u;
					pMapData[j*outWidth * 2 + 2 * i + 1] = v;
				}
				else
				{
					pMapData[j*outWidth * 2 + 2 * i + 0] = 0;
					pMapData[j*outWidth * 2 + 2 * i + 1] = 0;
				}
			}
			else
			{
				pMapData[j*outWidth * 2 + 2 * i + 0] = 0;
				pMapData[j*outWidth * 2 + 2 * i + 1] = 0;
			}
		}
	}
}

void remap(const cv::Mat& srcImg, cv::Mat& dstImg, const cv::Mat& mapImg, int inHeight, int inWidth, int outHeight, int outWidth)
{
	uchar* pSrcData = (uchar*)srcImg.data;
	uchar* pDstData = (uchar*)dstImg.data;
	float* pMapData = (float*)mapImg.data;

	for (int j = 0; j < outHeight; j++)
	{
		for (int i = 0; i < outWidth; i++)
		{
			int idx = j*outWidth * 2 + i * 2;
			float u = pMapData[idx + 0];
			float v = pMapData[idx + 1];

			int u0 = floor(u);
			int v0 = floor(v);
			float dx = u - u0;
			float dy = v - v0;
			float weight1 = (1 - dx)*(1 - dy);
			float weight2 = dx*(1 - dy);
			float weight3 = (1 - dx)*dy;
			float weight4 = dx*dy;

			if (u0 >= 0 && v0 >= 0 && (u0 + 1) < inWidth && (v0 + 1) < inHeight)
			{
				float B = weight1*pSrcData[v0*inWidth * 3 + u0 * 3 + 0] + weight2*pSrcData[v0*inWidth * 3 + (u0 + 1) * 3 + 0] +
					weight3*pSrcData[(v0 + 1)*inWidth * 3 + u0 * 3 + 0] + weight4*pSrcData[(v0 + 1)*inWidth * 3 + (u0 + 1) * 3 + 0];

				float G = weight1*pSrcData[v0*inWidth * 3 + u0 * 3 + 1] + weight2*pSrcData[v0*inWidth * 3 + (u0 + 1) * 3 + 1] +
					weight3*pSrcData[(v0 + 1)*inWidth * 3 + u0 * 3 + 1] + weight4*pSrcData[(v0 + 1)*inWidth * 3 + (u0 + 1) * 3 + 1];

				float R = weight1*pSrcData[v0*inWidth * 3 + u0 * 3 + 2] + weight2*pSrcData[v0*inWidth * 3 + (u0 + 1) * 3 + 2] +
					weight3*pSrcData[(v0 + 1)*inWidth * 3 + u0 * 3 + 2] + weight4*pSrcData[(v0 + 1)*inWidth * 3 + (u0 + 1) * 3 + 2];

				int idxResult = j*outWidth * 3 + i * 3;
				pDstData[idxResult + 0] = uchar(B);
				pDstData[idxResult + 1] = uchar(G);
				pDstData[idxResult + 2] = uchar(R);
			}
		}
	}
}

/*void testroi(Mat &img)
{
    int m = img.rows;
    int n = img.cols;
    Rect rect(500,0,1000,m);
    Mat temp(img, rect);
    imwrite("a2.jpg", temp);
}
*/

int main()
{
	string imgPath = "/home/pi/";
	Mat srcImg = imread(imgPath + "o1.jpg");
    //输入鱼眼圖像尺寸
	int inHeight = srcImg.rows;
	int inWidth = srcImg.cols;
    //输出經緯圖像尺寸
	int outHeight = 1000;
	int outWidth = 2000;
    //視場角
	float FOV = 180;
    //鱼眼半徑
	float radius = inWidth / 2.0;

    //以圖像中心為赤道
    float rot[9] = { 1,0,0,0,1,0,0,0,1 };
	float angle = PI/2;
	rot[0] = cos(angle);
	rot[2] = sin(angle);
	rot[6] = -sin(angle);
	rot[8] = cos(angle);

    //求映射Map
	cv::Mat mapImg = cv::Mat::zeros(outHeight, outWidth, CV_32FC2);
	rectifyMap(mapImg, inWidth, inHeight,rot, outWidth, outHeight, FOV, radius);
    //remap得到經緯度圖像
	Mat dstImg = Mat::zeros(outHeight, outWidth, CV_8UC3);
	remap(srcImg, dstImg, mapImg, inHeight, inWidth, outHeight, outWidth);

	imwrite("12051.jpg", dstImg);
	
	
	Mat srcImg2 = imread(imgPath + "o2.jpg");
	int inHeight2 = srcImg.rows;
	int inWidth2 = srcImg.cols;
	int outHeight2 = 1000;
	int outWidth2 = 2000;
	float FOV2 = 180;
	float radius2 = inWidth2 / 2.0;

    float rott[9] = { 1,0,0,0,1,0,0,0,1 };
	float angle2 = PI/2;
	rott[0] = cos(angle2);
	rott[2] = sin(angle2);
	rott[6] = -sin(angle2);
	rott[8] = cos(angle2);

	cv::Mat mapImg2 = cv::Mat::zeros(outHeight2, outWidth2, CV_32FC2);
	rectifyMap(mapImg2, inWidth2, inHeight2,rott, outWidth2, outHeight2, FOV2, radius2);
	Mat dstImg2 = Mat::zeros(outHeight2, outWidth2, CV_8UC3);
	remap(srcImg2, dstImg2, mapImg2, inHeight2, inWidth2, outHeight2, outWidth2);

	imwrite("12052.jpg", dstImg2);
	
	Mat img3 = imread("12051.jpg");
    int m = img3.rows;
    int n = img3.cols;
    Rect rect(500,0,1000,m);
    Mat temp(img3, rect);
    imwrite("a1.jpg", temp);
	
	
	Mat img4 = imread("12052.jpg");
    int m2 = img4.rows;
    int n2 = img4.cols;
    Rect rect2(500,0,1000,m2);
    Mat temp2(img4, rect2);
    imwrite("a2.jpg", temp2);
	
	Mat combine,combine1,combine2;  
    Mat a=imread("a1.jpg");  
    Mat b=imread("a2.jpg");    
    hconcat(b,a,combine);    
    imwrite("combine.jpg",combine);    
    //cout<<"Combine=:"<<combine<<endl;  
    system("pause"); 
	
	return 0;
}
