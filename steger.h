#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stack>
using namespace std;
using namespace cv;
class Steger {
public:
	Steger(Mat, int);
	~Steger();
	void Init();
	void FindNormal(int, int, double&, double&);
	void StructFindCenterLine(vector<Point2f>&);
	void inrange_demo(Mat &);
	Mat Img;
	int Threshold;
	Mat dx, dy;
	Mat dxx, dyy, dxy;

private:

};

Steger::Steger(Mat image, int thres) {
	Threshold = thres;
	Img = image;
	if (Threshold !=0)  inrange_demo(Img);
	if (Threshold == 0)
	{
		for (int i = 0; i < Img.cols; ++i)
		{
			for (int j = 0; j < Img.rows; ++j)
			{
				Vec3b bgr = Img.at<Vec3b>(j, i);
				float flag = bgr[0] * 0.114 + 0.587*bgr[1] + 0.299*bgr[2];
				if (flag > 70)//��ֵ���Ǹ�ʲô�����أ� ���������ɫ������
				{
					Img.at<Vec3b>(j, i)[0] = 255;
					Img.at<Vec3b>(j, i)[1] = 255;
					Img.at<Vec3b>(j, i)[2] = 255;
				}
				else
				{
					Img.at<Vec3b>(j, i)[0] = 0;
					Img.at<Vec3b>(j, i)[1] = 0;
					Img.at<Vec3b>(j, i)[2] = 0;
				}
			}
		}
		inrange_demo(Img);
	}
	cvtColor(Img, Img, COLOR_BGR2GRAY);
	cout << "size of Img:" << Img.cols << "*" << Img.rows << endl;
}
Steger::~Steger() {

}
void Steger::Init() {
	Mat image;
	Img.convertTo(image, CV_32FC1);
	GaussianBlur(image, image, Size(0, 0), 6, 6);
	Mat m1, m2, m3, m4, m5;
	m1 = (Mat_<float>(3, 1) << 1, 0, -1);  //xƫ��
	m2 = (Mat_<float>(1, 3) << 1, 0, -1);  //yƫ��
	m3 = (Mat_<float>(3, 1) << 1, -2, 1);   //����xƫ��
	m4 = (Mat_<float>(1, 3) << 1, -2, 1);   //����yƫ��
	m5 = (Mat_<float>(2, 2) << 1, -1, -1, 1);   //����xyƫ��

	filter2D(image, dx, CV_32FC1, m1);
	filter2D(image, dy, CV_32FC1, m2);
	filter2D(image, dxx, CV_32FC1, m3);
	filter2D(image, dyy, CV_32FC1, m4);
	filter2D(image, dxy, CV_32FC1, m5);
}
void Steger::inrange_demo(Mat &image)
{
	Mat hsv;
	Mat dst;
	cvtColor(image, hsv, COLOR_BGR2HSV);
	Mat mask;
	inRange(hsv, Scalar(0, 0, 221), Scalar(180, 30, 255), mask);
	bitwise_and(image, image, dst, mask);
	image = dst.clone();
}
void Steger::FindNormal(int y, int x, double& nx, double& ny) {
	Mat hessian(2, 2, CV_32FC1);
	hessian.at<float>(0, 0) = dxx.at<float>(y, x);
	hessian.at<float>(0, 1) = dxy.at<float>(y, x);
	hessian.at<float>(1, 0) = dxy.at<float>(y, x);
	hessian.at<float>(1, 1) = dyy.at<float>(y, x);
	double a = dxx.at<float>(y, x);
	double b = dyy.at<float>(y, x);
	double c = dxy.at<float>(y, x);
	Mat eValue;
	Mat eVectors;
	eigen(hessian, eValue, eVectors);  //��������ֵ
									   //��(x,y)�ķ��߷�����ɸõ� Hessian �������ֵ��������ֵ����Ӧ����������(nx,ny)����
									   //cout << eValue << endl;
	if (fabs(eValue.at<float>(0, 0)) >= fabs(eValue.at<float>(1, 0)))  //fabs���ظ���������ֵ
	{
		nx = eVectors.at<float>(0, 0); //��������
		ny = eVectors.at<float>(0, 1);
	}
	else
	{
		nx = eVectors.at<float>(1, 0);
		ny = eVectors.at<float>(1, 1);
	}
}
void Steger::StructFindCenterLine(vector<Point2f>& pt) {
	if (Threshold != 0)
	{
		for (int i = 0; i < Img.cols; i++)
			{
				for (int j = 0; j < Img.rows; j++)
				{
					if (Img.at<uchar>(j, i))
					{
						double nx, ny;
						FindNormal(j, i, nx, ny);//Ѱ�Ҽ���������߷���
														 // �ж������ص��Ƿ��ڸ������ڣ��õ� (x0,y0) Ϊ���������ĵ㣬 (px,py) ��Ϊ����������
						double bottom = (nx * nx * dxx.at<float>(j, i) + 2 * nx * ny * dxy.at<float>(j, i) + ny * ny * dyy.at<float>(j, i));
						if (bottom != 0) {
							double t = -(nx * dx.at<float>(j, i) + ny * dy.at<float>(j, i)) / bottom;
							if (fabs(t * nx) <= 0.5 && fabs(t * ny) <= 0.5)
							{
								pt.push_back(Point2f(i + t * nx, j + t * ny));
							}
						}
					}
				}
			}
	}
	else
	{
	    int scol = 330;
		int ecol = 460;
		int srow = 280;
		int erow = 305;
		for (int i = scol; i < ecol; i++)
		{
			for (int j = srow; j < erow; j++)
			{
				if (Img.at<uchar>(j, i))
				{
					double nx, ny;
					FindNormal(j, i, nx, ny);//Ѱ�Ҽ���������߷���
													 // �ж������ص��Ƿ��ڸ������ڣ��õ� (x0,y0) Ϊ���������ĵ㣬 (px,py) ��Ϊ����������
					double bottom = (nx * nx * dxx.at<float>(j, i) + 2 * nx * ny * dxy.at<float>(j, i) + ny * ny * dyy.at<float>(j, i));
					if (bottom != 0) {
						double t = -(nx * dx.at<float>(j, i) + ny * dy.at<float>(j, i)) / bottom;
						if (fabs(t * nx) <= 0.5 && fabs(t * ny) <= 0.5)
						{
							pt.push_back(Point2f(i + t * nx, j + t * ny));
						}
					}
				}
			}
		}
	}
	
}