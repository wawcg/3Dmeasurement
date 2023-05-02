#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include "steger.h"
#include <fstream>
#include <stdio.h>
#include <gsl/gsl_multifit.h>
# include <iostream>
#include <string>
#include<algorithm>
#include<cmath>
#define C(i) (gsl_vector_get(c,(i)))
#define COV(i,j) (gsl_matrix_get(cov,(i),(j)))
using namespace std;
using namespace cv;
class Calibrate {
public:
	Calibrate(Mat, vector<Mat>, vector<Mat>, vector<Mat>, int, int, int, int, int);
	~Calibrate();
	void calibrateNCamera();//�궨����ڲ�
	void calibrateWCamera(int wcol, int wrow);
	void planecalibration(int wcol, int wrow);
	void Struct();
	void chessDetect(Mat&, vector<Point2f>&);//������̸�ͼ���ϵĽǵ�
	void getWorldPoint(vector<Point3f>&,int);//��ȡÿ���ǵ��Ӧ�����������,�궨���ʹ�õ�
	void getWorldPoint(vector<Point3f>&);//�궨�ڲεĻ�ȡ���̵Ľǵ�
	vector<Point2f> LaserDetect(Mat&);  //��ȡ�����
	void drawLine(Mat&, Vec4f); //���ݻ�ȡ��ֱ�߷��̣��ҵ������˵����ֱ��
	Point2f getlinePoint(Vec4f, Vec4f);//��������ֱ�߷��̻�ȡֱ�߽���
	void getCrossWorld(Point3f&, vector<Point3f>, vector<Point2f>, Point2f,int);
	vector<Point3f> getLaserPoint(Mat&, Mat&, vector<Point3f>, vector<Point2f>,int);
	vector<Point2f> StructLaserDetect(Mat& ,int);//��ά�ؽ��Ĺ������
	void fitLineRansac(const vector<Point2f>& points, Vec4f& line, int , double);
	void getCarmeraXyz();
	void getLaserXYZ();
	Mat fitPlane(const Mat);
	vector<Mat> ChessSrc;   //�궨�ڲε�ͼƬ
	vector<Mat> LaserImage;   //�궨��ε�ͼƬ
	vector<Mat>  LaserImagej;  //�궨��ƽ�淽�̵�ͼƬ
	int gridSize;  //ÿ�����̸���С����λmm
	int col, row;  //���̸���������
	int initRow, initCol;  //��������ϵԭ�����ڵ�����+1��������
	Mat K;//�ڲξ���
	Mat CRT;//��ξ���
	Mat CR;//��ת����
	Mat CT;//ƽ������
	Mat LaserPlane;//��ƽ�淽�� Y=A*x+B*z+k
	vector <Mat> RoationMat;
private:

};
bool cmpy(cv::Point const& a, cv::Point const& b)
{
	if (a.x == b.x) return a.y < b.y;
	else
		return a.x < b.x;
}
Calibrate::Calibrate(Mat k, vector<Mat> chessimg, vector<Mat> laserimg, vector<Mat>  laserimgj,int Col, int Row, int GridSize, int InitRow = 0, int InitCol = 0) {
	ChessSrc = chessimg;
	LaserImage = laserimg;
	LaserImagej = laserimgj;
	col = Col;
	row = Row;
	gridSize = GridSize;
	initRow = InitRow;
	initCol = InitCol;
	K = k;
}
Calibrate::~Calibrate() {

}
void Calibrate::chessDetect(Mat& img, vector<Point2f>& corners) {
	Size board_size = Size(col, row);
	Mat gray;
	cvtColor(img, gray, COLOR_BGR2GRAY);
	bool patternfound = findChessboardCorners(img, board_size, corners);
	if (!patternfound) {return;}
	else {//�����ؾ�ȷ��
		cornerSubPix(gray, corners, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
	}
    if (corners[0].y<corners[1].y) reverse(corners.begin(), corners.end());

	//for (int i = 0; i < corners.size(); i++)
	//{
	//	circle(img, corners[i], 2, cv::Scalar(255, 0, 255));
	//	putText(img, to_string(i), corners[i], FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 1, 2);
	//	//cout << corners[i] << "|";
	//}
}
void Calibrate::getWorldPoint(vector<Point3f>& worldPt,int t) {
	Mat roate1 = Mat::zeros(Size(3, 3),CV_32FC1);//��y����ת90��
	roate1.at<float>(0, 0) = 0;roate1.at<float>(0, 1) = 0;roate1.at<float>(0, 2) = 1;
	roate1.at<float>(1, 0) = 0;roate1.at<float>(1, 1) = 1;roate1.at<float>(1, 2) = 0;
	roate1.at<float>(2, 0) = -1;roate1.at<float>(2, 1) = 0;roate1.at<float>(2, 2) = 0;
	Mat roate2 = Mat::zeros(Size(3, 3), CV_32FC1);//��x����ת��ʱ��45��
	double Rad_to_deg = 45.0 / atan(1.0);  //���ȵ��Ƕȱ���
	double angle = 45 / Rad_to_deg; //��ת����
	roate2.at<float>(0, 0) = 1; roate2.at<float>(0, 1) = 0; roate2.at<float>(0, 2) = 0;
	roate2.at<float>(1, 0) = 0; roate2.at<float>(1, 1) = float(cos(angle)); roate2.at<float>(1, 2) = float(sin(angle));
	roate2.at<float>(2, 0) = 0; roate2.at<float>(2, 1) = float(-sin(angle)); roate2.at<float>(2, 2) = float(cos(angle));
	Point3f pt;
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			//�ڱ궨���ʱ�������̸�ֱ������ƽ̨�ϣ���λ��XOY��
			//ʹĳһ��λ����������ϵԭ���ϣ�ͨ�������̸�����һ�У�
			//�����ڽǵ���ʱ�����һ�в���⣬���˳���Ǵ����ϵ�����
			//���������Ǵ����µ�����
			pt = Point3f((initRow - i) * gridSize, (initCol - j) * gridSize, 0.0);
			if (t == 1)
			{ 
				Mat dst = Mat::zeros(Size(3, 1), CV_32FC1);
				Mat m = Mat::zeros(3, 1, CV_32FC1);//����ط������⻹û���
				m.at<float>(0, 0) = pt.x; m.at<float>(1, 0) = pt.y; m.at<float>(2, 0) = pt.z;
				dst = roate1 * m;
				pt.x = dst.at<float>(0, 0); pt.y = dst.at<float>(1, 0); pt.z = dst.at<float>(2, 0);
			}
			if (t == 2)
			{
				Mat dst = Mat::zeros(Size(3, 1), CV_32FC1);
				Mat m = Mat::zeros(3, 1, CV_32FC1);//����ط������⻹û���
				m.at<float>(0, 0) = pt.x; m.at<float>(1, 0) = pt.y; m.at<float>(2, 0) = pt.z;
				dst = roate2 * m;
				pt.x = dst.at<float>(0, 0); pt.y = dst.at<float>(1, 0); pt.z = dst.at<float>(2, 0);
			}
			worldPt.push_back(pt);
		}
	}
}
void Calibrate::getWorldPoint(vector<Point3f>& worldPt) { //�ڲεĻ�������ķ���
	Point3f pt;
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			pt = Point3f(i* gridSize, j* gridSize, 0.0);//�����б궨�Ǵ�(0,0)��ʼ��
			worldPt.push_back(pt);
		}
	}
}
vector<Point2f> Calibrate::StructLaserDetect(Mat& img,int flag) {
	int threshold = 130;//��ֵ��Ϊ0
	if (flag == 1) threshold = 0;
	Steger Image = Steger(img, threshold);//������ֵ
	Image.Init();
	vector<Point2f> Pt;
	Image.StructFindCenterLine(Pt);
	for (int k = 0; k < Pt.size(); k++) {
		circle(img, Pt[k], 1, Scalar(0, 0, 255), -1);
	}
	return Pt;
}
void Calibrate::drawLine(Mat& img, Vec4f current) {
	//current[a,b,x0,y0]  k=b/a;y=k*(x-x0)+y0
	Point2f point1 = Point2f(0, current[3] - current[2] * (current[1] / current[0]));
	Point2f point2 = Point2f(img.cols, current[3] + (img.cols - current[2]) * (current[1] / current[0]));
	line(img, point1, point2, Scalar(0, 0, 255), 1);
	//�������ϻ���
}
Point2f Calibrate::getlinePoint(Vec4f lineParam1, Vec4f lineParam2) {
	float k1 = lineParam1[1] / lineParam1[0];
	float b1 = lineParam1[3] - k1 * lineParam1[2];
	float k2 = lineParam2[1] / lineParam2[0];
	float b2 = lineParam2[3] - k2 * lineParam2[2];
	float x = (b1 - b2) / (k2 - k1);
	Point2f point = Point2f(x, k1 * x + b1);
	return point;
}
//��ȡ��������ϵ��XOY�潻������,���Ǹ�����������
void Calibrate::getCrossWorld(Point3f& Wp, vector<Point3f> world, vector<Point2f> img, Point2f p,int t) {
		float mp_mn = (p.x - img[0].x) / (img[1].x - img[0].x);
		float Py = world[0].y - (world[0].y - world[1].y) * mp_mn;
		if (world[0].x != world[1].x)
			return;
		Wp = Point3f(world[0].x, Py, world[0].z);
}
void Calibrate::fitLineRansac(const vector<Point2f>& points, Vec4f& line, int iterations = 1000, double sigma = 10.)
{
	unsigned int n = points.size();//n�ǵ������
	if (n < 2) {
		cout << n << "���������" << endl;
		return;
	}
	RNG rng;  //���������
	double bestScore = -1.;
	for (int k = 0; k < iterations; k++) {
		int i1 = 0, i2 = 0;
		while (i1 == i2) {
			i1 = rng(n);  //�������������
			i2 = rng(n);
		}
		const Point2f& p1 = points[i1];
		const Point2f& p2 = points[i2];
		Point2f dp = p2 - p1;//ֱ�ߵķ�������
		dp *= 1. / norm(dp); //ֱ�ߵĵ�λ����
		double score = 0;
		for (int i = 0; i < n; i++)
		{
			Point2f v = points[i] - p1;
			double d = v.y * dp.x - v.x * dp.y; //�����ʵ�ǵ㵽ֱ�ߵľ���
			//����a��b���/����b����.||b||=1./norm(dp)
			//score += exp(-0.5*d*d/(sigma*sigma));//���巽ʽ��һ��
			                                   
			if (fabs(d) < sigma)
				score += 1;
		}
		if (score > bestScore) {
			line = Vec4f(dp.x, dp.y, p1.x, p1.y);
			bestScore = score;
		}
	}
}
vector<Point3f> Calibrate::getLaserPoint(Mat& img, Mat& Laser, vector<Point3f> worldPt, vector<Point2f> imgPt,int t) {
	vector<Point2f> LaserPt = StructLaserDetect(Laser,0);  //������� ���Ҫ�Ĺ������Ļ��͵�����
	//0��ֵΪ130,ƽ�淽�̼�����
	Vec4f LaserLine;   //��ȡ�������ڵ�ֱ��
	fitLineRansac(LaserPt, LaserLine,1000,1);
	drawLine(Laser, LaserLine);//�ڹ����ϻ���
	drawLine(img, LaserLine);//�������ϻ���������
	vector<Point2f>* temp = new vector<Point2f>[row];  //������̸���ÿ�����ߵĵ�
	vector<Point2f>* imgNode = new vector<Point2f>[row];  //���ͼ����ÿ�����ߵ������˵�
	vector<Point3f>* worldNode = new vector<Point3f>[row];  //�����������ϵ��ÿ�����ߵĵ������˵�
	int k = 0;
	for (int i = 0; i < imgPt.size(); i++) {//��0��ʼ��
		if (k >= row)continue;
		temp[k].push_back(imgPt[i]);
		if (i % col == 0) {
			imgNode[k].push_back(imgPt[i]);
			worldNode[k].push_back(worldPt[i]);
		}
		if ((i + 1) % col == 0) {
			imgNode[k].push_back(imgPt[i]);
			worldNode[k].push_back(worldPt[i]);
			k++;
		}
	}
	vector<Point2f> crossPoint;
	vector<Point3f> WorldCrossPoint;
	for (int i = 0; i < row; i++) {
		Vec4f current;//cv::Vec4f ���������ǰ�벿�ָ�������ֱ�ߵķ���
		              //����벿�ָ�������ֱ���ϵ�һ�㣨��ͨ����˵�ĵ�бʽֱ�ߣ�
		fitLine(temp[i], current, DIST_L2, 0, 0.01, 0.01);  //������̸�ÿ������
		//fitLineRansac(temp[i], LaserLine, 1000, 1);
		Point2f point = getlinePoint(LaserLine, current); //��ȡÿ�����뼤��Ľ���
		Point3f WP;
		getCrossWorld(WP, worldNode[i], imgNode[i], point,t); //��ȡ�������������
		drawLine(img, current);
		circle(img, point, 2, Scalar(0, 255, 255), -1);
		crossPoint.push_back(point);
		WorldCrossPoint.push_back(WP);
	}
	delete[] temp;
	delete[] imgNode;
	delete[] worldNode;
	return WorldCrossPoint;
}
/* ��С�������ƽ�棬ƽ�淽�̣�Ax+By+Cz=D */
Mat Calibrate::fitPlane(const Mat points) {
	//cout <<"points.size"<<points.size() << endl;
	int rows = points.rows;//��
	int cols = points.cols;//��
	int  n= points.rows;
	float xi, yi, zi;
	double chisq;
	gsl_matrix *X, *cov;
	gsl_vector *y, *w, *c;
	//��ʼ��X,y,w,c,cov�Ȳ���������ռ�
	X = gsl_matrix_alloc(n, 3);
	y = gsl_vector_alloc(n);//
	w = gsl_vector_alloc(n);
	c = gsl_vector_alloc(3);//��������
	cov = gsl_matrix_alloc(3, 3);

	for (int i = 0; i < rows; i++)
	{
		xi = points.at<float>(i,0);
		yi = points.at<float>(i, 1);
		zi = points.at<float>(i, 2);
		//cout << xi << "," << yi << ", " << zi << endl;
		gsl_matrix_set(X, i, 0, 1.0);//����ֵ1����X��i,0��
		gsl_matrix_set(X, i, 1, xi); //����ֵxi����X��i,1��
		gsl_matrix_set(X, i, 2, zi); //����ֵxi*xi����X(i,2)
		gsl_vector_set(y, i, yi);//��������
	}
		gsl_multifit_linear_workspace * work
			= gsl_multifit_linear_alloc(n, 3);//ʹ����Ϻ����ĵ�һ�����������к����Ŀռ�
		gsl_multifit_linear(X, y, c, cov,&chisq, work);
		gsl_multifit_linear_free(work); //�ͷſռ�

		printf("# best fit: Y = %g + %g X + %gZ\n",
			C(0), C(1), C(2));
	Mat plane = Mat::zeros(4, 1, CV_32FC1);
	plane.at<float>(0, 0) = 1; //Z��ϵ��
	plane.at<float>(1, 0) = C(1); //X��ϵ��
	plane.at<float>(2, 0) = C(2); //Y��ϵ��
	plane.at<float>(3, 0) = C(0); //k��ֵ
	gsl_matrix_free(X);
	gsl_vector_free(y);
	gsl_vector_free(c);
	gsl_matrix_free(cov);
	return plane;
}
void Calibrate::calibrateNCamera()//�궨�ڲ�
{
	int sum = 0;
	vector<vector<Point2f>> AllChessPoint;
    vector<vector<Point3f>> AllObjectPoint;
	for (int t = 0; t < ChessSrc.size(); t++)
	{
		vector<Point2f> chessPoint;
		chessDetect(ChessSrc[t], chessPoint);
		if (chessPoint.size() !=row * col){sum++;continue;}
		for (int i = 0; i < chessPoint.size(); i++)
		{
			circle(ChessSrc[t], chessPoint[i], 2, cv::Scalar(255, 0, 255));
			putText(ChessSrc[t], to_string(i), chessPoint[i], FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 1, 2);
		}
		AllChessPoint.push_back(chessPoint);	
		vector<Point3f> worldPoint;
		getWorldPoint(worldPoint);
		AllObjectPoint.push_back(worldPoint);
	}
	Mat cameraMatirx, distCoeffs;
	vector<cv::Mat> rvecs, tvecs, rvecs2, tvecs2;
	calibrateCamera(AllObjectPoint, AllChessPoint,ChessSrc[0].size(), cameraMatirx, distCoeffs,rvecs2,tvecs2, CALIB_FIX_ASPECT_RATIO);
	cout << "һ���ܾ���" << sum << "������ͼƬ" << endl;
	cout <<"������ڲξ���Ϊ"<<endl<< cameraMatirx << endl;
	cout << "�ڲα궨���" << endl;
	K = cameraMatirx;//K�����ڲξ���
	imshow("wcg",ChessSrc[0]);
}
void Calibrate::calibrateWCamera(int wcol, int wrow) //�궨���
{
	col = wcol;
	row = wrow;
	vector<Point2f> AllChessPoint;  //���е����̸��
	vector<Point3f> AllWorldPoint;  //���̸���Ӧ����������ϵ�㣬���ڼ������
	int n = LaserImage.size();
	for (int t = 0; t < n; t++) {
		cout << t<<endl;
		vector<Point2f> chessPoint;//���̽ǵ��ͼ������b
		chessDetect(LaserImage[t], chessPoint);
		vector<Point3f> worldPoint;
		getWorldPoint(worldPoint, t);//���̽ǵ���������
		AllChessPoint.insert(AllChessPoint.end(), chessPoint.begin(), chessPoint.end());                //size=n*col*row
		AllWorldPoint.insert(AllWorldPoint.end(), worldPoint.begin(), worldPoint.end());                //size=n*col*row
	}
	Mat R, T;
	solvePnPRansac(AllWorldPoint, AllChessPoint, K, noArray(), R, T);  //pnp���r,t
	Rodrigues(R, R);
	Mat Rt;
	hconcat(R, T, Rt);
	CR = R;
	CRT = Rt;
	cout << "��ξ���" << Rt << endl;
	cout << "��α궨���" << endl;
	}
void Calibrate::planecalibration(int wcol, int wrow)
{
	if (LaserImage.size() != LaserImagej.size())
	{
		cout << "��ƽ���󲻳�" << endl;
		return ;
	}
	col = wcol;
	row = wrow;
	vector<Point2f> AllChessPoint;  //ͼ�����е����̸�������
	vector<Point3f> AllWorldPoint;  //���̸�����������
	vector<Point3f> LaserWorldPoint; //�������������
	int n = LaserImagej.size();
	for (int t = 0; t < 3; t++) {
		vector<Point2f> chessPoint;//���̽ǵ��ͼ������b
		chessDetect(LaserImage[t], chessPoint);
		if (chessPoint.size() == 0) continue;
		vector<Point3f> worldPoint;
		getWorldPoint(worldPoint, t);//���̽ǵ���������
		vector<Point3f> WorldCrossPoint = getLaserPoint(LaserImage[t], LaserImagej[t], worldPoint, chessPoint, t);
		AllChessPoint.insert(AllChessPoint.end(), chessPoint.begin(), chessPoint.end());                //size=n*col*row
		AllWorldPoint.insert(AllWorldPoint.end(), worldPoint.begin(), worldPoint.end());                //size=n*col*row
		LaserWorldPoint.insert(LaserWorldPoint.end(), WorldCrossPoint.begin(), WorldCrossPoint.end()); //size=n*row;
	}
		for (int i = 0; i <3; ++i)
		{
			string a = "imm" + to_string(i);
			string b = "i" + to_string(i);
			imshow(b, LaserImage[i]);
			imshow(a, LaserImagej[i]);
		}
	Mat PointMat = Mat(LaserWorldPoint).reshape(1, n * row);//��������ת��Ϊ������ʽ: ͨ����Ϊ1,����n,����row
														    //reshape���÷����ʼ�
	LaserPlane = fitPlane(PointMat);
	cout << "��ƽ��궨���" << endl;
}
void Calibrate::Struct()
{

	ofstream outfile("D:\\��һ�ؽ�\\��ά����\\�������XYZ.txt", ios::trunc);
	Mat Kinvert;
	invert(K, Kinvert);
	Mat Rinvert;
	vector<Mat>StructP;
	int imageCount = 0;
	string img1Dir = "D:\\��һ�ؽ�\\��ά����\\structimg\\";
	string img2Dir = "D:\\��һ�ؽ�\\��ά����\\structimgj\\";
	while (true)
	{
		string img1 = img1Dir+"image"+to_string(imageCount) + ".bmp";  //ע��ͼƬ��ʽ
		string img2 = img2Dir + "image" + to_string(imageCount) + ".bmp";  //ע��ͼƬ��ʽ
		struct stat buffer;
		if (stat(img1.c_str(), &buffer) != 0) {
			break;
		}
		Mat src1 = imread(img1);
		Mat src2 = imread(img2);
		Mat src3;
		subtract(src2, src1, src3);
		StructP.push_back(src3);
		imageCount++;
	}
	cout << "�ؽ�һ��������ͼƬ" <<imageCount<< endl;
	vector <Point3f> Allworldpoint;
	Mat b = Mat::zeros(3, 1, CV_32FC1);
	double A=LaserPlane.at<float>(1, 0);//x��ϵ��
	double B= LaserPlane.at<float>(2, 0);//z��ϵ��
	double C= LaserPlane.at<float>(3, 0);//k��ֵ
	double R1 = CR.at<double>(0, 0); double R2 = CR.at<double>(0, 1); double R3 = CR.at<double>(0, 2); double T1 = CRT.at<double>(0, 3);
	double R4 = CR.at<double>(1, 0); double R5 = CR.at<double>(1, 1); double R6 = CR.at<double>(1, 2); double T2 = CRT.at<double>(1, 3);
	double R7 = CR.at<double>(2, 0); double R8 = CR.at<double>(2, 1); double R9 = CR.at<double>(2, 2);double T3 = CRT.at<double>(2, 3);
	Mat c = Mat::zeros(3, 1, CV_64FC1);
	Mat d = Mat::zeros(3, 1, CV_64FC1);
	Mat F = Mat::zeros(3, 3, CV_64FC1);
	F.at<double>(0, 0) = R1 + A*R2; F.at<double>(0, 1) = R3 + B*R2; F.at<double>(0, 2) = T1 + R2*C;
	F.at<double>(1, 0) = R4 + A*R5; F.at<double>(1, 1) = R6+ B*R5; F.at<double>(1, 2) = T2+R5*C;
	F.at<double>(2, 0) = R7+ A*R8; F.at<double>(2, 1) =R9+ B*R8; F.at<double>(2, 2) = T3+R8*C;
	for (int i =0; i <StructP.size(); i++)
	{
		vector<Point2f> LaserPt = StructLaserDetect(StructP[i],1);
		string a = "img" + to_string(i);
		imshow(a, StructP[i]);
		vector<Point3f>Allimgpoint;
		double angle = i*M_PI/ 180;
		for (int j = 0; j < LaserPt.size(); ++j)
		{
			Point3f a = Point3f(LaserPt[j].x, LaserPt[j].y, 1);
			b.at<float>(0, 0) = a.x; b.at<float>(1, 0) = a.y; b.at<float>(2, 0) = a.z;
			c = Kinvert * Mat_<double>(b);
			c = F.inv() * c;
			c.at<double>(0, 0) = c.at<double>(0, 0)/ c.at<double>(2, 0);
			c.at<double>(1, 0) = c.at<double>(1, 0) / c.at<double>(2, 0);
			Mat d = Mat::zeros(3, 1, CV_64FC1);
			d.at<double>(0, 0) = c.at<double>(0, 0); //x
			d.at<double>(1, 0) = c.at<double>(1, 0);//y
			d.at<double>(2, 0) = 0;  //z
			d.at<double>(2, 0) = d.at<double>(1, 0)*cos(angle);
			d.at<double>(1, 0) = -d.at<double>(1, 0)*sin(angle);
			if (d.at<double>(0, 0) >= 0)
			{
				Allimgpoint.push_back(Point3f(d.at<double>(0, 0), d.at<double>(1, 0), d.at<double>(2, 0)));
			}
		}
		for (int k = 0; k < Allimgpoint.size(); k++)
		{
			outfile << Allimgpoint[k].x << " " << Allimgpoint[k].y << " " << Allimgpoint[k].z;
			outfile << "\n";

		}
	}
	outfile.close();
	cout << "�ؽ����,�鿴����" << endl;
	//��εõ��������ϵ
	//getCarmeraXyz();
	//getLaserXYZ();
	waitKey(0);
}
void Calibrate::getCarmeraXyz()
{
	vector<Point3f>wpoint;
	vector<Point3f>cpoint;
	ofstream outfile("D:\\��һ�ؽ�\\��ά����\\���λ��.txt", ios::trunc);
	Mat m = Mat::zeros(3, 1, CV_64FC1);
	Mat n = Mat::zeros(3, 1, CV_64FC1);
	double R1 = CR.at<double>(0, 0); double R2 = CR.at<double>(0, 1); double R3 = CR.at<double>(0, 2); double T1 = CRT.at<double>(0, 3);
	double R4 = CR.at<double>(1, 0); double R5 = CR.at<double>(1, 1); double R6 = CR.at<double>(1, 2); double T2 = CRT.at<double>(1, 3);
	double R7 = CR.at<double>(2, 0); double R8 = CR.at<double>(2, 1); double R9 = CR.at<double>(2, 2); double T3 = CRT.at<double>(2, 3);
	for (int i = 0; i < 100; ++i)//�������
		wpoint.push_back(Point3f(i, 0, 0));
	for (int i = 0; i < 100; ++i)
		wpoint.push_back(Point3f(0, i, 0));
	for (int i = 0; i < 100; ++i)
		wpoint.push_back(Point3f(0, 0, i));
	for (int i = 0; i < wpoint.size(); ++i)
	{
		m.at<double>(0, 0) = wpoint[i].x - T1; m.at<double>(1, 0) = wpoint[i].y - T2; m.at<double>(2, 0) = wpoint[i].z - T3;
		n = CR.inv()*m;
		cpoint.push_back(Point3f(n.at<double>(0, 0), n.at<double>(1, 0), n.at<double>(2, 0)));
	}
	struct stat buf;

	for (int k = 0; k < cpoint.size(); ++k)
	{
		outfile << cpoint[k].x << " " << cpoint[k].y << " " << cpoint[k].z;
		outfile << "\n";
	}
	outfile.close();
	cout << "���λ��������" << endl;
}
void Calibrate::getLaserXYZ()
{
	ofstream outfile("D:\\��һ�ؽ�\\��ά����\\��ƽ��.txt", ios::trunc);
	double A=LaserPlane.at<float>(1, 0);//x��ϵ��
	double B= LaserPlane.at<float>(2, 0);//z��ϵ��
	double C= LaserPlane.at<float>(3, 0);//k��ֵ
	int sum = 0;
		for (int z = -50; z < 50; z++)
		{

			for (int x = 0; x < 100; x++)
			{
				float y = A*x+B*z+C;
				cout << x << " " << y << " " << z << endl;
				outfile << x << " " << y << " " << z;
				outfile << "\n";
			}
		}
	outfile.close();
	cout << "��ƽ��������" << endl;
}