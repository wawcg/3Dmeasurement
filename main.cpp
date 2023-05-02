ﻄ1�7#include<opencv2/opencv.hpp>
#include<iostream>
#include"Calibration.h"
#include<algorithm>
using namespace std;
using namespace cv;
int main()
{
	//cout << "��ʼ���г���"<<endl;
	string ChessSrcDir = "";//���̸�ͼ��
	string LaserDir = "";  //�ر��߼����ͼ��
	string LaserJDir = ""; //���߼����ͼ��
	vector <Mat>ChessSrc;
	vector<Mat> LaserImage;
	vector<Mat> LaserImagej;
	int imageCount = 0;
	while (true)
	{
		string chessimg = ChessSrcDir + to_string(imageCount) + ".jpg"; 
		struct stat buffer;
		if (stat(chessimg.c_str(), &buffer) != 0) {
			break;
		}
		Mat src = imread(chessimg);
		ChessSrc.push_back(src);
		imageCount++;
	}
	cout << "���̸�ͼ������" << imageCount << endl;
	if (ChessSrc.empty())return 0;
	imageCount = 0;
	while (true)
	{
		string Laser = LaserDir + to_string(imageCount) + ".jpg";  
		string Laserj = LaserJDir + to_string(imageCount) + ".jpg";  
		struct stat buffer;
		if (stat(Laserj.c_str(), &buffer) != 0|| stat(Laser.c_str(), &buffer) != 0) {
			break;
		}
		Mat src1 = imread(Laser);
		Mat src2 = imread(Laserj);
		if (src1.size() != src2.size()) continue;
		Mat src4;
		LaserImage.push_back(src1);
		LaserImagej.push_back(src2);
		imageCount++;
	}
	cout <<"����궨ͼ������:"<<imageCount<< endl;
	if (LaserImage.empty())return 0;
	for (int i = 0; i <imageCount; ++i)
		{
		string a = "image" + to_string(i);
		imshow(a, LaserImagej[i]);
		}
	Mat K = Mat::zeros(3, 3, CV_32FC1);
	int col = 9;//���̸���м���
	int row = 6;//���̸���м���
	int wcol=9;//������̸���м���
	int wrow = 6;//������̸���м���
	int gridsize = 20;//���̸�ÿһ����20mm 2cm
	int initCol = 4; //����ԭ�������̵��ĸ�λ��
    int initRow = 6;          
	Calibrate myCalib = Calibrate(K, ChessSrc, LaserImage, LaserImagej,col, row, gridsize, initRow, initCol);
	myCalib.calibrateNCamera();//�궨������ڲ�
	myCalib.calibrateWCamera(wcol,wrow);//�궨������
	myCalib.planecalibration(wcol, wrow);//�궨����ƽ��
	myCalib.Struct();//�ؽ�������ģ��
	waitKey(0);
	return 0;
}
