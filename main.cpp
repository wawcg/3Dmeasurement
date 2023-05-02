锘�#include<opencv2/opencv.hpp>
#include<iostream>
#include"Calibration.h"
#include<algorithm>
using namespace std;
using namespace cv;
int main()
{
	//cout << "开始运行程序"<<endl;
	string ChessSrcDir = "";//棋盘格图像
	string LaserDir = "";  //关闭线激光的图像
	string LaserJDir = ""; //打开线激光的图像
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
	cout << "棋盘格图像数量" << imageCount << endl;
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
	cout <<"激光标定图像数量:"<<imageCount<< endl;
	if (LaserImage.empty())return 0;
	for (int i = 0; i <imageCount; ++i)
		{
		string a = "image" + to_string(i);
		imshow(a, LaserImagej[i]);
		}
	Mat K = Mat::zeros(3, 3, CV_32FC1);
	int col = 9;//棋盘格宽有几格
	int row = 6;//棋盘格高有几格
	int wcol=9;//外参棋盘格宽有几格
	int wrow = 6;//外参棋盘格高有几格
	int gridsize = 20;//棋盘格每一个是20mm 2cm
	int initCol = 4; //世界原点在棋盘的哪个位置
    int initRow = 6;          
	Calibrate myCalib = Calibrate(K, ChessSrc, LaserImage, LaserImagej,col, row, gridsize, initRow, initCol);
	myCalib.calibrateNCamera();//标定相机的内参
	myCalib.calibrateWCamera(wcol,wrow);//标定相机外参
	myCalib.planecalibration(wcol, wrow);//标定激光平面
	myCalib.Struct();//重建出整个模型
	waitKey(0);
	return 0;
}
