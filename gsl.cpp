//
//////拟合平面的算法:如果精度达不到就得用这个方法
////ceres
//#include <stdio.h>
//#include <gsl/gsl_multifit.h>
//# include <iostream>
//using namespace std;
//int main()
//{
//	int i, n;
//	float a[20] = { 160,140,120,100,80,60,40,20,160,140,120,100,80,60,40,20};
//	float b[20] = { 49.8347,50.3605,50.9043,51.6308,52.1318,52.8196,53.2899,53.8963};
//	float d[20] = {0,0,0,0,0,0,0,0};
//	float xi, yi, zi;
//	double chisq;
//	gsl_matrix *X, *cov;
//	gsl_vector *z, *w, *c;
//	n = 8;
//	//初始化X,y,w,c,cov等参数，申请空间
//	X = gsl_matrix_alloc(n, 3);
//	z = gsl_vector_alloc(n);//
//	w = gsl_vector_alloc(n);
//
//	c = gsl_vector_alloc(3);//三个变量
//	cov = gsl_matrix_alloc(3, 3);
//
//	for (i = 0; i < n; i++)
//	{
//		xi = a[i];
//		yi = b[i];
//		zi = d[i];
//		gsl_matrix_set(X, i, 0, 1.0);//将数值1赋给X（i,0）
//		gsl_matrix_set(X, i, 1, xi); //将数值xi赋给X（i,1）
//		gsl_matrix_set(X, i, 2, yi); //将数值xi*xi赋给X(i,2)
//		gsl_vector_set(z, i, zi);//类似上面
//		gsl_vector_set(w, i, 1);
//	}
//
//	{
//		gsl_multifit_linear_workspace * work
//			= gsl_multifit_linear_alloc(n, 3);//使用拟合函数的第一步，申请运行函数的空间
//		gsl_multifit_linear(X,  z, c, cov,
//			&chisq, work);
//		gsl_multifit_linear_free(work); //释放空间
//	}
//
//#define C(i) (gsl_vector_get(c,(i)))
//#define COV(i,j) (gsl_matrix_get(cov,(i),(j)))
//
//	{
//		printf("# best fit: Z = %g + %g X + %g Y\n",
//			C(0), C(1), C(2));
//	}
//	gsl_matrix_free(X);
//	gsl_vector_free(z);
//	gsl_vector_free(w);
//	gsl_vector_free(c);
//	gsl_matrix_free(cov);
//	system("pause");
//	//return 0;
//
//
//
////#include <stdio.h>
////#include <gsl/gsl_rng.h>
////#include <gsl/gsl_randist.h>
////#include <gsl/gsl_multifit.h>
////#include <vector>
////# include <iostream>
////using namespace std;
////
////int main()
////{
////	vector<double> X, YODY, SIGMA;
////	{
////		double x;
////		const gsl_rng_type*T;
////		gsl_rng *r;
////		gsl_rng_env_setup();
////		T = gsl_rng_default;
////		r = gsl_rng_alloc(T);
////		for (x = 0.1; x < 2; x += 0.1)
////			{
////				double y0 = exp(x);
////				double sigma = 0.1*y0;
////				double dy = gsl_ran_gaussian(r, sigma);
////				printf("%g%g%g\n", x, y0 + dy, sigma);
////				X.push_back(x);
////				YODY.push_back(y0 + dy);
////				SIGMA.push_back(sigma);
////			}
////	gsl_rng_free(r);
////	}
////	int n = X.size();
////	int i;
////	double xi, yi, ei, chisq;
////	gsl_matrix *x, *cov;
////	gsl_vector *y, *w, *c;
////
////	x = gsl_matrix_alloc(n, 3);
////	y = gsl_vector_alloc(n);
////	w = gsl_vector_alloc(n);
////	c = gsl_vector_alloc(3);
////	cov = gsl_matrix_alloc(3, 3);
////	for (i = 0; i < n; i++)
////	{
////		printf("%g%g+/-%g\n", X[i], YODY[i], SIGMA[i]);
////		gsl_matrix_set(x, i, 0, 1.0);
////		gsl_matrix_set(x, i, 1, X[i]);
////		gsl_matrix_set(x, i, 2, X[i] * X[i]);
////		gsl_vector_set(y, i, YODY[i]);
////		gsl_vector_set(w, i, 1.0 / (SIGMA[i] * SIGMA[i]));
////	}
////	{
////		gsl_multifit_linear_workspace *work
////			= gsl_multifit_linear_alloc(n, 3);
////		gsl_multifit_wlinear(x, w, y, c, cov,
////			&chisq, work);
////		gsl_multifit_linear_free(work);
////	}
////#define C(i) (gsl_vector_get(c,(i)))
////#define COV(i,j) (gsl_matrix_get(cov,(i),(j)))
////	{
////		printf("# best fit: Y =%g+%gX +%gX^2\n",
////			C(0), C(1), C(2));
////		printf("# covariance matrix:\n");
////		printf("[%+.5e,%+.5e,%+.5e\n",
////			COV(0, 0), COV(0, 1), COV(0, 2));
////		printf("%+.5e,%+.5e,%+.5e\n",
////			COV(1, 0), COV(1, 1), COV(1, 2));
////		printf("%+.5e,%+.5e,%+.5e]\n",
////			COV(2, 0), COV(2, 1), COV(2, 2));
////		printf("# chisq =%g\n", chisq);
////	}
////	gsl_matrix_free(x);
////
////	while (true);
////
////	return 0;
////}
//
//
////刘洋的拟合办法
////cout << "points.rows" << points.rows << " " << "points.cols" << points.cols << endl;//到这里没问题
////Mat centroid = Mat::zeros(1, cols, CV_32FC1); // 平均坐标 算x,y,z的平均坐标
////for (int i = 0; i < cols; i++) {
////	for (int j = 0; j < rows; j++) {
////		centroid.at<float>(0, i) += points.at<float>(j, i);
////	}
////	centroid.at<float>(0, i) /= rows;
////}
////Mat points2 = Mat::ones(rows, cols, CV_32FC1);//A矩阵
////for (int i = 0; i < rows; i++) {
////	for (int j = 0; j < cols; j++) {
////		points2.at<float>(i, j) = points.at<float>(i, j) - centroid.at<float>(0, j);
////	}
////}
////Mat A, W, U, VT;
////gemm(points2, points, 1, NULL, 0, A, GEMM_1_T);
//////A = points2;
////cout << A.size() << endl;
////SVD::compute(A, W, U, VT);  //奇异值分解
////cout << W.size() << U.size() << VT.size() << endl;
////Mat plane = Mat::zeros(cols + 1, 1, CV_32FC1);
////for (int c = 0; c < cols; c++) {
////	plane.at<float>(c, 0) = VT.at<float>(cols - 1, c);
////	plane.at<float>(cols, 0) += plane.at<float>(c, 0) * centroid.at<float>(0, c); //d
////}
////cout << plane << endl;
////return plane;
