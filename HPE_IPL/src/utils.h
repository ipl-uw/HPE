#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define IM1 2147483563
#define IM2 2147483399
#define AM (1.0/IM1)
#define IMM1 (IM1-1)
#define IA1 40014
#define IA2 40692
#define IQ1 53668
#define IQ2 52774
#define IR1 12211
#define IR2 3791
#define NTAB 32
#define NDIV (1+IMM1/NTAB)
#define EPS1 1.2e-7
#define RNMX (1.0-EPS1)

template <typename T> T deg2rad(T deg) { return deg * (CV_PI / 180.0); }
template <typename T> T rad2deg(T rad) { return rad * (180.0 / CV_PI); }

//! generates a random double variable
static double rand2(long *idum)
{
	int j;
	long k;
	static long idum2 = 123456789;
	static long iy = 0;
	static long iv[NTAB];
	double temp;
	if (*idum <= 0) {
		if (-(*idum) < 1) *idum = 1;
		else *idum = -(*idum);
		idum2 = (*idum);
		for (j = NTAB + 7; j >= 0; j--) {
			k = (*idum) / IQ1;
			*idum = IA1*(*idum - k*IQ1) - k*IR1;
			if (*idum < 0) *idum += IM1;
			if (j < NTAB) iv[j] = *idum;
		}
		iy = iv[0];
	}
	k = (*idum) / IQ1;
	*idum = IA1*(*idum - k*IQ1) - k*IR1;
	if (*idum < 0) *idum += IM1;
	k = idum2 / IQ2;
	idum2 = IA2*(idum2 - k*IQ2) - k*IR2;
	if (idum2 < 0) idum2 += IM2;
	j = iy / NDIV;
	iy = iv[j] - idum2;
	iv[j] = *idum;
	if (iy < 1) iy += IMM1;
	if ((temp = AM*iy) > RNMX) return RNMX;
	else return temp;
}

//! generates a random number
static double get_rand_num(double max, double min, long seed)
{
	double rand = rand2(&seed);
	double duration = max - min;
	return min + rand*duration;
}

//! prints a matrix of double
static void prtMatDbl(cv::Mat oMat)
{
	for (int y = 0; y < oMat.rows; y++)
	{
		for (int x = 0; x < oMat.cols; x++)
		{
			std::printf("%.3f ", oMat.at<double>(y, x));
		}
		std::printf("\n");
	}
}

//! calculates the cosine value of two vectors in 3D
static double calcCosVec(cv::Point3f oVec0, cv::Point3f oVec1)
{
	return ((oVec0.x * oVec1.x) + (oVec0.y * oVec1.y) + (oVec0.z * oVec1.z)) / (cv::norm(oVec0) * cv::norm(oVec1));
}

//! calculates the cosine value of two vectors in 3D
static double calcCosVec(cv::Vec3f oVec0, cv::Point3f oVec1)
{
	return ((oVec0[0] * oVec1.x) + (oVec0[1] * oVec1.y) + (oVec0[2] * oVec1.z)) / (cv::norm(oVec0) * cv::norm(oVec1));
}

//! translates a 3D point in world coordinate to a 3D point in camera coordinate
static cv::Point3f tnt3dPtW2C(cv::Point3f o3dPtW, cv::Matx34d oCamExtMatx)
{
	cv::Matx33d oCamRotMatx(oCamExtMatx(0, 0), oCamExtMatx(0, 1), oCamExtMatx(0, 2),
		oCamExtMatx(1, 0), oCamExtMatx(1, 1), oCamExtMatx(1, 2), 
		oCamExtMatx(2, 0), oCamExtMatx(2, 1), oCamExtMatx(2, 2));
	cv::Matx31d oCamTntMatx(oCamExtMatx(0, 3), oCamExtMatx(1, 3), oCamExtMatx(2, 3));
	cv::Matx31d o3dPtWMatx(o3dPtW.x, o3dPtW.y, o3dPtW.z);

	cv::Matx31d o3dPtCMatx = (oCamRotMatx * o3dPtWMatx) + oCamTntMatx;

	return cv::Point3f(o3dPtCMatx(0), o3dPtCMatx(1), o3dPtCMatx(2));
}

//! translates a 3D point in camera coordinate to a 3D point in world coordinate
static cv::Point3f tnt3dPtC2W(cv::Point3f o3dPtC, cv::Matx34d oCamExtMatx)
{
	cv::Matx33d oCamRotMatx(oCamExtMatx(0, 0), oCamExtMatx(0, 1), oCamExtMatx(0, 2),
		oCamExtMatx(1, 0), oCamExtMatx(1, 1), oCamExtMatx(1, 2),
		oCamExtMatx(2, 0), oCamExtMatx(2, 1), oCamExtMatx(2, 2));
	cv::Matx31d oCamTntMatx(oCamExtMatx(0, 3), oCamExtMatx(1, 3), oCamExtMatx(2, 3));
	cv::Matx31d o3dPtCMatx(o3dPtC.x, o3dPtC.y, o3dPtC.z);

	cv::Matx31d o3dPtWMatx = oCamRotMatx.inv() * (o3dPtCMatx - oCamTntMatx);

	return cv::Point3f(o3dPtWMatx(0), o3dPtWMatx(1), o3dPtWMatx(2));
}

//! projects a 3D point in world coordinate to 2D
static cv::Point2f proj3dPtW22(cv::Point3f o3dPtW, cv::Mat oCamIntMat, cv::Matx34d oCamExtMatx)
{
	cv::Matx33d oCamIntMatx(oCamIntMat.at<double>(0, 0), oCamIntMat.at<double>(0, 1), oCamIntMat.at<double>(0, 2),
		oCamIntMat.at<double>(1, 0), oCamIntMat.at<double>(1, 1), oCamIntMat.at<double>(1, 2),
		oCamIntMat.at<double>(2, 0), oCamIntMat.at<double>(2, 1), oCamIntMat.at<double>(2, 2));
	cv::Matx34d oCamProjMatx = oCamIntMatx * oCamExtMatx;
	cv::Matx41d o3dPtWMatx(o3dPtW.x, o3dPtW.y, o3dPtW.z, 1.0);
	cv::Matx31d o2dPtMatx = oCamProjMatx * o3dPtWMatx;

	return cv::Point2f(o2dPtMatx(0) / o2dPtMatx(2), o2dPtMatx(1) / o2dPtMatx(2));
}

//! projects a 3D point in camera coordinate to 2D
static cv::Point2f proj3dPtC22(cv::Point3f o3dPtC, cv::Mat oCamIntMat, cv::Matx34d oCamExtMatx)
{
	return proj3dPtW22(tnt3dPtC2W(o3dPtC, oCamExtMatx), oCamIntMat, oCamExtMatx);
}

//! back projects a 2D point to a 3D point in camera coordinate
static cv::Point3f bkproj2dPt22C(cv::Point2f o2dPt, double fDep, cv::Mat oCamIntMat)
{
	cv::Matx33d oCamIntMatx(oCamIntMat.at<double>(0, 0), oCamIntMat.at<double>(0, 1), oCamIntMat.at<double>(0, 2),
		oCamIntMat.at<double>(1, 0), oCamIntMat.at<double>(1, 1), oCamIntMat.at<double>(1, 2),
		oCamIntMat.at<double>(2, 0), oCamIntMat.at<double>(2, 1), oCamIntMat.at<double>(2, 2));
	cv::Matx33d oCamInMatxInv = oCamIntMatx.inv();
	cv::Matx31d o2dPtMatx(o2dPt.x, o2dPt.y, 1.0f);

	cv::Matx31d o3dPtCMatx = (fDep / ((oCamInMatxInv(2, 0) * o2dPtMatx(0)) + (oCamInMatxInv(2, 1) * o2dPtMatx(1)) + oCamInMatxInv(2, 2))) * oCamInMatxInv * o2dPtMatx;

	return cv::Point3f(o3dPtCMatx(0), o3dPtCMatx(1), o3dPtCMatx(2));
}

//! back projects a 2D point to a 3D point in world coordinate
static cv::Point3f bkproj2dPt22W(cv::Point2f o2dPt, double fDep, cv::Mat oCamIntMat, cv::Matx34d oCamExtMatx)
{
	return tnt3dPtC2W(bkproj2dPt22C(o2dPt, fDep, oCamIntMat), oCamExtMatx);
}

//! converts camera extrinsic matrix to rotation vector and translation matrix
static void cvtExtMatx2RotVecTntMat(cv::Matx34d oCamExtMatx, cv::Mat& oCamRotVec, cv::Mat& oCamTntMat)
{
	cv::Mat oCamRotMat = cv::Mat::eye(3, 3, CV_64F);
	oCamRotMat.at<double>(0, 0) = oCamExtMatx(0, 0);
	oCamRotMat.at<double>(0, 1) = oCamExtMatx(0, 1);
	oCamRotMat.at<double>(0, 2) = oCamExtMatx(0, 2);
	oCamRotMat.at<double>(1, 0) = oCamExtMatx(1, 0);
	oCamRotMat.at<double>(1, 1) = oCamExtMatx(1, 1);
	oCamRotMat.at<double>(1, 2) = oCamExtMatx(1, 2);
	oCamRotMat.at<double>(2, 0) = oCamExtMatx(2, 0);
	oCamRotMat.at<double>(2, 1) = oCamExtMatx(2, 1);
	oCamRotMat.at<double>(2, 2) = oCamExtMatx(2, 2);
	cv::Rodrigues(oCamRotMat, oCamRotVec);

	oCamTntMat = cv::Mat::zeros(3, 1, CV_64F);
	oCamTntMat.at<double>(0) = oCamExtMatx(0, 3);
	oCamTntMat.at<double>(1) = oCamExtMatx(1, 3);
	oCamTntMat.at<double>(2) = oCamExtMatx(2, 3);
}

//! converts rotation vector and translation vector to camera extrinsic matrix
static void cvtRotVecTntMat2ExtMatx(cv::Mat oCamRotVec, cv::Mat oCamTntMat, cv::Matx34d& oCamExtMatx)
{
	cv::Mat oCamRotMat;
	cv::Rodrigues(oCamRotVec, oCamRotMat);
	oCamExtMatx(0, 0) = oCamRotMat.at<double>(0, 0);
	oCamExtMatx(0, 1) = oCamRotMat.at<double>(0, 1);
	oCamExtMatx(0, 2) = oCamRotMat.at<double>(0, 2);
	oCamExtMatx(1, 0) = oCamRotMat.at<double>(1, 0);
	oCamExtMatx(1, 1) = oCamRotMat.at<double>(1, 1);
	oCamExtMatx(1, 2) = oCamRotMat.at<double>(1, 2);
	oCamExtMatx(2, 0) = oCamRotMat.at<double>(2, 0);
	oCamExtMatx(2, 1) = oCamRotMat.at<double>(2, 1);
	oCamExtMatx(2, 2) = oCamRotMat.at<double>(2, 2);
	oCamExtMatx(0, 3) = oCamTntMat.at<double>(0);
	oCamExtMatx(1, 3) = oCamTntMat.at<double>(1);
	oCamExtMatx(2, 3) = oCamTntMat.at<double>(2);
}

//! converts translation matrix to translation vector
static void cvtTntMat2Vec(cv::Mat oCamRotVec, cv::Mat oCamTntMat, cv::Mat& oCamTntVec)
{
	cv::Mat oCamRotMat;
	cv::Rodrigues(oCamRotVec, oCamRotMat);
	oCamTntVec = -(oCamRotMat.inv() * oCamTntMat);
}

//! converts translation vector to translation matrix
static void cvtTntVec2Mat(cv::Mat oCamRotVec, cv::Mat oCamTntVec, cv::Mat& oCamTntMat)
{
	cv::Mat oCamRotMat;
	cv::Rodrigues(oCamRotVec, oCamRotMat);
	oCamTntMat = oCamRotMat * (-oCamTntVec);
}

//! gets translation vector from camera extrinsic matrix
static cv::Mat getTntVecInExtMatx(cv::Matx34d oCamExtMatx)
{
	cv::Mat oCamRotVec, oCamTntMat, oCamTntVec;
	cvtExtMatx2RotVecTntMat(oCamExtMatx, oCamRotVec, oCamTntMat);
	cvtTntMat2Vec(oCamRotVec, oCamTntMat, oCamTntVec);
	return oCamTntVec;
}