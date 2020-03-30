#pragma once

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include "Cfg.h"
#include "utils.h"

//! size of Gaussian blur (default: cv::Size(5, 5))
#define GAUSS_BLR_SZ (cv::Size(5, 5))
//! flag of non-maximum suppresion for the detection of FAST feature points (default: true)
#define FAST_NMS_FLG (true)
//! window size of KLT tracking (default: cv::Size(21, 21))
#define KLT_TRK_WIN_SZ (cv::Size(21, 21))
//! number of iterations for RANSAC (default: 100)
#define RANSAC_ITER_NUM (100)
//! threshold of reprojection error for RANSAC (default: 8.0)
#define RANSAC_REPROJ_ERR_THLD (8.0)
//! scalar of color to plot feature points (default: cv::Scalar(0, 255, 0))
#define PLT_KEYPT_CLR (cv::Scalar(0, 255, 0))

class CVisOdom
{
public:
	CVisOdom(void);
	~CVisOdom(void);

	//! initializes visual odometry
	void initialize(CCfg oCfg);
	//! processes visual odometry
	cv::Matx34d process(cv::Mat oImgFrmCurr, cv::Mat oImgMsk, int nFrmCnt);
	//! updates camera extrinsic matrices
	void updCamExtMat(cv::Matx34d oCamExtMat);
	//! outputs visual odometry
	void output(cv::Mat& oImgFrm);

private:
	//! reads camera parameters
	void rdCamParam(int nFrmCntCurr);
	//! runs visual odometry
	void rnVisOdom(cv::Mat oImgFrmCurr, cv::Mat oImgMsk, int nFrmCnt);
	//! removes feature points by mask
	void rmvFeatPtMsk(cv::Mat oImgMsk, std::vector<cv::KeyPoint>& voFeatPt, std::vector<cv::Point2f>& vo2dPt, bool bRmv3dPtFlg = false);
	//! erases untracked feature points
	void ersUntrkFeatPt(std::vector<cv::Point2f>& vo2dPtTrk, std::vector<uchar>& vuStat, std::vector<float>& vfErr);
	//! combines detected and tracked feature points
	void combDetTrkFeatPt(std::vector<cv::KeyPoint>& voFeatPt, std::vector<cv::Point2f>& vo2dPt,
		std::vector<cv::KeyPoint> voFeatPtTrk, std::vector<cv::Point2f> vo2dPtTrk);
	//! outputs text file
	void outTxt(void);

	//! configuration file
	CCfg m_oCfg;
	//! current frame count
	int m_nFrmCnt;
	//! flag of solving PnP problem using RANSAC
	bool m_bRansac;
	//! input camera parameters
	std::ifstream m_ifsInCamParam;
	//! previous camera rotation matrix
	cv::Mat m_oCamRotMatPrev;
	//! previous camera translation matrix
	cv::Mat m_oCamTntMatPrev;
	//! updated camera rotation matrix
	cv::Mat m_oCamRotMatUpd;
	//! updated camera translation matrix
	cv::Mat m_oCamTntMatUpd;
	//! updated camera extrinsic matrix
	cv::Matx34d m_oCamExtMatxUpd;
	//! the previous frame in grey scale
	cv::Mat m_oImgFrmGryPrev;
	//! list of feature points in previous frame
	std::vector<cv::KeyPoint> m_voFeatPtPrev;
	//! list of 2D points in previous frame
	std::vector<cv::Point2f> m_vo2dPtPrev;
	//! list of tracked feature points in previous frame
	std::vector<cv::KeyPoint> m_voFeatPtTrkPrev;
	//! list of tracked 2D points iin previous frame
	std::vector<cv::Point2f> m_vo2dPtTrkPrev;
	//! list of 3D points
	std::vector<cv::Point3f> m_vo3dPt;
	//! FAST feature detector
	cv::Ptr<cv::FastFeatureDetector> m_poFastFeatDet;
};
