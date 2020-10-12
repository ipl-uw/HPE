#pragma once

#include <direct.h>	// in Windows
//#include <sys/stat.h>	// in Linux
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
// only in Release mode
#include <gflags/gflags.h>
#include <openpose/headers.hpp>

////! body parts mapping for Body25 format
//const std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS {
//        {0,  "Nose"},
//        {1,  "Neck"},
//        {2,  "RShoulder"},
//        {3,  "RElbow"},
//        {4,  "RWrist"},
//        {5,  "LShoulder"},
//        {6,  "LElbow"},
//        {7,  "LWrist"},
//        {8,  "MidHip"},
//        {9,  "RHip"},
//        {10, "RKnee"},
//        {11, "RAnkle"},
//        {12, "LHip"},
//        {13, "LKnee"},
//        {14, "LAnkle"},
//        {15, "REye"},
//        {16, "LEye"},
//        {17, "REar"},
//        {18, "LEar"},
//        {19, "LBigToe"},
//        {20, "LSmallToe"},
//        {21, "LHeel"},
//        {22, "RBigToe"},
//        {23, "RSmallToe"},
//        {24, "RHeel"},
//        {25, "Background"}
//    };
////! body parts mapping for COCO format
//const std::map<unsigned int, std::string> PS_COCO_BD_PTS
//{
//	{ 0,  "Nose" },
//	{ 1,  "Neck" },
//	{ 2,  "RShoulder" },
//	{ 3,  "RElbow" },
//	{ 4,  "RWrist" },
//	{ 5,  "LShoulder" },
//	{ 6,  "LElbow" },
//	{ 7,  "LWrist" },
//	{ 8,  "RHip" },
//	{ 9,  "RKnee" },
//	{ 10, "RAnkle" },
//	{ 11, "LHip" },
//	{ 12, "LKnee" },
//	{ 13, "LAnkle" },
//	{ 14, "REye" },
//	{ 15, "LEye" },
//	{ 16, "REar" },
//	{ 17, "LEar" },
//	{ 18, "Background" }
//};
////! body parts mapping for MPI format
//const std::map<unsigned int, std::string> PS_MPI_BD_PTS
//{
//	{ 0,  "Head" },
//	{ 1,  "Neck" },
//	{ 2,  "RShoulder" },
//	{ 3,  "RElbow" },
//	{ 4,  "RWrist" },
//	{ 5,  "LShoulder" },
//	{ 6,  "LElbow" },
//	{ 7,  "LWrist" },
//	{ 8,  "RHip" },
//	{ 9,  "RKnee" },
//	{ 10, "RAnkle" },
//	{ 11, "LHip" },
//	{ 12, "LKnee" },
//	{ 13, "LAnkle" },
//	{ 14, "Chest" },
//	{ 15, "Background" }
//};
////! body parts mapping for INFANT format
//const std::map<unsigned int, std::string> PS_INFANT_BD_PTS
//{
//	{ 0,  "Nose" },
//	{ 1,  "RShoulder" },
//	{ 2,  "LShoulder" },
//	{ 3,  "RElbow" },
//	{ 4,  "LElbow" },
//	{ 5,  "RWrist" },
//	{ 6,  "LWrist" },
//	{ 7,  "RHip" },
//	{ 8,  "LHip" },
//	{ 9,  "RKnee" },
//	{ 10, "LKnee" },
//	{ 11, "RAnkle" },
//	{ 12, "LAnkle" },
//};
//! list of pairs of joint points (COCO standard)
const int PS_COCO_PRS[] = { 1,2,   1,5,   2,3,   3,4,   5,6,   6,7,   1,8,   8,9,   9,10,  1,11,  11,12, 12,13,  1,0,   0,14, 14,16,  0,15, 15,17 };
//! list of pairs of joint points (MPI standard)
const int PS_MPI_PRS[] = { 0,1,   1,2,   2,3,   3,4,   1,5,   5,6,   6,7,   1,14,  14,8,  8,9,  9,10,  14,11, 11,12, 12,13 };
//! list of pairs of joint points (Body25 standard)
const int PS_BODY25_PRS[] = { 1,8,   1,2,   1,5,   2,3,   3,4,   5,6,   6,7,   8,9,   9,10,  10,11, 8,12,  12,13, 13,14,  1,0,   0,15, 15,17,  0,16, 16,18,   2,17,  5,18,   14,19,19,20,14,21, 11,22,22,23,11,24 };
//! list of colors for plotting joint points (COCO standard)
const int PS_INFANT_PRS[] = { 0,1, 0,2, 1,2, 1,3, 3,5, 2,4, 4,6, 1,7, 2,8, 7,8, 7,9, 9,11, 8,10, 10,12 };
const cv::Scalar PS_COCO_CLRS[] =
 {
	cv::Scalar(255,   0,  85),
	cv::Scalar(255,   0,   0),
	cv::Scalar(255,  85,   0),
	cv::Scalar(255, 170,   0),
	cv::Scalar(255, 255,   0),
	cv::Scalar(170, 255,   0),
	cv::Scalar( 85, 255,   0),
	cv::Scalar(  0, 255,   0),
	cv::Scalar(  0, 255,  85),
	cv::Scalar(  0, 255, 170),
	cv::Scalar(  0, 255, 255),
	cv::Scalar(  0, 170, 255),
	cv::Scalar(  0,  85, 255),
	cv::Scalar(  0,   0, 255),
	cv::Scalar(255,   0, 170),
	cv::Scalar(170,   0, 255),
	cv::Scalar(255,   0, 255),
	cv::Scalar( 85,   0, 255)
};
//! list of colors for plotting joint points (MPI standard)
const cv::Scalar PS_MPI_CLRS[] =
 {
	cv::Scalar(255,   0,  85),
	cv::Scalar(255,   0,   0),
	cv::Scalar(255,  85,   0),
	cv::Scalar(255, 170,   0),
	cv::Scalar(255, 255,   0),
	cv::Scalar(170, 255,   0),
	cv::Scalar(85,  255,   0),
	cv::Scalar(43,  255,   0),
	cv::Scalar(0,   255,   0),
	cv::Scalar(0,   255,  85),
	cv::Scalar(0,   255, 170),
	cv::Scalar(0,   255, 255),
	cv::Scalar(0,   170, 255),
	cv::Scalar(0,    85, 255),
	cv::Scalar(0,     0, 255)
};
//! number of joint points in COCO standard (default: 18)
#define PS_COCO_JNTPT_NUM (18)
//! number of joint points in Body25 standard (default: 25)
#define PS_BODY25_JNTPT_NUM (25)
//! number of joint points in MPI standard (default: 15)
#define PS_MPI_JNTPT_NUM (15)
//! number of joint points in INFANT standard (default: 15)
#define PS_INFANT_JNTPT_NUM (13)
//! size of Gaussian blur (default: cv::Size(5, 5))
#define GAUSS_BLR_SZ (cv::Size(5, 5))
//! window size of KLT tracking (default: cv::Size(21, 21))
#define KLT_TRK_WIN_SZ (cv::Size(21, 21))
//! aspect ratio to plot limbs in the mask (default: 0.333f)
#define PLT_MSK_LMB_AR (0.333f)
//! number of iterations for erosion in the mask (default: 20)
#define PLT_MSK_ERO_ITER_NUM (20)
//! radius of circle to plot joint points (default: 5)
#define PLT_PS_JNTPT_RAD (5)
//! thickness of line to plot links between joint points (default: 3)
#define PLT_PS_JNTLN_THK (3)
//! blending ratio to plot the mask of 2D human pose (default: 0.2f)
#define PLT_MSK_BLN_RAT (0.2f)
//! the logging level for OpenPose in the range 0-4: 1 for low priority messages and 4 for important ones (default: 3)
#define OP_LOG_LVL (3)


class CJntPt
{
public:
	CJntPt(void);
	CJntPt(cv::Point2f o2dPt, cv::Point3f o3dPt, float fPsEst2dScr);
	~CJntPt(void);

	inline cv::Point2f get2dPt(void) { return m_o2dPt; }
	inline void set2dPt(cv::Point2f o2dPt) { m_o2dPt = o2dPt; }
	inline cv::Point3f get3dPt(void) { return m_o3dPt; }
	inline void set3dPt(cv::Point3f o3dPt) { m_o3dPt = o3dPt; }
	inline float getPsEst2dScr(void) { return m_fPsEst2dScr; }
	inline void setPsEst2dScr(float fPsEst2dScr) { m_fPsEst2dScr = fPsEst2dScr; }

private:
	//! 2D coordinates of the joint point
	cv::Point2f m_o2dPt;
	//! 3D coordinates of the joint point
	cv::Point3f m_o3dPt;
	//! score of 2D pose estimation
	float m_fPsEst2dScr;
};

class CHumPs
{
public:
	CHumPs(void);
	CHumPs(int nFrmCnt, std::vector<CJntPt> voJntPt);
	CHumPs(int nFrmCnt, std::vector<CJntPt> voJntPt, std::vector<int> vnMtchFrmNum);
	~CHumPs(void);

	inline int getFrmCnt(void) { return m_nFrmCnt; }
	inline void setFrmCnt(int nFrmCnt) { m_nFrmCnt = nFrmCnt; }
	inline std::vector<CJntPt> getJntPtLs(void) { return m_voJntPt; }
	inline CJntPt getJntPt(int i) { return m_voJntPt[i]; }
	inline void addJntPt(CJntPt oJntPt) { m_voJntPt.push_back(oJntPt); }
	inline void setJntPtLs(std::vector<CJntPt> voJntPt) { m_voJntPt = voJntPt; }
	inline void resetJntPtLs(void) { std::vector<CJntPt>().swap(m_voJntPt); }
	inline int getMtchFrmNum(int i) { return m_vnMtchFrmNum[i]; }
	inline void setMtchFrmNum(int i, int nMtchFrmNum) { m_vnMtchFrmNum[i] = nMtchFrmNum; }
	inline void addMtchFrmNum(int nMtchFrmNum) { m_vnMtchFrmNum.push_back(nMtchFrmNum); }
	inline void setMtchFrmNumLs(std::vector<int> vnMtchFrmNum) { m_vnMtchFrmNum = vnMtchFrmNum; }
	inline void resetMtchFrmNumLs(void) { std::vector<int>().swap(m_vnMtchFrmNum); }

private:
	//! frame count associated with object detection
	int m_nFrmCnt;
	//! bounding box of object detection
	std::vector<CJntPt> m_voJntPt;
	//! number of matched frames to recover pose estimation
	std::vector<int> m_vnMtchFrmNum;
};

class CPsEst2d
{
public:
	CPsEst2d(void);
	~CPsEst2d(void);

	//! initializes 2D pose estimation
	void initialize(CCfg oCfg);
	//! processes 2D pose estimation
	CHumPs process(cv::Mat oImgFrmCurr, cv::Mat& oImgMsk, int nFrmCnt);
	//! outputs 2D pose estimation
	void output(cv::Mat& oImgFrm, cv::Mat oImgMsk);
	// only in Release mode
	//! preprocesses by OpenPose
	float preProcOp(void);

private:
	//! reads 2D human pose from txt file
	CHumPs rdPsEst2dTxt(int nFrmCnt);
	//! reads 2D human pose from json files
	CHumPs rdPsEst2dJson(int nFrmCnt);
	//! finds the largest human pose from a list of poses
	CHumPs fdMaxHumPs(std::vector<CHumPs> voHumPs);
	// track missing joint points
	void trkMsJtPt(cv::Mat oImgFrmCurrGry);
	//! plots the mask of 2D human pose
	void plt2dPsMsk(cv::Mat& oImgMsk);
	//! outputs text file
	void outTxt(void);
	//! plots 2D human pose
	void pltPsEst2d(cv::Mat& oImgFrm, cv::Mat oImgMsk);

	//! configuration file
	CCfg m_oCfg;
	//! current frame count
	int m_nFrmCnt;
	//! input 2D human poses
	std::ifstream m_ifsInPs2d;
	//! the previous frame in grey scale
	cv::Mat m_oImgFrmPrevGry;
	//! number of joint points corresponding to different standards
	int m_nJntPtNum;
	//! the next human pose when reading txt file
	CHumPs m_oHumPsNxt;
	//! the previous human pose in the last frame
	CHumPs m_oHumPsPrev;
	//! the estimated human pose
	CHumPs m_oHumPs;
};
