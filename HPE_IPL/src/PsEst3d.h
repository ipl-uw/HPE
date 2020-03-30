#pragma once

#include "VisOdom.h"
#include "PsEst2d.h"

//! range of cosine value between neck and shoulder (default: cv::Vec2d(0.0, 0.5))
#define COS_VEC_RNG_NK_SHLDR (cv::Vec2d(0.0, 0.5))
//! range of cosine value between shoulder and hip (default: cv::Vec2d(0.0, 0.7071068))
#define COS_VEC_RNG_SHLDR_HP (cv::Vec2d(0.0, 0.7071068))
//! threshold of cosine value between normal vector of frontal body and a joint pair (default: -0.3333333)
#define COS_VEC_THLD_BDY_PLN_JNT_PR (-0.3333333)

class COptParam
{
public:
	COptParam(void);
	COptParam(cv::Vec3d ovRotVec, cv::Vec3d ovTntMat, std::vector<float> vfJntPtDep);
	~COptParam(void);

	inline float getRotParam(int i) { return m_ovRotVec[i]; }
	inline void setRotParam(int i, double fRotParam) { m_ovRotVec[i] = fRotParam; }
	inline cv::Vec3d getRotVec(void) { return m_ovRotVec; }
	inline void setRotVec(cv::Vec3f ovRotVec) { m_ovRotVec = ovRotVec; }
	inline float getTntParam(int i) { return m_ovTntMat[i]; }
	inline void setTntParam(int i, double fTntParam) { m_ovTntMat[i] = fTntParam; }
	inline cv::Vec3d getTntMat(void) { return m_ovTntMat; }
	inline void setTntMat(cv::Vec3f ovTntMat) { m_ovTntMat = ovTntMat; }
	inline float getJntPtDep(int i) { return m_vfJntPtDep[i]; }
	inline void setJntPtDep(int i, float fJntPtDep) { m_vfJntPtDep[i] = fJntPtDep; }
	inline void addJntPtDep(float fJntPtDep) { m_vfJntPtDep.push_back(fJntPtDep); }
	inline std::vector<float> getJntPtDepLs(void) { return m_vfJntPtDep; }
	inline void setJntPtDepLs(std::vector<float> vfJntPtDep) { m_vfJntPtDep = vfJntPtDep; }
	inline void resetJntPtDepLs(void) { std::vector<float>().swap(m_vfJntPtDep); }
	inline double getCost(void) { return m_fCost; }
	inline void setCost(double fCost) { m_fCost = fCost; }

	struct SParamRng
	{
		cv::Vec3d ovRotVecMax, ovRotVecMin, ovRotVecMean;
		cv::Vec3d ovTntMatMax, ovTntMatMin, ovTntMatMean;
		std::vector<float> vfJntPtDepMax, vfJntPtDepMin, vfJntPtDepMean;
	};

private:
	//! vector of camera rotation parameters
	cv::Vec3d m_ovRotVec;
	//! vector of camera translation parameters
	cv::Vec3d m_ovTntMat;
	//! vector of depth of joint points to the image plane
	std::vector<float> m_vfJntPtDep;
	//! the cost value
	double m_fCost;
};

class CPsEst3d
{
public:
	CPsEst3d(void);
	~CPsEst3d(void);

	//! initializes 3D pose estimation
	void initialize(CCfg oCfg);
	//! processes 3D pose estimation
	void process(CHumPs& oHumPs, cv::Matx34d& oCamExtMatx, int nFrmCnt);
	//! outputs 3D pose estimation
	void output(void);

private:
	//! runs 3D pose estimation through EDA optimization
	void rnPsEst3dEda(CHumPs& oHumPs, cv::Matx34d& oCamExtMatx);
	//! performs EDA optimization for camera parameters or 3D human pose
	double optEdaCamParamHumPs(CHumPs& oHumPs, cv::Matx34d& oCamExtMatx, int iIter, bool bOptHumPsFlg);
	//! initializes the range for EDA optimization
	COptParam::SParamRng initEdaParamRng(CHumPs oHumPs, cv::Matx34d oCamExtMatx, bool bOptHumPsFlg);
	//! calcualtes the new range for EDA optimization
	COptParam::SParamRng calcEdaParamRng(std::vector<COptParam> voOptParam);
	//! generates a random set of optimization parameters
	COptParam genOptParamRand(COptParam::SParamRng sParamRng);
	//! calculates the cost value to be minimized in camera parameters optimization
	double calcCamParamOptCost(COptParam oOptParam, std::vector<cv::Point3f> voJnt3dPt, std::vector<cv::Point2f> voJnt2dPt);
	//! calculates the cost value to be minimized in human pose optimization
	double calcHumPsOptCost(COptParam oOptParam, CHumPs oHumPs, cv::Matx34d oCamExtMatx);
	//! stores the 3D joint points and 2D joint points to compute reprojection error in camera parameters optimization
	void str3d2dJntPt(CHumPs oHumPs, std::vector<cv::Point3f>& voJnt3dPt, std::vector<cv::Point2f>& voJnt2dPt);
	//! calculates the angle error according to angle constraints between joint pairs
	void calcAngErr(cv::Vec3i ovIdx, CHumPs oHumPs, std::vector<cv::Point3f> voJnt3dPt, cv::Vec2d ovCosVecRng, double& fAngErr, int& nValNum);
	//! calculates the angle error according to angle constraints between joint pairs and the body plane
	void calcAngErr(cv::Vec2i ovIdx, CHumPs oHumPs, std::vector<cv::Point3f> voJnt3dPt, cv::Vec3d ovJntPtPlnNormVec, double& fAngErr, int& nValNum);
	//! updates 3D human pose and camera extrinsic parameters
	void updHumPsCamExtMat(CHumPs& oHumPs, cv::Matx34d& oCamExtMatx, COptParam::SParamRng sParamRng, bool bOptHumPsFlg);
	//! outputs text file
	void outTxt(void);

	//! configuration file
	CCfg m_oCfg;
	//! current frame count
	int m_nFrmCnt;
	//! number of joint points corresponding to different standards
	int m_nJntPtNum;
	//! updated camera extrinsic matrix
	cv::Matx34d m_oCamExtMatxUpd;
	//! list of previous human poses
	std::vector<CHumPs> m_voHumPsPrev;
	//! moving average of 3D joint points (additional number count)
	std::vector<cv::Vec4f> m_voJnt3dPtAvg;
};
