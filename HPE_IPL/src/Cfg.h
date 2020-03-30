#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class CCfg
{
public:
	//! full constructor
	CCfg();
	//! default destructor
	~CCfg();

	//! loads configuration file from directory
	void ldCfgFl(void);

	inline cv::Size getFrmSz(void) { return m_oFrmSz; }
	void setFrmSz(cv::Size oFrmSz) { m_oFrmSz = oFrmSz; };
	inline float getFrmRt(void) { return m_fFrmRt; }
	inline void setFrmRt(float fFrmRt) { m_fFrmRt = fFrmRt; }

	inline char* getInVdoPth(void) { return m_acInVdoPth; }
	inline char* getFrmFlrPth(void) { return m_acFrmFlrPth; }
	inline int getInCamIdx(void) { return m_nInCamIdx; }
	inline char* getOutVdoPth(void) { return m_acOutVdoPth; }
	inline char* getOutImgFlrPth(void) { return m_acOutImgFlrPth; }
	inline char* getCamParamPth(void) { return m_acCamParamPth; }
	inline char* getPsEst2dJsonFlrPth(void) { return m_acPsEst2dJsonFlrPth; }
	inline char* getPsEst2dTxtPth(void) { return m_acPsEst2dTxtPth; }
	inline char* getOutPsEst3dPth(void) { return m_acOutPsEst3dPth; }
	inline int getInVdoTyp(void) { return m_nInVdoTyp; }
	inline int getInCalTyp(void) { return m_nInCalTyp; }
	inline int getInPsEst2dTyp(void) { return m_nInPsEst2dTyp; }
	inline int getPsMdlTyp(void) { return m_nPsMdlTyp; }
	inline bool getOutVdoFlg(void) { return m_bOutVdoFlg; }
	inline bool getOutImgFlg(void) { return m_bOutImgFlg; }
	inline bool getOutCamParamFlg(void) { return m_bOutCamParamFlg; }
	inline bool getOutPsEst2dFlg(void) { return m_bOutPsEst2dFlg; }
	inline bool getOutPsEst3dFlg(void) { return m_bOutPsEst3dFlg; }
	inline bool getPltVisOdomFlg(void) { return m_bPltVisOdomFlg; }
	inline bool getPltPsEst2dFlg(void) { return m_bPltPsEst2dFlg; }
	inline bool getCamMovFlg(void) { return m_bCamMovFlg; }
	inline bool getCamDistFlg(void) { return m_bCamDistFlg; }
	inline int getProcStFrmCnt(void) { return m_nProcStFrmCnt; }
	inline void incProcStFrmCnt(void) { m_nProcStFrmCnt++; }
	inline int getProcFrmNum(void) { return m_nProcFrmNum; }
	inline float getOvrdFrmRt(void) { return m_fOvrdFrmRt; }
	inline int getRszFrmHei(void) { return m_nRszFrmHei; }
	inline bool getVisOdomRansacFlg(void) { return m_bVisOdomRansacFlg; }
	inline bool getVisOdomMsk2dPsFlg(void) { return m_bVisOdomMsk2dPsFlg; }
	inline float getVisOdomSclFac(void) { return m_fVisOdomSclFac; }
	inline int getVisOdomMinFeatNum(void) { return m_nVisOdomMinFeatNum; }
	inline float getVisOdomMinFeatDist(void) { return m_fVisOdomMinFeatDist; }
	inline int getVisOdomFastThld(void) { return m_nVisOdomFastThld; }
	inline std::vector<float> getInCamFocLen(void) { return m_vfInCamFocLen; }
	inline std::vector<float> getInCamPrinPt(void) { return m_vfInCamPrinPt; }
	inline std::vector<float> getInCamRotVecInit(void) { return m_vfInCamRotVecInit; }
	inline std::vector<float> getInCamTntVecInit(void) { return m_vfInCamTntVecInit; }
	inline std::vector<float> getInCamDistCoeff(void) { return m_vfInCamDistCoeff; }
	inline char* getOpPsMdlFlr(void) { return m_acOpPsMdlFlr; }
	inline char* getOpNetRes(void) { return m_acOpNetRes; }
	inline int getOpGpuNum(void) { return m_nOpGpuNum; }
	inline int getOpGpuStCnt(void) { return m_nOpGpuStCnt; }
	inline float getOpSclGap(void) { return m_fOpSclGap; }
	inline int getOpSclNum(void) { return m_nOpSclNum; }
	inline bool getTrkMs2dJntPtFlg(void) { return m_bTrkMs2dJntPtFlg; }
	inline float getPsEst2dScrThld(void) { return m_fPsEst2dScrThld; }
	//inline float getPsEst2dScrThldHgh(void) { return m_fPsEst2dScrThldHgh; }
	inline float getPsEst2dDistThld(void) { return m_fPsEst2dDistThld; }
	inline float getPsEst2dContTmSecThld(void) { return m_fPsEst2dContTmSecThld; }
	inline int getPsEst2dContFrmCntThld(void) { return m_fPsEst2dContTmSecThld * m_fFrmRt; }
	inline std::vector<float> getJntPr3dLen(void) { return m_vfJntPr3dLen; }
	inline float getJntPr3dLenErrRatThld(void) { return m_fJntPr3dLenErrRatThld; }
	inline std::vector<int> getPsEst3dInitPop(void) { return m_vnPsEst3dInitPop; }
	inline std::vector<int> getPsEst3dSelPop(void) { return m_vnPsEst3dSelPop; }
	inline std::vector<float> getPsEst3dStpCritCostRat(void) { return m_vfPsEst3dStpCritCostRat; }
	inline std::vector<float> getPsEst3dStpCritCostVal(void) { return m_vfPsEst3dStpCritCostVal; }
	inline std::vector<int> getPsEst3dMaxIterNum(void) { return m_vnPsEst3dMaxIterNum; }
	inline std::vector<int> getPsEst3dMaxGenNum(void) { return m_vnPsEst3dMaxGenNum; }
	inline float getPsEst3dRotRng(void) { return m_fPsEst3dRotRng; }
	inline float getPsEst3dTntRng(void) { return m_fPsEst3dTntRng; }
	inline float getPsEst3dJnt3dMovRng(void) { return m_fPsEst3dJnt3dMovRng; }
	inline std::vector<float> getPsEst3dJnt3dInitRng(void) { return m_vfPsEst3dJnt3dInitRng; }
	inline int getPsEst3dTmpConstRefTyp(void) { return m_nPsEst3dTmpConstRefTyp; }
	inline float getPsEst3dTmpConstSclFac(void) { return m_fPsEst3dTmpConstSclFac; }
	inline float getPsEst3dDist2BdyPlnThld(void) { return m_fPsEst3dDist2BdyPlnThld; }
	inline float getPsEst3dTmpConstRegParam(void) { return m_fPsEst3dTmpConstRegParam; }
	inline float getPsEst3dDist2BdyPlnRegParam(void) { return m_fPsEst3dDist2BdyPlnRegParam; }
	inline float getPsEst3dAngCstrRegParam(void) { return m_fPsEst3dAngCstrRegParam; }

	inline double getPsEst3dTntMaxDist(void) { return m_fPsEst3dTntMaxDist; }
	inline double getPsEst3dJnt3dMovMaxDist(void) { return m_fPsEst3dJnt3dMovMaxDist; }
	inline cv::Mat getCamIntMat(void) { return m_oCamIntMat; }
	inline cv::Mat getCamRotMatInit(void) { return m_oCamRotMatInit; }
	inline cv::Mat getCamTntMatInit(void) { return m_oCamTntMatInit; }
	inline cv::Mat getDistCoeffMat(void) { return m_oCamDistCoeffMat; }

	//! reads char array
	std::string rdCharArr(std::string strCfg, int nParamPos);
	//! reads integer number
	int rdInt(std::string strCfg, int nParamPos);
	//! reads float number
	float rdFlt(std::string strCfg, int nParamPos);
	//! reads bool value
	bool rdBool(std::string strCfg, int nParamPos);
	//! reads vector of integer number
	std::vector<int> rdIntVec(std::string strCfg, int nParamPos);
	//! reads vector of float number
	std::vector<float> rdFltVec(std::string strCfg, int nParamPos);

private:

	//! video frame size
	cv::Size m_oFrmSz;
	//! video frame rate
	float m_fFrmRt;

	//! path of input video stream, necessary when m_nInVdoTyp == 0, 2
	char m_acInVdoPth[256];
	//! path of folder for frame image files, necessary when m_nInVdoTyp == 1 or m_nInVdoTyp == 2, 3 and m_nInPsEst2dTyp == 2
	char m_acFrmFlrPth[256];
	//! index of input local camera, necessary when m_nInVdoTyp = 3
	int m_nInCamIdx;
	//! path of output video file, necessary when m_bOutVdoFlg == true
	char m_acOutVdoPth[256];
	//! path of folder for output image files, necessary when m_bOutImgFlg == true
	char m_acOutImgFlrPth[256];
	//! path of camera parameters, necessary when m_nInCalTyp == 0 or m_bOutCamParamFlg == true
	char m_acCamParamPth[256];
	//! path of folder for json files of 2D pose estimation, necessary when m_bInPsEst2dTyp = 1 or 2
	char m_acPsEst2dJsonFlrPth[256];
	//! path of txt file of 2D pose estimation, necessary when m_bInPsEst2dTyp = 0 or m_bOutPsEst2dFlg == true
	char m_acPsEst2dTxtPth[256];
	//! path of output 3D pose estimation, necessary when m_bOutPsEst3dFlg == true
	char m_acOutPsEst3dPth[256];
	//! type of input video source: 0: video file; 1: image files; 2: IP camera; 3: local camera
	int m_nInVdoTyp;
	//! type of input calibration: 0: txt file; 1: visual odometry; 2: optimization while 3D pose estimation (set to 2 when m_bCamMovFlg == false)
	int m_nInCalTyp;
	//! type of input 2D pose estimation: 0: txt file; 1: json files; 2: preprocessed by OpenPose
	int m_nInPsEst2dTyp;
	//! the type of model of human pose: 0: COCO; 1: MPI
	int m_nPsMdlTyp;
	//! flag of output video file
	bool m_bOutVdoFlg;
	//! flag of output image files
	bool m_bOutImgFlg;
	//! flag of output camera parameters
	bool m_bOutCamParamFlg;
	//! flag of output 2D pose estimation
	bool m_bOutPsEst2dFlg;
	//! flag of output 3D pose estimation
	bool m_bOutPsEst3dFlg;
	//! flag of plotting visual odometry, necessary when m_nInCalTyp == 1
	bool m_bPltVisOdomFlg;
	//! flag of plotting 2D pose estimation
	bool m_bPltPsEst2dFlg;
	//! flag of camera movement
	bool m_bCamMovFlg;
	//! flag of camera distortion
	bool m_bCamDistFlg;
	//! starting frame count to process
	int m_nProcStFrmCnt;
	//! number of frames to process (-1: till the end of the video source)
	int m_nProcFrmNum;
	//! overriden frame rate, necessary when m_nInVdoTyp > 0
	float m_fOvrdFrmRt;
	//! resized video frame height (-1: original size)
	int m_nRszFrmHei;
	//! flag of applying RANSAC for estimating extrinsic camera matrices, necessary when m_nInCalTyp == 1
	bool m_bVisOdomRansacFlg;
	//! flag of masking by 2D pose estimation, necessary when m_nInCalTyp == 1
	bool m_bVisOdomMsk2dPsFlg;
	//! scaling factor for visual odometry, necessary when m_nInCalTyp == 1
	float m_fVisOdomSclFac;
	//! minimum number of feature points in visual odometry, necessary when m_nInCalTyp == 1
	int m_nVisOdomMinFeatNum;
	//! threshold of minimum distance between feature points to remove duplicates in visual odometry, necessary when m_nInCalTyp == 1
	float m_fVisOdomMinFeatDist;
	//! threshold for the detection of FAST feature points, necessary when m_nInCalTyp == 1
	int m_nVisOdomFastThld;
	//! focal length(s) of the camera intrinsic matrix
	std::vector<float> m_vfInCamFocLen;
	//! principal point of the camera intrinsic matrix
	std::vector<float> m_vfInCamPrinPt;
	//! initial rotation vector of the camera
	std::vector<float> m_vfInCamRotVecInit;
	//! initial translation vector of the camera
	std::vector<float> m_vfInCamTntVecInit;
	//! distortion coefficients of the camera, necessary when m_bCamDistFlg == true
	std::vector<float> m_vfInCamDistCoeff;
	//! path of folder where the pose models for OpenPose are located, necessary when m_nInPsEst2dTyp == 2
	char m_acOpPsMdlFlr[256];
	//! net resolution for OpenPose, necessary when m_nInPsEst2dTyp == 2
	char m_acOpNetRes[256];
	//! number of GPUs for OpenPose, necessary when m_nInPsEst2dTyp == 2
	int m_nOpGpuNum;
	//! GPU device start number for OpenPose, necessary when m_nInPsEst2dTyp == 2
	int m_nOpGpuStCnt;
	//! scale gap between scales for OpenPose, necessary when m_nInPsEst2dTyp == 2
	float m_fOpSclGap;
	//! number of scales to average for OpenPose, necessary when m_nInPsEst2dTyp == 2
	int m_nOpSclNum;
	//! flag of tracking missing 2D joint points
	bool m_bTrkMs2dJntPtFlg;
	//! the threshold for the human pose estimation score to remove false positives
	float m_fPsEst2dScrThld;
	////! the high threshold for the human pose estimation score to recover true positives
	//float m_fPsEst2dScrThldHgh;
	//! the threshold for the distance between a joint point and its estimation, necessary when m_bTrkMs2dJntPtFlg = true
	float m_fPsEst2dDistThld;
	//! the time threshold in seconds for continuous human pose estimation to recover true positives, necessary when m_bTrkMs2dJntPtFlg = true
	float m_fPsEst2dContTmSecThld;
	//! 3D lengths between pre-defined pairs of joint points in millimeters
	std::vector<float> m_vfJntPr3dLen;
	//! the threshold for the ratio of allowable error in 3D length estimation
	float m_fJntPr3dLenErrRatThld;
	//! the sizes of initial population for camera parameters and human pose in EDA
	std::vector<int> m_vnPsEst3dInitPop;
	//! the sizes of selected population for camera parameters and human pose in EDA
	std::vector<int> m_vnPsEst3dSelPop;
	//! the stopping criterion for the decreasing ratio of cost for generation of camera parameters and both generation and iteration of human pose in EDA
	std::vector<float> m_vfPsEst3dStpCritCostRat;
	//! the stopping criterion for the value of cost in EDA optimization of camera parameters and human pose (> 1.0)
	std::vector<float> m_vfPsEst3dStpCritCostVal;
	//! the maximum numbers of iterations in EDA optimization of initial case and regular case
	std::vector<int> m_vnPsEst3dMaxIterNum;
	//! the maximum numbers of generations in EDA optimization of initial case of human pose and regular cases of both camera parameters and human pose
	std::vector<int> m_vnPsEst3dMaxGenNum;
	//! the range of camera rotation (between 0.0 and 1.0) in EDA optimization, necessary when m_bCalMovFlg == true
	float m_fPsEst3dRotRng;
	//! the range of camera translation in millimeters in EDA optimization, necessary when m_bCalMovFlg == true
	float m_fPsEst3dTntRng;
	//! the range of 3D joint point movement (depth) in millimeters in EDA optimization
	float m_fPsEst3dJnt3dMovRng;
	//! the range of 3D joint point initialization (depth) in millimeters in EDA optimization
	std::vector<float> m_vfPsEst3dJnt3dInitRng;
	//! type of reference for temporal constancy in EDA optimization: n<0: the running average; n=0: the initial frame; n>0: the previous n frames
	int m_nPsEst3dTmpConstRefTyp;
	//! the scaling factor for temporal constancy of joint points in EDA optimization
	float m_fPsEst3dTmpConstSclFac;
	//! the threshold for the standard deviation of distance to the fitted plane of 3D joint points in millimeters in EDA optimization
	float m_fPsEst3dDist2BdyPlnThld;
	//! the regularization parameter for the term of temporal constancy in EDA optimization
	float m_fPsEst3dTmpConstRegParam;
	//! the regularization parameter for the term of distance to body plane in EDA optimization
	float m_fPsEst3dDist2BdyPlnRegParam;
	//! the regularization parameter for the term of angle constraints in EDA optimization
	float m_fPsEst3dAngCstrRegParam;

	//! maximum translation distance of the camera in 3D pose estimation
	double m_fPsEst3dTntMaxDist;
	//! maximum movement distance of a 3D joint point in 3D pose estimation
	double m_fPsEst3dJnt3dMovMaxDist;
	//! camera intrinsic matrix
	cv::Mat m_oCamIntMat;
	//! initial camera rotation matrix
	cv::Mat m_oCamRotMatInit;
	//! initial camera translation matrix
	cv::Mat m_oCamTntMatInit;
	//! distortion coefficients matrix
	cv::Mat m_oCamDistCoeffMat;
};
