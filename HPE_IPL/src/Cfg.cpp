#include "Cfg.h"
#include <cstring>
#include <cstdio>

CCfg::CCfg()
{
	m_oFrmSz = cv::Size(640, 480);
	m_fFrmRt = 10.0f;
	std::strcpy(m_acInVdoPth, ".\\data\\vdo.avi");	// in Windows
	//std::strcpy(m_acInVdoPth, "./data/vdo.avi");	// in Linux
	std::strcpy(m_acFrmFlrPth, ".\\data\\img1\\");	// in Windows
	//std::strcpy(m_acFrmFlrPth, "./data/img1/");	// in Linux
	m_nInCamIdx = 0;
	std::strcpy(m_acOutVdoPth, ".\\data\\outVdo.avi");	// in Windows
	//std::strcpy(m_acOutVdoPth, "./data/outVdo.avi");	// in Linux
	std::strcpy(m_acOutImgFlrPth, ".\\data\\outImg1\\");	// in Windows
	//std::strcpy(m_acOutImgFlrPth, "./data/outImg1/");	// in Linux
	std::strcpy(m_acCamParamPth, ".\\data\\camParam.txt");	// in Windows
	//std::strcpy(m_acCamParamPth, "./data/camParam.txt");	// in Linux
	std::strcpy(m_acPsEst2dJsonFlrPth, ".\\data\\psEst2d\\");	// in Windows
	//std::strcpy(m_acPsEst2dJsonPth, "./data/psEst2d/");	// in Linux
	std::strcpy(m_acPsEst2dTxtPth, ".\\data\\psEst2d.txt");	// in Windows
	//std::strcpy(m_acPsEst2dTxtPth, "./data/psEst2d.txt");	// in Linux
	std::strcpy(m_acOutPsEst3dPth, ".\\data\\psEst3d.txt");	// in Windows
	//std::strcpy(m_acOutPsEst3dPth, "./data/psEst3d.txt");	// in Linux
	m_nInVdoTyp = 0;
	m_nInCalTyp = 0;
	m_nInPsEst2dTyp = 0;
	m_nPsMdlTyp = 2;
	m_bOutVdoFlg = false;
	m_bOutImgFlg = false;
	m_bOutCamParamFlg = false;
	m_bOutPsEst2dFlg = false;
	m_bOutPsEst3dFlg = false;
	m_bPltVisOdomFlg = false;
	m_bPltPsEst2dFlg = 0;
	m_bCamMovFlg = true;
	m_bCamDistFlg = false;
	m_nProcStFrmCnt = 0;
	m_nProcFrmNum = -1;
	m_fOvrdFrmRt = 10.0f;
	m_nRszFrmHei = -1;
	m_bVisOdomRansacFlg = true;
	m_bVisOdomMsk2dPsFlg = true;
	m_fVisOdomSclFac = 1.0f;
	m_nVisOdomMinFeatNum = 2000;
	m_fVisOdomMinFeatDist = 30.0f;
	m_nVisOdomFastThld = 30;
	m_vfInCamFocLen = std::vector<float>(2);
	m_vfInCamPrinPt = std::vector<float>(2);
	m_vfInCamRotVecInit = std::vector<float>(3);
	m_vfInCamTntVecInit = std::vector<float>(3);
	m_vfInCamDistCoeff = std::vector<float>(4);
	std::strcpy(m_acOpPsMdlFlr, ".\\data\\");	// in Windows
	//std::strcpy(m_acOpPsMdlFlr, "./data/");	// in Linux
	std::strcpy(m_acOpNetRes, "-1x736");
	m_nOpGpuNum = -1;
	m_nOpGpuStCnt = 0;
	m_fOpSclGap = 0.25f;
	m_nOpSclNum = 4;
	m_bTrkMs2dJntPtFlg = true;
	m_fPsEst2dScrThld = 0.30f;
	//m_fPsEst2dScrThldHgh = 0.75f;
	m_fPsEst2dDistThld = 30.0f;
	m_fPsEst2dContTmSecThld = 0.20f;
	m_vfJntPr3dLen = std::vector<float>(17);
	m_fJntPr3dLenErrRatThld = 0.10f;
	m_vnPsEst3dInitPop = std::vector<int>(2);
	m_vnPsEst3dSelPop = std::vector<int>(2);
	m_vfPsEst3dStpCritCostRat = std::vector<float>(2);
	m_vfPsEst3dStpCritCostVal = std::vector<float>(2);
	m_vnPsEst3dMaxIterNum = std::vector<int>(2);
	m_vnPsEst3dMaxGenNum = std::vector<int>(3);
	m_fPsEst3dRotRng = 0.333f;
	m_fPsEst3dTntRng = 50.0f;
	m_fPsEst3dJnt3dMovRng = 10.0f;
	std::vector<float>().swap(m_vfPsEst3dJnt3dInitRng);
	m_nPsEst3dTmpConstRefTyp = 1;
	m_fPsEst3dTmpConstSclFac = 0.03f;
	m_fPsEst3dDist2BdyPlnThld = 100.0f;
	m_fPsEst3dTmpConstRegParam = 1.0f;
	m_fPsEst3dDist2BdyPlnRegParam = 1.0f;
	m_fPsEst3dAngCstrRegParam = 1.0f;

	m_oCamIntMat = cv::Mat::eye(3, 3, CV_64F);
	m_oCamRotMatInit = cv::Mat::eye(3, 3, CV_64F);
	m_oCamTntMatInit = cv::Mat::zeros(3, 1, CV_64F);
	m_oCamDistCoeffMat = cv::Mat::zeros(4, 1, CV_64F);
}

CCfg::~CCfg()
{

}

void CCfg::ldCfgFl()
{
	FILE * poCfgFl;
	long nlFlSz, nlRdRst;
	char * pcBuf;

	poCfgFl = std::fopen(".\\data\\cfg.json", "r");	// in Windows
	//poCfgFl = std::fopen("./data/cfg.json", "r");	// in Linux
	if (poCfgFl == NULL) { std::fputs("Error: configuration file not opened", stderr); exit(1); }

	// obtain file size:
	fseek(poCfgFl, 0, SEEK_END);
	nlFlSz = ftell(poCfgFl);
	rewind(poCfgFl);

	// allocate memory to contain the whole file:
	pcBuf = (char*)malloc(sizeof(char)*nlFlSz);
	if (pcBuf == NULL) { fputs("Memory error", stderr); exit(2); }

	// copy the file into the buffer:
	nlRdRst = fread(pcBuf, 1, nlFlSz, poCfgFl);
	//if (nlRdRst != nlFlSz) { fputs("Reading error", stderr); exit(3); }

	std::string strCfg(pcBuf);
	strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), [](char c) { return c >= 0 && isspace(c); }), strCfg.end());	// in Windows
    //strCfg.erase(std::remove_if(strCfg.begin(), strCfg.end(), ::isspace), strCfg.end());	// in Linux

	int nParamPos = strCfg.find("\"inVdoPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acInVdoPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"frmFlrPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acFrmFlrPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inCamIdx\"");
	if (std::string::npos != nParamPos)
		m_nInCamIdx = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outVdoPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acOutVdoPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outImgFlrPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acOutImgFlrPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"camParamPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acCamParamPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"psEst2dJsonFlrPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acPsEst2dJsonFlrPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"psEst2dTxtPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acPsEst2dTxtPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"outPsEst3dPth\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acOutPsEst3dPth, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"inVdoTyp\"");
	if (std::string::npos != nParamPos)
		m_nInVdoTyp = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"inCalTyp\"");
	if (std::string::npos != nParamPos)
		m_nInCalTyp = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"inPsEst2dTyp\"");
	if (std::string::npos != nParamPos)
		m_nInPsEst2dTyp = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psMdlTyp\"");
	if (std::string::npos != nParamPos)
		m_nPsMdlTyp = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outVdoFlg\"");
	if (std::string::npos != nParamPos)
		m_bOutVdoFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outImgFlg\"");
	if (std::string::npos != nParamPos)
		m_bOutImgFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outCamParamFlg\"");
	if (std::string::npos != nParamPos)
		m_bOutCamParamFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outPsEst2dFlg\"");
	if (std::string::npos != nParamPos)
		m_bOutPsEst2dFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"outPsEst3dFlg\"");
	if (std::string::npos != nParamPos)
		m_bOutPsEst3dFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"pltVisOdomFlg\"");
	if (std::string::npos != nParamPos)
		m_bPltVisOdomFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"pltPsEst2dFlg\"");
	if (std::string::npos != nParamPos)
		m_bPltPsEst2dFlg = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"camMovFlg\"");
	if (std::string::npos != nParamPos)
		m_bCamMovFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"camDistFlg\"");
	if (std::string::npos != nParamPos)
		m_bCamDistFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"procStFrmCnt\"");
	if (std::string::npos != nParamPos)
		m_nProcStFrmCnt = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"procFrmNum\"");
	if (std::string::npos != nParamPos)
		m_nProcFrmNum = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"ovrdFrmRt\"");
	if (std::string::npos != nParamPos)
		m_fOvrdFrmRt = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"rszFrmHei\"");
	if (std::string::npos != nParamPos)
		m_nRszFrmHei = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"visOdomRansacFlg\"");
	if (std::string::npos != nParamPos)
		m_bVisOdomRansacFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"visOdomMsk2dPsFlg\"");
	if (std::string::npos != nParamPos)
		m_bVisOdomMsk2dPsFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"visOdomSclFac\"");
	if (std::string::npos != nParamPos)
		m_fVisOdomSclFac = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"visOdomMinFeatNum\"");
	if (std::string::npos != nParamPos)
		m_nVisOdomMinFeatNum = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"visOdomMinFeatDist\"");
	if (std::string::npos != nParamPos)
		m_fVisOdomMinFeatDist = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"visOdomFastThld\"");
	if (std::string::npos != nParamPos)
		m_nVisOdomFastThld = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"inCamFocLen\"");
	if (std::string::npos != nParamPos)
		m_vfInCamFocLen = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"inCamPrinPt\"");
	if (std::string::npos != nParamPos)
		m_vfInCamPrinPt = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"inCamRotVecInit\"");
	if (std::string::npos != nParamPos)
		m_vfInCamRotVecInit = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"inCamTntVecInit\"");
	if (std::string::npos != nParamPos)
		m_vfInCamTntVecInit = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"inCamDistCoeff\"");
	if (std::string::npos != nParamPos)
		m_vfInCamDistCoeff = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"opPsMdlFlr\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acOpPsMdlFlr, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"opNetRes\"");
	if (std::string::npos != nParamPos)
		std::strcpy(m_acOpNetRes, rdCharArr(strCfg, nParamPos).c_str());

	nParamPos = strCfg.find("\"opGpuNum\"");
	if (std::string::npos != nParamPos)
		m_nOpGpuNum = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"opGpuStCnt\"");
	if (std::string::npos != nParamPos)
		m_nOpGpuStCnt = rdInt(strCfg, nParamPos);
	
	nParamPos = strCfg.find("\"opSclGap\"");
	if (std::string::npos != nParamPos)
		m_fOpSclGap = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"opSclNum\"");
	if (std::string::npos != nParamPos)
		m_nOpSclNum = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"trkMs2dJntPtFlg\"");
	if (std::string::npos != nParamPos)
		m_bTrkMs2dJntPtFlg = rdBool(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst2dScrThld\"");
	if (std::string::npos != nParamPos)
		m_fPsEst2dScrThld = rdFlt(strCfg, nParamPos);

	//nParamPos = strCfg.find("\"psEst2dScrThldHgh\"");
	//if (std::string::npos != nParamPos)
	//	m_fPsEst2dScrThldHgh = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst2dDistThld\"");
	if (std::string::npos != nParamPos)
		m_fPsEst2dDistThld = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst2dContTmSecThld\"");
	if (std::string::npos != nParamPos)
		m_fPsEst2dContTmSecThld = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"jntPr3dLen\"");
	if (std::string::npos != nParamPos)
		m_vfJntPr3dLen = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"jntPr3dLenErrRatThld\"");
	if (std::string::npos != nParamPos)
		m_fJntPr3dLenErrRatThld = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dInitPop\"");
	if (std::string::npos != nParamPos)
		m_vnPsEst3dInitPop = rdIntVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dSelPop\"");
	if (std::string::npos != nParamPos)
		m_vnPsEst3dSelPop = rdIntVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dStpCritCostRat\"");
	if (std::string::npos != nParamPos)
		m_vfPsEst3dStpCritCostRat = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dStpCritCostVal\"");
	if (std::string::npos != nParamPos)
		m_vfPsEst3dStpCritCostVal = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dMaxIterNum\"");
	if (std::string::npos != nParamPos)
		m_vnPsEst3dMaxIterNum = rdIntVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dMaxGenNum\"");
	if (std::string::npos != nParamPos)
		m_vnPsEst3dMaxGenNum = rdIntVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dRotRng\"");
	if (std::string::npos != nParamPos)
		m_fPsEst3dRotRng = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dTntRng\"");
	if (std::string::npos != nParamPos)
		m_fPsEst3dTntRng = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dJnt3dMovRng\"");
	if (std::string::npos != nParamPos)
		m_fPsEst3dJnt3dMovRng = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dJnt3dInitRng\"");
	if (std::string::npos != nParamPos)
		m_vfPsEst3dJnt3dInitRng = rdFltVec(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dTmpConstRefTyp\"");
	if (std::string::npos != nParamPos)
		m_nPsEst3dTmpConstRefTyp = rdInt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dTmpConstSclFac\"");
	if (std::string::npos != nParamPos)
		m_fPsEst3dTmpConstSclFac = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dDist2BdyPlnThld\"");
	if (std::string::npos != nParamPos)
		m_fPsEst3dDist2BdyPlnThld = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dTmpConstRegParam\"");
	if (std::string::npos != nParamPos)
		m_fPsEst3dTmpConstRegParam = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dDist2BdyPlnRegParam\"");
	if (std::string::npos != nParamPos)
		m_fPsEst3dDist2BdyPlnRegParam = rdFlt(strCfg, nParamPos);

	nParamPos = strCfg.find("\"psEst3dAngCstrRegParam\"");
	if (std::string::npos != nParamPos)
		m_fPsEst3dAngCstrRegParam = rdFlt(strCfg, nParamPos);

	// terminate
	fclose(poCfgFl);
	free(pcBuf);

	// assert that the configuration parameters are valid
	CV_Assert((0 == m_nInVdoTyp) || (1 == m_nInVdoTyp) || (2 == m_nInVdoTyp) || (3 == m_nInVdoTyp));
	CV_Assert((0 == m_nInCalTyp) || (1 == m_nInCalTyp) || (2 == m_nInCalTyp));
	CV_Assert((0 == m_nInPsEst2dTyp) || (1 == m_nInPsEst2dTyp) || (2 == m_nInPsEst2dTyp));
	if ((2 == m_nInVdoTyp) || (3 == m_nInVdoTyp))
	{
		CV_Assert(2 == m_nInPsEst2dTyp);
		CV_Assert(0 < m_nProcFrmNum);
	}
	if (2 == m_nInPsEst2dTyp)
	{
		CV_Assert(0 >= m_nRszFrmHei);
		CV_Assert(!m_bCamDistFlg);
	}
	CV_Assert((0 == m_nPsMdlTyp) || (1 == m_nPsMdlTyp) || (2 == m_nPsMdlTyp));
	CV_Assert((0 < m_fOpSclGap) || (1 >= m_fOpSclGap));
	CV_Assert(2 == m_vnPsEst3dInitPop.size());
	CV_Assert(2 == m_vnPsEst3dSelPop.size());
	CV_Assert(3 == m_vfPsEst3dStpCritCostRat.size());
	CV_Assert(2 == m_vfPsEst3dStpCritCostVal.size());
	CV_Assert(2 == m_vnPsEst3dMaxIterNum.size());
	CV_Assert(3 == m_vnPsEst3dMaxGenNum.size());
	CV_Assert(2 == m_vfPsEst3dJnt3dInitRng.size());
	CV_Assert((0.0f < m_vfPsEst3dJnt3dInitRng[0]) && (0.0f < m_vfPsEst3dJnt3dInitRng[1]));

	if (0 == m_nPsMdlTyp)
		CV_Assert(17 == m_vfJntPr3dLen.size());
	else if (1 == m_nPsMdlTyp)
		CV_Assert(14 == m_vfJntPr3dLen.size());
	else
		CV_Assert(17 == m_vfJntPr3dLen.size());

	// maximum translation distance of the camera in 3D pose estimation
	m_fPsEst3dTntMaxDist = std::sqrt(m_fPsEst3dTntRng * m_fPsEst3dTntRng * 3.0);

	// maximum movement distance of a 3D joint point in 3D pose estimation
	m_fPsEst3dJnt3dMovMaxDist = std::sqrt(m_fPsEst3dJnt3dMovRng * m_fPsEst3dJnt3dMovRng * 3.0);

	// build the camera matrices and the distortion coefficients
	// the camera intrinsic matrix
	CV_Assert((1 == m_vfInCamFocLen.size()) || (2 == m_vfInCamFocLen.size()));
	CV_Assert(2 == m_vfInCamPrinPt.size());
	if (1 == m_vfInCamFocLen.size())
	{
		m_oCamIntMat.at<double>(0, 0) = m_vfInCamFocLen[0];
		m_oCamIntMat.at<double>(1, 1) = m_vfInCamFocLen[0];
	}
	else if (2 == m_vfInCamFocLen.size())
	{
		m_oCamIntMat.at<double>(0, 0) = m_vfInCamFocLen[0];
		m_oCamIntMat.at<double>(1, 1) = m_vfInCamFocLen[1];
	}
	m_oCamIntMat.at<double>(0, 2) = m_vfInCamPrinPt[0];
	m_oCamIntMat.at<double>(1, 2) = m_vfInCamPrinPt[1];
	m_oCamIntMat.at<double>(2, 2) = 1.0;

	// the initial camera rotation matrix
	CV_Assert(3 == m_vfInCamRotVecInit.size());
	cv::Mat oCamRotVecInit = cv::Mat::zeros(3, 1, CV_64F);
	oCamRotVecInit.at<double>(0) = m_vfInCamRotVecInit[0];
	oCamRotVecInit.at<double>(1) = m_vfInCamRotVecInit[1];
	oCamRotVecInit.at<double>(2) = m_vfInCamRotVecInit[2];
	cv::Rodrigues(oCamRotVecInit, m_oCamRotMatInit);

	// the initial camera translaiton matrix
	CV_Assert(3 == m_vfInCamTntVecInit.size());
	m_oCamTntMatInit.at<double>(0) = m_vfInCamTntVecInit[0];
	m_oCamTntMatInit.at<double>(1) = m_vfInCamTntVecInit[1];
	m_oCamTntMatInit.at<double>(2) = m_vfInCamTntVecInit[2];

	if (m_bCamDistFlg)
	{
		// the distortion coefficients
		CV_Assert(4 == m_vfInCamDistCoeff.size());
		m_oCamDistCoeffMat.at<double>(0) = m_vfInCamDistCoeff[0];
		m_oCamDistCoeffMat.at<double>(1) = m_vfInCamDistCoeff[1];
		m_oCamDistCoeffMat.at<double>(2) = m_vfInCamDistCoeff[2];
		m_oCamDistCoeffMat.at<double>(3) = m_vfInCamDistCoeff[3];
	}
}

std::string CCfg::rdCharArr(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nValLen = strCfg.find("\"", (nValPos + 1)) - nValPos;

	return strCfg.substr(nValPos, nValLen);
}

int CCfg::rdInt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	return std::atoi(strCfg.substr(nValPos, nValLen).c_str());
}

float CCfg::rdFlt(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	return std::atof(strCfg.substr(nValPos, nValLen).c_str());
}

bool CCfg::rdBool(std::string strCfg, int nParamPos)
{
	int nBoolVal, nValPos, nValLen, nValEnd1, nValEnd2;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 1;
	nValEnd1 = strCfg.find(",", (nValPos + 1));
	nValEnd2 = strCfg.find("}", (nValPos + 1));
	nValLen = (nValEnd1 <= nValEnd2) ? (nValEnd1 - nValPos) : (nValEnd2 - nValPos);

	nBoolVal = std::atoi(strCfg.substr(nValPos, nValLen).c_str());
	if (nBoolVal > 0)
		return true;
	else if (nBoolVal <= 0)
		return false;
}

std::vector<int> CCfg::rdIntVec(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd;
	std::vector<int> vnVal;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nValEnd = strCfg.find("]", (nValPos + 1));

	while (nValPos < nValEnd)
	{
		nValLen = strCfg.find(",", (nValPos + 1)) - nValPos;
		if (0 > nValLen)
			nValLen = nValEnd - nValPos;
		vnVal.push_back(std::atoi(strCfg.substr(nValPos, nValLen).c_str()));
		nValPos = nValPos + nValLen + 1;
	}

	return vnVal;
}

std::vector<float> CCfg::rdFltVec(std::string strCfg, int nParamPos)
{
	int nValPos, nValLen, nValEnd;
	std::vector<float> vfVal;

	nValPos = strCfg.find(":", (nParamPos + 1)) + 2;
	nValEnd = strCfg.find("]", (nValPos + 1));

	while (nValPos < nValEnd)
	{
		nValLen = strCfg.find(",", (nValPos + 1)) - nValPos;
		if (0 > nValLen)
			nValLen = nValEnd - nValPos;
		vfVal.push_back(std::atof(strCfg.substr(nValPos, nValLen).c_str()));
		nValPos = nValPos + nValLen + 1;
	}

	return vfVal;
}
