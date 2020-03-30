#include "PsEst2d.h"

CJntPt::CJntPt(void)
{
	set2dPt(cv::Point2f(0.0f, 0.0f));
	set3dPt(cv::Point3f(0.0f, 0.0f, 0.0f));
	setPsEst2dScr(0.0f);
}

CJntPt::CJntPt(cv::Point2f o2dPt, cv::Point3f o3dPt, float fPsEst2dScr)
{
	set2dPt(o2dPt);
	set3dPt(o3dPt);
	setPsEst2dScr(fPsEst2dScr);
}

CJntPt::~CJntPt(void)
{

}

CHumPs::CHumPs(void)
{
	setFrmCnt(-1);
	resetJntPtLs();
	resetMtchFrmNumLs();
}

CHumPs::CHumPs(int nFrmCnt, std::vector<CJntPt> voJntPt, std::vector<int> vnMtchFrmNum)
{
	setFrmCnt(nFrmCnt);
	setJntPtLs(voJntPt);
	setMtchFrmNumLs(vnMtchFrmNum);
}

CHumPs::CHumPs(int nFrmCnt, std::vector<CJntPt> voJntPt)
{
	setFrmCnt(nFrmCnt);
	setJntPtLs(voJntPt);
	std::vector<int> vnMtchFrmNum;
	for (int i = 0; i < voJntPt.size(); i++)
		vnMtchFrmNum.push_back(0);
	setMtchFrmNumLs(vnMtchFrmNum);
}

CHumPs::~CHumPs(void)
{
	resetJntPtLs();
	resetMtchFrmNumLs();
}

CPsEst2d::CPsEst2d(void)
{

}

CPsEst2d::~CPsEst2d(void)
{
	// input camera parameters
	m_ifsInPs2d.close();
}

void CPsEst2d::initialize(CCfg oCfg)
{
	// configuration file
	m_oCfg = oCfg;

	// current frame count
	m_nFrmCnt = -1;

	if (0 == m_oCfg.getInPsEst2dTyp())
	{
		// input 2D human poses
		std::string strInPsEst2dPth = m_oCfg.getPsEst2dTxtPth();
		m_ifsInPs2d.open(strInPsEst2dPth.c_str());
	}

	// the previous frame in grey scale
	m_oImgFrmPrevGry = cv::Mat::zeros(m_oCfg.getFrmSz(), CV_8UC1);

	// number of joint points corresponding to different standards
	// COCO standard
	if (0 == m_oCfg.getPsMdlTyp() || 2 == m_oCfg.getPsMdlTyp())
		m_nJntPtNum = PS_COCO_JNTPT_NUM;
	// MPI standard
	else
		m_nJntPtNum = PS_MPI_JNTPT_NUM;

	// the human pose
	std::vector<CJntPt> voJntPt(m_nJntPtNum);
	std::vector<int> vnMtchFrmNum(m_nJntPtNum);
	for (int i = 0; i < m_nJntPtNum; i++)
		vnMtchFrmNum[i] = 0;
	m_oHumPs = CHumPs(m_nFrmCnt, voJntPt, vnMtchFrmNum);

    // output text file
    if (m_oCfg.getOutPsEst2dFlg())
    {
        FILE* pfOutPsEst2dTxt = std::fopen(m_oCfg.getPsEst2dTxtPth(), "w");
        std::fclose(pfOutPsEst2dTxt);
    }
}

CHumPs CPsEst2d::process(cv::Mat oImgFrmCurr, cv::Mat& oImgMsk, int nFrmCnt)
{
    m_nFrmCnt = nFrmCnt;
	m_oHumPsPrev = m_oHumPs;

	// read 2D human pose from txt file
	if (0 == m_oCfg.getInPsEst2dTyp())
		m_oHumPs = rdPsEst2dTxt(nFrmCnt);
	// read 2D human pose from json files
	else if ((1 == m_oCfg.getInPsEst2dTyp()) || (2 == m_oCfg.getInPsEst2dTyp()))
		m_oHumPs = rdPsEst2dJson(nFrmCnt);
	
	cv::Mat oImgFrmCurrBlr, oImgFrmCurrGry;
	cv::GaussianBlur(oImgFrmCurr, oImgFrmCurrBlr, GAUSS_BLR_SZ, 0, 0);
	cv::cvtColor(oImgFrmCurrBlr, oImgFrmCurrGry, cv::COLOR_BGR2GRAY);

	// track missing joint points
	//if (m_oCfg.getTrkMs2dJntPtFlg() && (m_oCfg.getProcStFrmCnt() < nFrmCnt))
	if (m_oCfg.getTrkMs2dJntPtFlg() && (m_oCfg.getProcStFrmCnt() < nFrmCnt) )
        trkMsJtPt(oImgFrmCurrGry);

	m_oImgFrmPrevGry = oImgFrmCurrGry;

	// plot the mask of 2D human pose
	if ((1 == m_oCfg.getInCalTyp()) && m_oCfg.getVisOdomMsk2dPsFlg())
		plt2dPsMsk(oImgMsk);
	else
		oImgMsk = cv::Mat(m_oCfg.getFrmSz(), CV_8UC1, cv::Scalar(255));

	return m_oHumPs;
}

void CPsEst2d::output(cv::Mat& oImgFrm, cv::Mat oImgMsk)
{
	// output text file
	if (m_oCfg.getOutPsEst2dFlg())
		outTxt();

	// plot 2D human pose
	if (m_oCfg.getPltPsEst2dFlg())
		pltPsEst2d(oImgFrm, oImgMsk);
}

// only in Release mode
float CPsEst2d::preProcOp(void)
{
	// Step 1 - Set logging level
	op::checkBool((0 <= OP_LOG_LVL) && (4 >= OP_LOG_LVL), "Wrong logging_level value.",
		__LINE__, __FUNCTION__, __FILE__);
	op::ConfigureLog::setPriorityThreshold((op::Priority)OP_LOG_LVL);
	op::opLog("Starting pose estimation by OpenPose", op::Priority::High);

    // Step 2 - Applying user defined configuration - Google flags to program variables
	// output size
	char acFrmSz[128] = {};
	cv::Size oFrmSz = m_oCfg.getFrmSz();
	std::sprintf(acFrmSz, "%dx%d", oFrmSz.width, oFrmSz.height);
	const auto voOutSz = op::flagsToPoint(acFrmSz, "-1x-1");
	// net input size
	const auto vonNetInSz = op::flagsToPoint(m_oCfg.getOpNetRes(), "-1x368");
	// producer type
	char acInFrmFlrPth[128] = {};
	char acInVdoPth[128] = {};
	char acInIpCamPth[128] = {};
	//// from video file
	//if (0 == m_oCfg.getInVdoTyp())
	std::sprintf(acInVdoPth, m_oCfg.getInVdoPth());
	//// from image files
	//else if (1 == m_oCfg.getInVdoTyp())
	//	std::sprintf(acInFrmFlrPth, m_oCfg.getFrmFlrPth());
	//// from IP camera
	//else if (2 == m_oCfg.getInVdoTyp())
	//	std::sprintf(acInIpCamPth, m_oCfg.getInVdoPth());
	op::ProducerType oProdShrPtr;
	op::String producerString;
	std::tie(oProdShrPtr, producerString) = op::flagsToProducer(acInFrmFlrPth, acInVdoPth, acInIpCamPth, /*m_oCfg.getInCamIdx()*/-1, false, -1);
	producerString = op::String(acInVdoPth);
	// pose model
	char acPsMdl[128] = {};
	if (0 == m_oCfg.getPsMdlTyp())
		std::sprintf(acPsMdl, "COCO");
	else if (1 == m_oCfg.getPsMdlTyp())
		std::sprintf(acPsMdl, "MPI");
	else
		std::sprintf(acPsMdl, "BODY_25");
	const auto oPsMdl = op::flagsToPoseModel(acPsMdl);

	// poseMode
	const auto poseMode = op::flagsToPoseMode(1);
	// JSON saving
	_mkdir(m_oCfg.getPsEst2dJsonFlrPth());	// in Windows
	//mkdir(m_oCfg.getPsEst2dJsonFlrPth(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux
	// joint point scale
	const auto oJntPtScl = op::flagsToScaleMode(2);
	// heatmaps to add
	const auto voHtmpTyp = op::flagsToHeatMaps(false, false, false);
	const auto oHtmpScl = op::flagsToHeatMapScaleMode(2);
	// enabling Google logging
	const bool bGoogleLogFlg = true;
	// logging
	op::opLog("", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);

	// Step 3 - Set OpenPose wrapper
	op::opLog("Configuring OpenPose wrapper.", op::Priority::Low, __LINE__, __FUNCTION__, __FILE__);
	op::Wrapper opWrapper;
	// Pose configuration (use WrapperStructPose{} for default and recommended configuration)
	//const op::WrapperStructPose oWrprStructPs{ poseMode };
	const op::WrapperStructPose oWrprStructPs(poseMode, vonNetInSz, voOutSz, oJntPtScl,
		m_oCfg.getOpGpuNum(), m_oCfg.getOpGpuStCnt(), m_oCfg.getOpSclNum(),
		m_oCfg.getOpSclGap(), op::RenderMode::None,
		oPsMdl, true, 0.6f, 0.7f, 0, m_oCfg.getOpPsMdlFlr(),
		voHtmpTyp, op::ScaleMode::UnsignedChar, false, 0.05f);
	// Producer (use default to disable any input)
	int nProcNdFrmCnt = -1;
	if (0 < m_oCfg.getProcFrmNum())
		nProcNdFrmCnt = m_oCfg.getProcStFrmCnt() + m_oCfg.getProcFrmNum() - 1;
	const op::WrapperStructInput oWrprStructIn{ oProdShrPtr, op::String(acInVdoPth), static_cast<unsigned long long>(m_oCfg.getProcStFrmCnt()),
		static_cast<unsigned long long>(1), 
		static_cast<unsigned long long>(nProcNdFrmCnt),
		false, false, 0, false };
	// Consumer (comment or use default argument to disable any output)
	char acOutFrmFlrPth[128] = {};
	double bDispFlg = -1;
	//if ((2 == m_oCfg.getInVdoTyp()) || (3 == m_oCfg.getInVdoTyp()))
	//{
	//	_mkdir(m_oCfg.getFrmFlrPth());	// in Windows
	//	//mkdir(m_oCfg.getFrmFlrPth(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux
	//	std::sprintf(acOutFrmFlrPth, m_oCfg.getFrmFlrPth());
	//	bDispFlg = true;
	//}
	const op::WrapperStructOutput oWrprStructOut{ 0.1, "", op::DataFormat::Json, m_oCfg.getPsEst2dJsonFlrPth(), "",
		1, 1, "", "jpg"};
	// Configure wrapper
	opWrapper.configure(oWrprStructPs); 
	//opWrapper.configure(op::WrapperStructPose{});
	opWrapper.configure(op::WrapperStructFace{});
	opWrapper.configure(op::WrapperStructHand{});
	opWrapper.configure(op::WrapperStructExtra{});
	//opWrapper.configure(op::WrapperStructInput{});
	opWrapper.configure(oWrprStructIn);
	//opWrapper.configure(op::WrapperStructOutput{});
	opWrapper.configure(oWrprStructOut);
	opWrapper.configure(op::WrapperStructGui());

	// Step 4 - Start processing
	op::opLog("Starting thread(s)", op::Priority::High);
	// Using the main thread (this thread) for processing (it saves 1 thread)
	// Start, run & stop threads
	const auto oTmSt = std::chrono::high_resolution_clock::now();
	opWrapper.exec();  // It blocks this thread until all threads have finished
	const auto oTmNd = std::chrono::high_resolution_clock::now();

	// Step 5 - Measure total time
	const auto fTtlTmSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(oTmNd - oTmSt).count() * 1e-9;
	const auto message = "Preprocessed 2D pose estimation successfully finished. Total time: " + std::to_string(fTtlTmSec) + " seconds.";
	op::opLog(message, op::Priority::High);

	// Step 6 - Compute average frame rate
	float fFrmRt = m_oCfg.getProcFrmNum() / fTtlTmSec;
	m_oCfg.setFrmRt(fFrmRt);
	return fFrmRt;
}

CHumPs CPsEst2d::rdPsEst2dTxt(int nFrmCnt)
{
	bool bNxtFrmFlg = true;
	int nHumPsFrmCnt = -1;
	char acBuf[1024] = {};
	std::vector<CHumPs> voHumPs;
	std::vector<cv::Point2f> vo2dPt(m_nJntPtNum);
	std::vector<float> vfPsEst2dScr(m_nJntPtNum);
	std::vector<CJntPt> voJntPt(m_nJntPtNum);
	for (int i = 0; i < m_nJntPtNum; i++)
	{
		vo2dPt[i] = cv::Point2f(0.0f, 0.0f);
		vfPsEst2dScr[i] = 0.0f;
	}

	while ((!m_ifsInPs2d.eof()) && ((nFrmCnt == m_oHumPsNxt.getFrmCnt()) || (-1 == m_oHumPsNxt.getFrmCnt())))
	{
		// at the first frame with human poses
		if (-1 == m_oHumPsNxt.getFrmCnt())
			bNxtFrmFlg = false;

		// push back the extra human pose read from the last iteration
		// not at the first frame with human poses
		if (bNxtFrmFlg && (nFrmCnt == m_oHumPsNxt.getFrmCnt()))
		{
			voHumPs.push_back(m_oHumPsNxt);
			bNxtFrmFlg = false;
		}

		// read from the input txt file
		m_ifsInPs2d.getline(acBuf, 1024);
		// COCO standard
		if (0 == m_oCfg.getPsMdlTyp())
			std::sscanf(acBuf, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
				&nHumPsFrmCnt, &vo2dPt[0].x, &vo2dPt[0].y, &vfPsEst2dScr[0], &vo2dPt[1].x, &vo2dPt[1].y, &vfPsEst2dScr[1], &vo2dPt[2].x, &vo2dPt[2].y, &vfPsEst2dScr[2],
				&vo2dPt[3].x, &vo2dPt[3].y, &vfPsEst2dScr[3], &vo2dPt[4].x, &vo2dPt[4].y, &vfPsEst2dScr[4], &vo2dPt[5].x, &vo2dPt[5].y, &vfPsEst2dScr[5],
				&vo2dPt[6].x, &vo2dPt[6].y, &vfPsEst2dScr[6], &vo2dPt[7].x, &vo2dPt[7].y, &vfPsEst2dScr[7], &vo2dPt[8].x, &vo2dPt[8].y, &vfPsEst2dScr[8],
				&vo2dPt[9].x, &vo2dPt[9].y, &vfPsEst2dScr[9], &vo2dPt[10].x, &vo2dPt[10].y, &vfPsEst2dScr[10], &vo2dPt[11].x, &vo2dPt[11].y, &vfPsEst2dScr[11],
				&vo2dPt[12].x, &vo2dPt[12].y, &vfPsEst2dScr[12], &vo2dPt[13].x, &vo2dPt[13].y, &vfPsEst2dScr[13], &vo2dPt[14].x, &vo2dPt[14].y, &vfPsEst2dScr[14],
				&vo2dPt[15].x, &vo2dPt[15].y, &vfPsEst2dScr[15], &vo2dPt[16].x, &vo2dPt[16].y, &vfPsEst2dScr[16], &vo2dPt[17].x, &vo2dPt[17].y, &vfPsEst2dScr[17]);
		// MPI standard
		else if (1 == m_oCfg.getPsMdlTyp())
			std::sscanf(acBuf, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
				&nHumPsFrmCnt, &vo2dPt[0].x, &vo2dPt[0].y, &vfPsEst2dScr[0], &vo2dPt[1].x, &vo2dPt[1].y, &vfPsEst2dScr[1], &vo2dPt[2].x, &vo2dPt[2].y, &vfPsEst2dScr[2],
				&vo2dPt[3].x, &vo2dPt[3].y, &vfPsEst2dScr[3], &vo2dPt[4].x, &vo2dPt[4].y, &vfPsEst2dScr[4], &vo2dPt[5].x, &vo2dPt[5].y, &vfPsEst2dScr[5],
				&vo2dPt[6].x, &vo2dPt[6].y, &vfPsEst2dScr[6], &vo2dPt[7].x, &vo2dPt[7].y, &vfPsEst2dScr[7], &vo2dPt[8].x, &vo2dPt[8].y, &vfPsEst2dScr[8],
				&vo2dPt[9].x, &vo2dPt[9].y, &vfPsEst2dScr[9], &vo2dPt[10].x, &vo2dPt[10].y, &vfPsEst2dScr[10], &vo2dPt[11].x, &vo2dPt[11].y, &vfPsEst2dScr[11],
				&vo2dPt[12].x, &vo2dPt[12].y, &vfPsEst2dScr[12], &vo2dPt[13].x, &vo2dPt[13].y, &vfPsEst2dScr[13], &vo2dPt[14].x, &vo2dPt[14].y, &vfPsEst2dScr[14]);

		if (nHumPsFrmCnt >= nFrmCnt)
		{
			for (int i = 0; i < m_nJntPtNum; i++)
				voJntPt[i] = CJntPt(vo2dPt[i], cv::Point3f(0.0f, 0.0f, 0.0f), vfPsEst2dScr[i]);

			m_oHumPsNxt = CHumPs(nHumPsFrmCnt, voJntPt);

			if ((0 <= nHumPsFrmCnt) && (nHumPsFrmCnt <= nFrmCnt))
				voHumPs.push_back(m_oHumPsNxt);
		}
	}

	if (0 >= voHumPs.size())
		return CHumPs(nFrmCnt, voJntPt);
	if (1 == voHumPs.size())
		return voHumPs[0];
	else
		return fdMaxHumPs(voHumPs);
}

CHumPs CPsEst2d::rdPsEst2dJson(int nFrmCnt)
{
	std::vector<CHumPs> voHumPs;
	std::vector<cv::Point2f> vo2dPt;
	std::vector<float> vfJntPt;
	std::vector<CJntPt> voJntPt;

    for (int i = 0; i < m_nJntPtNum; i++)
        voJntPt.push_back(CJntPt());

	FILE * poJsonFl;
	int nHumPsPos = 0;
	long nlFlSz, nlRdRst;
	char * pcBuf;

	char acInJsonNm[128] = {};

	//if (0 == m_oCfg.getInVdoTyp())
	//{
	std::string strInVdoPth(m_oCfg.getInVdoPth());
	std::size_t iInVdoNmSt = strInVdoPth.find_last_of("/\\");
	std::size_t iInVdoNmNd = strInVdoPth.find_last_of(".");
	std::string strInVdoNm = strInVdoPth.substr((iInVdoNmSt + 1), (iInVdoNmNd - iInVdoNmSt - 1));
	std::sprintf(acInJsonNm, "%s_%012d_keypoints.json", strInVdoNm.c_str(), nFrmCnt);
	//}
	//else if (1 == m_oCfg.getInVdoTyp())
	//	std::sprintf(acInJsonNm, "%012d_keypoints.json", nFrmCnt);
	//else if ((2 == m_oCfg.getInVdoTyp()) || (3 == m_oCfg.getInVdoTyp()))
	//	std::sprintf(acInJsonNm, "%012d_keypoints.json", nFrmCnt);

	char acInJsonPth[128] = {};
	std::sprintf(acInJsonPth, m_oCfg.getPsEst2dJsonFlrPth());
	std::strcat(acInJsonPth, acInJsonNm);

	poJsonFl = std::fopen(acInJsonPth, "r");
	if (poJsonFl == NULL)
		return CHumPs(nFrmCnt, voJntPt);

	// obtain file size:
	fseek(poJsonFl, 0, SEEK_END);
	nlFlSz = ftell(poJsonFl);
	rewind(poJsonFl);

	// allocate memory to contain the whole file:
	pcBuf = (char*)malloc(sizeof(char)*nlFlSz);
	if (pcBuf == NULL) { fputs("Memory error", stderr); exit(2); }

	// copy the file into the buffer:
	nlRdRst = fread(pcBuf, 1, nlFlSz, poJsonFl);
	//if (nlRdRst != nlFlSz) { fputs("Reading error", stderr); exit(3); }

	std::string strJson(pcBuf);
	strJson.erase(std::remove_if(strJson.begin(), strJson.end(), [](char c) { return c >= 0 && isspace(c); }), strJson.end());	// in Windows
	//strJson.erase(std::remove_if(strJson.begin(), strJson.end(), ::isspace), strJson.end());	// in Linux

	while (std::string::npos != nHumPsPos)
	{
		nHumPsPos = strJson.find("\"pose_keypoints_2d\"", (nHumPsPos + 1));

		if (std::string::npos != nHumPsPos)
		{
			vfJntPt = m_oCfg.rdFltVec(strJson, nHumPsPos);

			for (int i = 0; i < m_nJntPtNum; i++) {
				if (2 == m_oCfg.getPsMdlTyp()) {
					int index = i > 7 ? i + 1 : i;
					voJntPt[i] = (CJntPt(cv::Point2f(vfJntPt[index * 3], vfJntPt[(index * 3) + 1]), cv::Point3f(0.0f, 0.0f, 0.0f), vfJntPt[(index * 3) + 2]));
				}
				else
					voJntPt[i] = (CJntPt(cv::Point2f(vfJntPt[i * 3], vfJntPt[(i * 3) + 1]), cv::Point3f(0.0f, 0.0f, 0.0f), vfJntPt[(i * 3) + 2]));
			}

            voHumPs.push_back(CHumPs(nFrmCnt, voJntPt));

            nHumPsPos = strJson.find("]", (nHumPsPos + 1));
		}
	}

	// terminate
	fclose(poJsonFl);
	free(pcBuf);

	if (0 >= voHumPs.size())
		return CHumPs(nFrmCnt, voJntPt);
	if (1 == voHumPs.size())
		return voHumPs[0];
	else
		return fdMaxHumPs(voHumPs);
}

CHumPs CPsEst2d::fdMaxHumPs(std::vector<CHumPs> voHumPs)
{
	int iMaxArea = -1, nArea, nMaxArea = 0;
	cv::Mat oImgMsk;
	cv::RotatedRect oMinAreaRect;
	cv::Point2f aoMinAreaRectPtFlt[4];
	cv::Point aoMinAreaRectPt[4];
	std::vector<CJntPt> voJntPt;
	std::vector<cv::Point2f> voJnt2dPt(m_nJntPtNum);

	for (int i = 0; i < voHumPs.size(); i++)
	{
		oImgMsk = cv::Mat::zeros(m_oCfg.getFrmSz(), CV_8UC1);
		voJntPt = voHumPs[i].getJntPtLs();

		for (int i = 0; i < m_nJntPtNum; i++)
            voJnt2dPt[i] = voJntPt[i].get2dPt();

		oMinAreaRect = cv::minAreaRect(voJnt2dPt);
		oMinAreaRect.points(aoMinAreaRectPtFlt);
		for (int i = 0; i < 4; i++)
			aoMinAreaRectPt[i] = aoMinAreaRectPtFlt[i];

		cv::fillConvexPoly(oImgMsk, aoMinAreaRectPt, 4, cv::Scalar(255));
		nArea = cv::countNonZero(oImgMsk);

		if (nMaxArea < nArea)
		{
			nMaxArea = nArea;
			iMaxArea = i;
		}
	}

	if (0 <= iMaxArea)
		return voHumPs[iMaxArea];
	else
		return CHumPs(m_nFrmCnt, voJntPt);
}

void CPsEst2d::trkMsJtPt(cv::Mat oImgFrmCurrGry)
{
	bool bTrkJntPtFlg;
	int nPsEst2dContFrmCntThld = m_oCfg.getPsEst2dContFrmCntThld(), nMtchFrmNumPrev;
	float fPsEst2dScrThld = m_oCfg.getPsEst2dScrThld(), fPsEst2dDistThld = m_oCfg.getPsEst2dDistThld();
	std::vector<CJntPt> voJntPt = m_oHumPs.getJntPtLs();
	std::vector<CJntPt> voJntPtPrev = m_oHumPsPrev.getJntPtLs();
	std::vector<cv::Point2f> voJnt2dPtPrev(m_nJntPtNum);
	std::vector<cv::Point2f> voJnt2dPtTrk;
	std::vector<uchar> vuStat;
	std::vector<float> vfErr;

	for (int i = 0; i < m_nJntPtNum; i++)
		voJnt2dPtPrev[i] = m_oHumPsPrev.getJntPt(i).get2dPt();

	// track previous feature points by KLT tracker
	cv::calcOpticalFlowPyrLK(m_oImgFrmPrevGry, oImgFrmCurrGry, voJnt2dPtPrev, voJnt2dPtTrk, vuStat, vfErr, KLT_TRK_WIN_SZ);

	for (int i = 0; i < m_nJntPtNum; i++)
	{   

		// the flag that the tracked joint point position is trust worthy
		// the previous joint point is trustworthy and it is tracked in the current frame
		bTrkJntPtFlg = (fPsEst2dScrThld <= voJntPtPrev[i].getPsEst2dScr()) && (0 != vuStat[i]);
		if (fPsEst2dScrThld <= voJntPtPrev[i].getPsEst2dScr())
		{
			m_oHumPs.setMtchFrmNum(i, nMtchFrmNumPrev);
		}
		/* the estimation score is too low that the detected joint pointis considered a false positive */
		else if ((fPsEst2dScrThld > voJntPt[i].getPsEst2dScr()) || 
			/* the detected position is too far away from the prediction of tracking */
			    (bTrkJntPtFlg && (fPsEst2dDistThld < cv::norm(voJntPt[i].get2dPt() - voJnt2dPtTrk[i]))))
		{
			    if (bTrkJntPtFlg && (fPsEst2dDistThld < cv::norm(voJntPt[i].get2dPt() - voJnt2dPtTrk[i])))
			    {
				    voJntPt[i].set2dPt(voJnt2dPtTrk[i]);
				    voJntPt[i].setPsEst2dScr(voJntPtPrev[i].getPsEst2dScr());
			    }
			    else
			    {
				    voJntPt[i].set2dPt(cv::Point2f(0.0f, 0.0f));
				    voJntPt[i].setPsEst2dScr(0.0f);
			    }

			    m_oHumPs.setMtchFrmNum(i, 0);
		}
		else
		{
			nMtchFrmNumPrev = m_oHumPsPrev.getMtchFrmNum(i);

			if (nPsEst2dContFrmCntThld > nMtchFrmNumPrev)
			{
				// the previous joint point is trustworthy and it is close to the current detected joint point
				if (((0 < voJnt2dPtPrev[i].x) || (0 < voJnt2dPtPrev[i].y)) && (fPsEst2dDistThld > cv::norm(voJntPt[i].get2dPt() - voJnt2dPtPrev[i])))
					m_oHumPs.setMtchFrmNum(i, (nMtchFrmNumPrev + 1));

				// wait for more continuous positions to be collected
				if (nPsEst2dContFrmCntThld > m_oHumPs.getMtchFrmNum(i))
				{
					if (bTrkJntPtFlg && (fPsEst2dDistThld < cv::norm(voJntPt[i].get2dPt() - voJnt2dPtTrk[i])))
					{
						voJntPt[i].set2dPt(voJnt2dPtTrk[i]);
						voJntPt[i].setPsEst2dScr(voJntPtPrev[i].getPsEst2dScr());
					}
					else
						voJntPt[i].setPsEst2dScr(0.0f);
				}
			}
			// accept the detected joint points
			else
				m_oHumPs.setMtchFrmNum(i, nMtchFrmNumPrev);
		}
	}

	m_oHumPs.setJntPtLs(voJntPt);
}

void CPsEst2d::plt2dPsMsk(cv::Mat& oImgMsk)
{
	int iMinDist = -1, iMaxDist = -1;
	float fDist, fMinDist = FLT_MAX, fMaxDist = 0.0f;
	std::vector<CJntPt> voJntPt = m_oHumPs.getJntPtLs();
	std::vector<cv::Point2f> voTor2dPt;
	cv::RotatedRect oMinAreaRect;
    cv::Point2f aoMinAreaRectPtFlt[4];
    cv::Point aoMinAreaRectPt[4];

	oImgMsk = cv::Mat(m_oCfg.getFrmSz(), CV_8UC1, cv::Scalar(255));

	// plot the right upper arm
	if ((0 < voJntPt[2].getPsEst2dScr()) && (0 < voJntPt[3].getPsEst2dScr()))
		cv::line(oImgMsk, voJntPt[2].get2dPt(), voJntPt[3].get2dPt(), cv::Scalar(0), (cv::norm(voJntPt[2].get2dPt() - voJntPt[3].get2dPt()) * PLT_MSK_LMB_AR));

	// plot the right lower arm
	if ((0 < voJntPt[3].getPsEst2dScr()) && (0 < voJntPt[4].getPsEst2dScr()))
		cv::line(oImgMsk, voJntPt[3].get2dPt(), voJntPt[4].get2dPt(), cv::Scalar(0), (cv::norm(voJntPt[3].get2dPt() - voJntPt[4].get2dPt()) * PLT_MSK_LMB_AR));

	// plot the left upper arm
	if ((0 < voJntPt[5].getPsEst2dScr()) && (0 < voJntPt[6].getPsEst2dScr()))
		cv::line(oImgMsk, voJntPt[5].get2dPt(), voJntPt[6].get2dPt(), cv::Scalar(0), (cv::norm(voJntPt[5].get2dPt() - voJntPt[6].get2dPt()) * PLT_MSK_LMB_AR));

	// plot the left lower arm
	if ((0 < voJntPt[6].getPsEst2dScr()) && (0 < voJntPt[7].getPsEst2dScr()))
		cv::line(oImgMsk, voJntPt[6].get2dPt(), voJntPt[7].get2dPt(), cv::Scalar(0), (cv::norm(voJntPt[6].get2dPt() - voJntPt[7].get2dPt()) * PLT_MSK_LMB_AR));

	// plot the right thigh
	if ((0 < voJntPt[8].getPsEst2dScr()) && (0 < voJntPt[9].getPsEst2dScr()))
		cv::line(oImgMsk, voJntPt[8].get2dPt(), voJntPt[9].get2dPt(), cv::Scalar(0), (cv::norm(voJntPt[8].get2dPt() - voJntPt[9].get2dPt()) * PLT_MSK_LMB_AR));

	// plot the right calf
	if ((0 < voJntPt[9].getPsEst2dScr()) && (0 < voJntPt[10].getPsEst2dScr()))
		cv::line(oImgMsk, voJntPt[9].get2dPt(), voJntPt[10].get2dPt(), cv::Scalar(0), (cv::norm(voJntPt[9].get2dPt() - voJntPt[10].get2dPt()) * PLT_MSK_LMB_AR));

	// plot the left thigh
	if ((0 < voJntPt[11].getPsEst2dScr()) && (0 < voJntPt[12].getPsEst2dScr()))
		cv::line(oImgMsk, voJntPt[11].get2dPt(), voJntPt[12].get2dPt(), cv::Scalar(0), (cv::norm(voJntPt[11].get2dPt() - voJntPt[12].get2dPt()) * PLT_MSK_LMB_AR));

	// plot the left calf
	if ((0 < voJntPt[12].getPsEst2dScr()) && (0 < voJntPt[13].getPsEst2dScr()))
		cv::line(oImgMsk, voJntPt[12].get2dPt(), voJntPt[13].get2dPt(), cv::Scalar(0), (cv::norm(voJntPt[12].get2dPt() - voJntPt[13].get2dPt()) * PLT_MSK_LMB_AR));

	// COCO standard
	if (0 == m_oCfg.getPsMdlTyp() || 2 == m_oCfg.getPsMdlTyp())
	{
		// plot the head
		// if the neck point exists, find the face point that is closest to the neck as the center of the circle
		if (0 < voJntPt[1].getPsEst2dScr())
		{
			for (int i = 0; i < PS_COCO_JNTPT_NUM; i++)
			{
				if ((0 == i) || ((14 <= i) && (17 >= i)))
				{
					if (0 < voJntPt[i].getPsEst2dScr())
					{
						fDist = cv::norm(voJntPt[i].get2dPt() - voJntPt[1].get2dPt());
						if (fMinDist > fDist)
						{
							fMinDist = fDist;
							iMinDist = i;
						}
					}
				}
			}

			if (0 <= iMinDist)
				cv::circle(oImgMsk, voJntPt[iMinDist].get2dPt(),
					cv::norm(voJntPt[iMinDist].get2dPt() - voJntPt[1].get2dPt()), cv::Scalar(0), -1);
		}
		// use nose or one of the ears to be the center of the circle
		else
		{
			if ((0 < voJntPt[0].getPsEst2dScr()) && (0 < voJntPt[16].getPsEst2dScr()) && (0 < voJntPt[17].getPsEst2dScr()))
			{
				fDist = cv::norm(voJntPt[0].get2dPt() - voJntPt[16].get2dPt());
				if (fMaxDist < fDist)
				{
					fMaxDist = fDist;
					iMaxDist = 16;
				}

				fDist = cv::norm(voJntPt[0].get2dPt() - voJntPt[17].get2dPt());
				if (fMaxDist < fDist)
				{
					fMaxDist = fDist;
					iMaxDist = 17;
				}

				if (0 <= iMinDist)
					cv::circle(oImgMsk, voJntPt[0].get2dPt(),
						cv::norm(voJntPt[0].get2dPt() - voJntPt[iMaxDist].get2dPt()), cv::Scalar(0), -1);
			}
			else if ((0 < voJntPt[0].getPsEst2dScr()) && (0 < voJntPt[16].getPsEst2dScr()))
				cv::circle(oImgMsk, voJntPt[16].get2dPt(),
					cv::norm(voJntPt[0].get2dPt() - voJntPt[16].get2dPt()), cv::Scalar(0), -1);
			else if ((0 < voJntPt[0].getPsEst2dScr()) && (0 < voJntPt[17].getPsEst2dScr()))
				cv::circle(oImgMsk, voJntPt[17].get2dPt(),
					cv::norm(voJntPt[0].get2dPt() - voJntPt[17].get2dPt()), cv::Scalar(0), -1);
			else if ((0 < voJntPt[16].getPsEst2dScr()) && (0 < voJntPt[17].getPsEst2dScr()))
				cv::circle(oImgMsk, cv::Point(((voJntPt[16].get2dPt().x + voJntPt[17].get2dPt().x) / 2), ((voJntPt[16].get2dPt().y + voJntPt[17].get2dPt().y) / 2)),
					(cv::norm(voJntPt[16].get2dPt() - voJntPt[17].get2dPt()) / 2), cv::Scalar(0), -1);
		}

        // plot the torso
        for (int i = 0; i < PS_COCO_JNTPT_NUM; i++)
        {
            if ((1 == i) || (2 == i) || (5 == i) || (8 == i) || (11 == i))
            {
                if (0 < voJntPt[i].getPsEst2dScr())
                    voTor2dPt.push_back(voJntPt[i].get2dPt());
            }
        }

        if (2 < voTor2dPt.size())
        {
            oMinAreaRect = cv::minAreaRect(voTor2dPt);
            oMinAreaRect.points(aoMinAreaRectPtFlt);
            for (int i = 0; i < 4; i++)
                aoMinAreaRectPt[i] = aoMinAreaRectPtFlt[i];
            cv::fillConvexPoly(oImgMsk, aoMinAreaRectPt, 4, cv::Scalar(0));
        }
        else if (2 == voTor2dPt.size())
            cv::line(oImgMsk, voTor2dPt[0], voTor2dPt[1], cv::Scalar(0), (cv::norm(voTor2dPt[0] - voTor2dPt[1]) * PLT_MSK_LMB_AR));
	}
	// MPI standard
	else if (1 == m_oCfg.getPsMdlTyp())
	{
		// plot the head
		if ((0 < voJntPt[0].getPsEst2dScr()) && (0 < voJntPt[1].getPsEst2dScr()))
			cv::circle(oImgMsk, cv::Point(((voJntPt[0].get2dPt().x + voJntPt[1].get2dPt().x) / 2), ((voJntPt[0].get2dPt().y + voJntPt[1].get2dPt().y) / 2)),
				(cv::norm(voJntPt[0].get2dPt() - voJntPt[1].get2dPt()) / 2), cv::Scalar(0), -1);

		// plot the torso
		for (int i = 0; i < PS_MPI_JNTPT_NUM; i++)
		{
			if ((1 == i) || (2 == i) || (5 == i) || (8 == i) || (11 == i) || (14 == i))
			{
				if (0 < voJntPt[i].getPsEst2dScr())
					voTor2dPt.push_back(voJntPt[i].get2dPt());
			}
		}

		if (2 < voTor2dPt.size())
		{
            oMinAreaRect = cv::minAreaRect(voTor2dPt);
            oMinAreaRect.points(aoMinAreaRectPtFlt);
            for (int i = 0; i < 4; i++)
                aoMinAreaRectPt[i] = aoMinAreaRectPtFlt[i];
            cv::fillConvexPoly(oImgMsk, aoMinAreaRectPt, 4, cv::Scalar(0));
		}
		else if (2 == voTor2dPt.size())
			cv::line(oImgMsk, voTor2dPt[0], voTor2dPt[1], cv::Scalar(0), (cv::norm(voTor2dPt[0] - voTor2dPt[1]) * PLT_MSK_LMB_AR));
	}

	// erode the mask
	cv::erode(oImgMsk, oImgMsk, cv::Mat(), cv::Point(-1, -1), PLT_MSK_ERO_ITER_NUM);

    //// debug
    ////cv::imwrite(".\\data\\msk.jpg", oImgMsk); // in Windows
    //cv::imwrite("./data/msk.jpg", oImgMsk); // in Linux
    //cv::waitKey(1);
}

void CPsEst2d::outTxt(void)
{
	CJntPt oJntPt;
	cv::Point2f oJnt2dPt;

	FILE* pfOutPsEst2dTxt = std::fopen(m_oCfg.getPsEst2dTxtPth(), "a");

	std::fprintf(pfOutPsEst2dTxt, "%d", m_nFrmCnt);
	for (int i = 0; i < m_nJntPtNum; i++)
	{
		oJntPt = m_oHumPs.getJntPt(i);
		oJnt2dPt = oJntPt.get2dPt();
		std::fprintf(pfOutPsEst2dTxt, ",%.3f,%.3f,%.5f", oJnt2dPt.x, oJnt2dPt.y, oJntPt.getPsEst2dScr());
	}
	std::fprintf(pfOutPsEst2dTxt, "\n");

	std::fclose(pfOutPsEst2dTxt);
}

void CPsEst2d::pltPsEst2d(cv::Mat& oImgFrm, cv::Mat oImgMsk)
{
	CJntPt oJntPt0, oJntPt1;
	cv::Scalar oPltClr;
	cv::Mat oImgMsk3Ch;

	// plot each joint point
	for (int i = 0; i < m_nJntPtNum; i++)
	{
		oJntPt0 = m_oHumPs.getJntPt(i);

		if (0 < oJntPt0.getPsEst2dScr())
		{
			// COCO standard
			if (0 == m_oCfg.getPsMdlTyp() || 2 == m_oCfg.getPsMdlTyp())
				oPltClr = PS_COCO_CLRS[i];
			// MPI standard
			else if (1 == m_oCfg.getPsMdlTyp())
				oPltClr = PS_MPI_CLRS[i];

			cv::circle(oImgFrm, oJntPt0.get2dPt(), PLT_PS_JNTPT_RAD, oPltClr, -1);
		}
	}

	// plot each link between joint points
	for (int i = 0; i < (m_nJntPtNum - 1); i++)
	{
		// COCO standard
		if (0 == m_oCfg.getPsMdlTyp() || 2 == m_oCfg.getPsMdlTyp())
		{
			oJntPt0 = m_oHumPs.getJntPt(PS_COCO_PRS[i*2]);
			oJntPt1 = m_oHumPs.getJntPt(PS_COCO_PRS[(i*2)+1]);
		}
		// MPI standard
		else if (1 == m_oCfg.getPsMdlTyp())
		{
			oJntPt0 = m_oHumPs.getJntPt(PS_MPI_PRS[i*2]);
			oJntPt1 = m_oHumPs.getJntPt(PS_MPI_PRS[(i*2)+1]);
		}

		if ((0 < oJntPt0.getPsEst2dScr()) && (0 < oJntPt1.getPsEst2dScr()))
		{
			// COCO standard
			if (0 == m_oCfg.getPsMdlTyp() || 2 == m_oCfg.getPsMdlTyp())
				oPltClr = PS_COCO_CLRS[i];
			// MPI standard
			else if (1 == m_oCfg.getPsMdlTyp())
				oPltClr = PS_MPI_CLRS[i];

			cv::line(oImgFrm, oJntPt0.get2dPt(), oJntPt1.get2dPt(), oPltClr, PLT_PS_JNTLN_THK);
		}
	}

    // plot the mask of 2D human pose
	if (1 == m_oCfg.getInCalTyp())
	{
		cv::cvtColor(oImgMsk, oImgMsk3Ch, cv::COLOR_GRAY2BGR);
		cv::addWeighted(oImgFrm, (1.0f - PLT_MSK_BLN_RAT), oImgMsk3Ch, PLT_MSK_BLN_RAT, 0.0, oImgFrm);
	}
}
