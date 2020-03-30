#include "VisOdom.h"

CVisOdom::CVisOdom(void)
{
	// list of feature points in previous frame
	std::vector<cv::KeyPoint>().swap(m_voFeatPtPrev);

	// list of 2D points in previous frame
	std::vector<cv::Point2f>().swap(m_vo2dPtPrev);

	// list of 3D points
	std::vector<cv::Point3f>().swap(m_vo3dPt);
}

CVisOdom::~CVisOdom(void)
{
	// input camera parameters
	m_ifsInCamParam.close();

	// list of feature points in previous frame
	std::vector<cv::KeyPoint>().swap(m_voFeatPtPrev);

	// list of 2D points in previous frame
	std::vector<cv::Point2f>().swap(m_vo2dPtPrev);

	// list of 3D points
	std::vector<cv::Point3f>().swap(m_vo3dPt);
}

void CVisOdom::initialize(CCfg oCfg)
{
	// configuration file
	m_oCfg = oCfg;

	// current frame count
	m_nFrmCnt = -1;

	// flag of solving PnP problem using RANSAC
	m_bRansac = false;

	if (0 == m_oCfg.getInCalTyp())
	{
		// previous camera rotation matrix
		m_oCamRotMatPrev = m_oCfg.getCamRotMatInit();

		// previous camera translation matrix
		m_oCamTntMatPrev = m_oCfg.getCamTntMatInit();

		// updated camera rotation matrix
		m_oCamRotMatUpd = m_oCfg.getCamRotMatInit();

		// updated camera translation matrix
		m_oCamTntMatUpd = m_oCfg.getCamTntMatInit();
		// input camera parameters
		std::string strInCamParamPth = m_oCfg.getCamParamPth();
		m_ifsInCamParam.open(strInCamParamPth.c_str());
	}
	else if (1 == m_oCfg.getInCalTyp())
	{
		// previous camera rotation matrix
		m_oCamRotMatPrev = m_oCfg.getCamRotMatInit();

		// previous camera translation matrix
		m_oCamTntMatPrev = m_oCfg.getCamTntMatInit();

		// updated camera rotation matrix
		m_oCamRotMatUpd = m_oCfg.getCamRotMatInit();

		// updated camera translation matrix
		m_oCamTntMatUpd = m_oCfg.getCamTntMatInit();

		// the previous camera projection matrix
		cv::Mat oCamRotVec;
		cv::Rodrigues(m_oCamRotMatUpd, oCamRotVec);
		cvtRotVecTntMat2ExtMatx(oCamRotVec, m_oCamTntMatUpd, m_oCamExtMatxUpd);

		// the previous frame in grey scale
		m_oImgFrmGryPrev = cv::Mat::zeros(m_oCfg.getFrmSz(), CV_8UC1);

		// list of feature points in previous frame
		std::vector<cv::KeyPoint>().swap(m_voFeatPtPrev);

		// list of 2D points in previous frame
		std::vector<cv::Point2f>().swap(m_vo2dPtPrev);

		// list of 3D points
		std::vector<cv::Point3f>().swap(m_vo3dPt);

		// FAST feature detector
		m_poFastFeatDet = cv::FastFeatureDetector::create(m_oCfg.getVisOdomFastThld(), FAST_NMS_FLG);

		// output text file
		if (m_oCfg.getOutCamParamFlg())
		{
			FILE* pfOutCamParamTxt = std::fopen(m_oCfg.getCamParamPth(), "w");
			std::fclose(pfOutCamParamTxt);
		}
	}
}

cv::Matx34d CVisOdom::process(cv::Mat oImgFrmCurr, cv::Mat oImgMsk, int nFrmCnt)
{
	float fVisOdomSclFac = m_oCfg.getVisOdomSclFac();

	// read camera parameters
	if (0 == m_oCfg.getInCalTyp())
		rdCamParam(nFrmCnt);
	// run visual odometry
	else if (1 == m_oCfg.getInCalTyp())
		rnVisOdom(oImgFrmCurr, oImgMsk, nFrmCnt);

	return m_oCamExtMatxUpd;
}

void CVisOdom::updCamExtMat(cv::Matx34d oCamExtMatx)
{
	cv::Mat oCamRotMatUpdPrev, oCamTntMatUpdPrev, oCamRotVec;

	// update extrinsic matrix
	m_oCamExtMatxUpd = oCamExtMatx;

	// update rotation matrix
	oCamRotMatUpdPrev = m_oCamRotMatPrev.inv() * m_oCamRotMatUpd;
	oCamTntMatUpdPrev = m_oCamTntMatUpd - (m_oCamRotMatUpd * m_oCamTntMatPrev);
	cvtExtMatx2RotVecTntMat(oCamExtMatx, oCamRotVec, m_oCamTntMatUpd);
	cv::Rodrigues(oCamRotVec, m_oCamRotMatUpd);
	m_oCamRotMatPrev = m_oCamRotMatUpd * oCamRotMatUpdPrev.inv();
	m_oCamTntMatPrev = m_oCamRotMatUpd.inv() * (m_oCamTntMatUpd - oCamTntMatUpdPrev);
}

void CVisOdom::output(cv::Mat& oImgFrm)
{
	// output text file
	if (m_oCfg.getOutCamParamFlg())
		outTxt();

	//// plot feature points from visual odometry
	//if (m_oCfg.getPltVisOdomFlg())
	//	cv::drawKeypoints(oImgFrm, m_voFeatPtPrev, oImgFrm, PLT_KEYPT_CLR, cv::DrawMatchesFlags::DEFAULT);
}

void CVisOdom::rdCamParam(int nFrmCntCurr)
{
	char acBuf [256];
	int nFrmCnt = -1;
	cv::Mat oCamRotVec = cv::Mat::zeros(3, 1, CV_64F), oCamTntVec = cv::Mat::zeros(3, 1, CV_64F), oCamTntMat;

	// assume that all the frame counts are continuous
	while ((!m_ifsInCamParam.eof()) && (nFrmCnt < nFrmCntCurr))
	{
		m_ifsInCamParam.getline(acBuf, 256);
		std::sscanf(acBuf, "%d,%f,%f,%f,%f,%f,%f", &nFrmCnt, &oCamRotVec.at<double>(0), &oCamRotVec.at<double>(1), &oCamRotVec.at<double>(2),
			&oCamTntVec.at<double>(0), &oCamTntVec.at<double>(1), &oCamTntVec.at<double>(2));
		cvtTntVec2Mat(oCamRotVec, oCamTntVec, oCamTntMat);
		cvtRotVecTntMat2ExtMatx(oCamRotVec, oCamTntMat, m_oCamExtMatxUpd);
		updCamExtMat(m_oCamExtMatxUpd);
	}
}

void CVisOdom::rnVisOdom(cv::Mat oImgFrmCurr, cv::Mat oImgMsk, int nFrmCnt)
{
	m_nFrmCnt = nFrmCnt;
	cv::Mat oImgFrmBlrCurr, oImgFrmGryCurr;
	cv::Mat oCamIntMat = m_oCfg.getCamIntMat(), oCamRotMat = m_oCamRotMatPrev.clone(), oCamTntMat = m_oCamTntMatPrev.clone(), oCamRotVec, oCamRotMatTrans, oEssMat;
	cv::Mat o4dPtMat, o3dPtMat, o2dProjPtMat;
	cv::Matx33d oCamIntMatx;
	cv::Matx34d oCamProjMatx, oCamProjMatxPrev;
	std::vector<uchar> vuStat;
	std::vector<float> vfErr;
	std::vector<cv::KeyPoint> voFeatPtCurr, voFeatPtCurrTrk;
	std::vector<cv::Point2f> vo2dPtCurr, vo2dPtCurrTrk;
	double fFocLen = (oCamIntMat.at<double>(0, 0) + oCamIntMat.at<double>(1, 1)) / 2.0;
	cv::Point2d oPrinPt = cv::Point2d(oCamIntMat.at<double>(0, 2), oCamIntMat.at<double>(1, 2));
	float fVisOdomSclFac = m_oCfg.getVisOdomSclFac();

	cv::GaussianBlur(oImgFrmCurr, oImgFrmBlrCurr, GAUSS_BLR_SZ, 0, 0);
	cv::cvtColor(oImgFrmBlrCurr, oImgFrmGryCurr, cv::COLOR_BGR2GRAY);

	// starting from the second frame, there is previous frame image
	if (m_oCfg.getProcStFrmCnt() < nFrmCnt)
	{
		// track previous feature points by KLT tracker
		cv::calcOpticalFlowPyrLK(m_oImgFrmGryPrev, oImgFrmGryCurr, m_vo2dPtPrev, vo2dPtCurrTrk, vuStat, vfErr, KLT_TRK_WIN_SZ);

		// erase untracked feature points
		ersUntrkFeatPt(vo2dPtCurrTrk, vuStat, vfErr);
		for (int i = 0; i < vo2dPtCurrTrk.size(); i++)
			voFeatPtCurrTrk.push_back(cv::KeyPoint(vo2dPtCurrTrk[i], 1.0f));

		if (m_oCfg.getVisOdomRansacFlg() && m_bRansac)
		{
			// update rotation matrix and translation matrix by solving PnP problem using RANSAC
			cv::Rodrigues(oCamRotMat, oCamRotVec);
			cv::solvePnPRansac(m_vo3dPt, vo2dPtCurrTrk, oCamIntMat, cv::Mat::zeros(4, 1, CV_64F), oCamRotVec, oCamTntMat,
				true, RANSAC_ITER_NUM, RANSAC_REPROJ_ERR_THLD);
			cv::Rodrigues(oCamRotVec, oCamRotMat);
		}
		else
		{
			// find essential matrix
			oEssMat = cv::findEssentialMat(vo2dPtCurrTrk, m_vo2dPtPrev, fFocLen, oPrinPt);

			// get rotation matrix and translation matrix
			cv::recoverPose(oEssMat, vo2dPtCurrTrk, m_vo2dPtPrev, oCamRotMat, oCamTntMat, fFocLen, oPrinPt);
		}

		if (m_oCfg.getVisOdomRansacFlg())
		{
			// build matrices
			oCamIntMatx = cv::Matx33d(oCamIntMat.at<double>(0, 0), oCamIntMat.at<double>(0, 1), oCamIntMat.at<double>(0, 2),
				oCamIntMat.at<double>(1, 0), oCamIntMat.at<double>(1, 1), oCamIntMat.at<double>(1, 2),
				oCamIntMat.at<double>(2, 0), oCamIntMat.at<double>(2, 1), oCamIntMat.at<double>(2, 2));
			oCamProjMatxPrev = cv::Matx34d(m_oCamRotMatPrev.at<double>(0, 0), m_oCamRotMatPrev.at<double>(0, 1), m_oCamRotMatPrev.at<double>(0, 2), m_oCamTntMatPrev.at<double>(0),
				m_oCamRotMatPrev.at<double>(1, 0), m_oCamRotMatPrev.at<double>(1, 1), m_oCamRotMatPrev.at<double>(1, 2), m_oCamTntMatPrev.at<double>(1),
				m_oCamRotMatPrev.at<double>(2, 0), m_oCamRotMatPrev.at<double>(2, 1), m_oCamRotMatPrev.at<double>(2, 2), m_oCamTntMatPrev.at<double>(2));
			oCamProjMatx = cv::Matx34d(oCamRotMat.at<double>(0, 0), oCamRotMat.at<double>(0, 1), oCamRotMat.at<double>(0, 2), oCamTntMat.at<double>(0),
				oCamRotMat.at<double>(1, 0), oCamRotMat.at<double>(1, 1), oCamRotMat.at<double>(1, 2), oCamTntMat.at<double>(1),
				oCamRotMat.at<double>(2, 0), oCamRotMat.at<double>(2, 1), oCamRotMat.at<double>(2, 2), oCamTntMat.at<double>(2));

			// triangulate points
			cv::triangulatePoints((oCamIntMatx * oCamProjMatxPrev), (oCamIntMatx * oCamProjMatx), m_vo2dPtPrev, vo2dPtCurrTrk, o4dPtMat);

			// convert points from homogeneous coordinate system
			cv::convertPointsFromHomogeneous(o4dPtMat.t(), o3dPtMat);
			o3dPtMat.copyTo(m_vo3dPt);
		}

		// update camera matrices
		m_oCamTntMatPrev = oCamTntMat.clone();
		m_oCamRotMatPrev = oCamRotMat.clone();
		m_oCamTntMatUpd = m_oCamTntMatUpd + (m_oCamRotMatUpd * oCamTntMat * fVisOdomSclFac);
		m_oCamRotMatUpd = oCamRotMat * m_oCamRotMatUpd;
		cv::Rodrigues(m_oCamRotMatUpd, oCamRotVec);
		cvtRotVecTntMat2ExtMatx(oCamRotVec, m_oCamTntMatUpd, m_oCamExtMatxUpd);

		// too few points are tracked -> detect new key points
		if (m_oCfg.getVisOdomMinFeatNum() > vo2dPtCurrTrk.size())
			m_bRansac = false;
		else
			m_bRansac = true;
	}

	m_oImgFrmGryPrev = oImgFrmGryCurr;

	if (!m_bRansac)
	{
		// detect feature points in the frame image
		m_poFastFeatDet->detect(oImgFrmGryCurr, voFeatPtCurr);
		cv::KeyPoint::convert(voFeatPtCurr, vo2dPtCurr, std::vector<int>());

		// remove feature points by mask
		if (m_oCfg.getVisOdomMsk2dPsFlg())
			rmvFeatPtMsk(oImgMsk, voFeatPtCurr, vo2dPtCurr);

		// combine detected and tracked feature points
		combDetTrkFeatPt(voFeatPtCurr, vo2dPtCurr, voFeatPtCurrTrk, vo2dPtCurrTrk);

		m_voFeatPtPrev = voFeatPtCurr;
		m_vo2dPtPrev = vo2dPtCurr;
	}
	else
	{
		// remove feature points by mask
		if (m_oCfg.getVisOdomMsk2dPsFlg())
			rmvFeatPtMsk(oImgMsk, voFeatPtCurrTrk, vo2dPtCurrTrk, true);

		m_voFeatPtPrev = voFeatPtCurrTrk;
		m_vo2dPtPrev = vo2dPtCurrTrk;
	}
}

void CVisOdom::rmvFeatPtMsk(cv::Mat oImgMsk, std::vector<cv::KeyPoint>& voFeatPt, std::vector<cv::Point2f>& vo2dPt, bool bRmv3dPtFlg)
{
	bool bRmvFlg;
	cv::Point o2dPt;
	std::vector<cv::KeyPoint> voFeatPtCp = voFeatPt;
	std::vector<cv::Point2f> vo2dPtCp = vo2dPt;
	std::vector<cv::Point3f> vo3dPtCp = m_vo3dPt;

	for (int i = voFeatPt.size() - 1; i >= 0; i--)
	{
		bRmvFlg = false;
		o2dPt = vo2dPt[i];

		if ((o2dPt.x < 0) || (o2dPt.y < 0) || (o2dPt.x >= m_oCfg.getFrmSz().width) || (o2dPt.y >= m_oCfg.getFrmSz().height))
			bRmvFlg = true;
		else if (!oImgMsk.at<uchar>(o2dPt))
			bRmvFlg = true;

		if (bRmvFlg)
		{
			voFeatPtCp.erase(voFeatPtCp.begin() + i);
			vo2dPtCp.erase(vo2dPtCp.begin() + i);
			if (m_oCfg.getVisOdomRansacFlg() && bRmv3dPtFlg)
				vo3dPtCp.erase(vo3dPtCp.begin() + i);
		}
	}

	// if the number of feature points is too small, use all the detected feature points instead
	if (m_oCfg.getVisOdomMinFeatNum() < voFeatPtCp.size())
	{
		voFeatPt = voFeatPtCp;
		vo2dPt = vo2dPtCp;
		if (m_oCfg.getVisOdomRansacFlg() && bRmv3dPtFlg)
			m_vo3dPt = vo3dPtCp;
	}
}

void CVisOdom::ersUntrkFeatPt(std::vector<cv::Point2f>& vo2dPtTrk, std::vector<uchar>& vuStat, std::vector<float>& vfErr)
{
    cv::Point2f o2dPtTrk;
	cv::Size oFrmSz = m_oCfg.getFrmSz();

	for (int i = vuStat.size() - 1; i >= 0; i--)
	{
		o2dPtTrk = vo2dPtTrk[i];

		if ((0 == vuStat[i]) || (0 > o2dPtTrk.x) || (oFrmSz.width <= o2dPtTrk.x) || (0 > o2dPtTrk.y) || (oFrmSz.height <= o2dPtTrk.y))
		{
			vo2dPtTrk.erase(vo2dPtTrk.begin() + i);
			vuStat.erase(vuStat.begin() + i);
			vfErr.erase(vfErr.begin() + i);

			m_voFeatPtPrev.erase(m_voFeatPtPrev.begin() + i);
			m_vo2dPtPrev.erase(m_vo2dPtPrev.begin() + i);
			if (m_oCfg.getVisOdomRansacFlg() && m_bRansac)
				m_vo3dPt.erase(m_vo3dPt.begin() + i);
		}
	}
}

void CVisOdom::combDetTrkFeatPt(std::vector<cv::KeyPoint>& voFeatPt, std::vector<cv::Point2f>& vo2dPt,
	std::vector<cv::KeyPoint> voFeatPtTrk, std::vector<cv::Point2f> vo2dPtTrk)
{
	cv::Size oFrmSz = m_oCfg.getFrmSz();

	// erase detected feature points overlapped with tracked feature points
	for (int i = vo2dPt.size() - 1; i >= 0; i--)
	{
		for (int j = 0; j < vo2dPtTrk.size(); j++)
		{
			if (m_oCfg.getVisOdomMinFeatDist() > cv::norm(vo2dPt[i] - vo2dPtTrk[j]))
			{
				vo2dPt.erase(vo2dPt.begin() + i);
				voFeatPt.erase(voFeatPt.begin() + i);
				break;
			}
		}
	}

	// merge detected and tracked feature points
	voFeatPt.insert(voFeatPt.end(), voFeatPtTrk.begin(), voFeatPtTrk.end());
	vo2dPt.insert(vo2dPt.end(), vo2dPtTrk.begin(), vo2dPtTrk.end());
}

void CVisOdom::outTxt(void)
{
	cv::Mat oCamRotVec, oCamTntVec;
	cv::Rodrigues(m_oCamRotMatUpd, oCamRotVec);
	cvtTntMat2Vec(oCamRotVec, m_oCamTntMatUpd, oCamTntVec);

	FILE* pfOutCamParamTxt = std::fopen(m_oCfg.getCamParamPth(), "a");
	std::fprintf(pfOutCamParamTxt, "%d,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n",
		m_nFrmCnt, oCamRotVec.at<double>(0), oCamRotVec.at<double>(1), oCamRotVec.at<double>(2),
		oCamTntVec.at<double>(0), oCamTntVec.at<double>(1), oCamTntVec.at<double>(2));
	std::fclose(pfOutCamParamTxt);
}
