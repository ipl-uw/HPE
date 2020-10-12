#include "PsEst3d.h"

//! compares the cost
bool compCost(COptParam oOptParam1, COptParam oOptParam2) 
{ 
	return (oOptParam1.getCost() < oOptParam2.getCost()); 
}

COptParam::COptParam(void)
{
	resetJntPtDepLs();
}

COptParam::COptParam(cv::Vec3d ovRotVec, cv::Vec3d ovTntMat, std::vector<float> vfJntPtDep)
{
	setRotVec(ovRotVec);
	setTntMat(ovTntMat);
	setJntPtDepLs(vfJntPtDep);
}

COptParam::~COptParam(void)
{
	resetJntPtDepLs();
}

CPsEst3d::CPsEst3d(void)
{
	// list of previous human poses
	std::vector<CHumPs>().swap(m_voHumPsPrev);

	// moving average of 3D joint points (additional number count)
	std::vector<cv::Vec4f>().swap(m_voJnt3dPtAvg);
}

CPsEst3d::~CPsEst3d(void)
{
	// list of previous human poses
	std::vector<CHumPs>().swap(m_voHumPsPrev);

	// moving average of 3D joint points (additional number count)
	std::vector<cv::Vec4f>().swap(m_voJnt3dPtAvg);
}

void CPsEst3d::initialize(CCfg oCfg)
{
	// configuration file
	m_oCfg = oCfg;

	// current frame count
	m_nFrmCnt = -1;

	// number of joint points corresponding to different standards
	// COCO standard
	if (0 == m_oCfg.getPsMdlTyp() || 2 == m_oCfg.getPsMdlTyp())
		m_nJntPtNum = PS_COCO_JNTPT_NUM;
	// MPI standard
	else if (1 == m_oCfg.getPsMdlTyp())
		m_nJntPtNum = PS_MPI_JNTPT_NUM;
	else if (3 == m_oCfg.getPsMdlTyp())
		m_nJntPtNum = PS_INFANT_JNTPT_NUM;

	// updated camera extrinsic matrix
	m_oCamExtMatxUpd = cv::Matx34d(1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0);

	// list of previous human poses
	std::vector<CHumPs>().swap(m_voHumPsPrev);

	// moving average of 3D joint points (additional number count)
	for (int i = 0; i < m_nJntPtNum; i++)
		m_voJnt3dPtAvg.push_back(cv::Vec4f(0.0f, 0.0f, 0.0f, 0.0f));

	// output text file
	if (m_oCfg.getOutPsEst3dFlg())
	{
		FILE* pfOutPsEst3dTxt = std::fopen(m_oCfg.getOutPsEst3dPth(), "w");
		std::fclose(pfOutPsEst3dTxt);
	}

	if ((2 == m_oCfg.getInCalTyp()) && (m_oCfg.getOutCamParamFlg()))
	{
		FILE* pfOutCamParamTxt = std::fopen(m_oCfg.getCamParamPth(), "w");
		std::fclose(pfOutCamParamTxt);
	}
}

void CPsEst3d::process(CHumPs& oHumPs, cv::Matx34d& oCamExtMatx, int nFrmCnt)
{
    m_nFrmCnt = nFrmCnt;

	// run 3D pose estimation through EDA optimization
	rnPsEst3dEda(oHumPs, oCamExtMatx);
}

void CPsEst3d::output(void)
{
	// output text file
	if (m_oCfg.getOutPsEst3dFlg() || ((2 == m_oCfg.getInCalTyp()) && (m_oCfg.getOutCamParamFlg())))
		outTxt();
}

void CPsEst3d::rnPsEst3dEda(CHumPs& oHumPs, cv::Matx34d& oCamExtMatx)
{
	std::vector<int> vnPsEst3dMaxIterNum = m_oCfg.getPsEst3dMaxIterNum();
	int nPsEst3dMaxIterNum, nPsEst3dTmpConstRefTyp = m_oCfg.getPsEst3dTmpConstRefTyp(), iIter = 0;
	float fStpCritCostRat = m_oCfg.getPsEst3dStpCritCostRat()[2];
	double fCostMean, fCostMeanPrev;

	if (m_oCfg.getProcStFrmCnt() == m_nFrmCnt)
		nPsEst3dMaxIterNum = vnPsEst3dMaxIterNum[0];
	else
		nPsEst3dMaxIterNum = vnPsEst3dMaxIterNum[1];

	if (2 == m_oCfg.getInCalTyp())
		oCamExtMatx = m_oCamExtMatxUpd;

	while (true)
	{
		// optimize camera parameters
		if ((m_oCfg.getCamMovFlg()) && (m_oCfg.getProcStFrmCnt() < m_nFrmCnt))
			fCostMean = optEdaCamParamHumPs(oHumPs, oCamExtMatx, iIter, false);

		// optimize human pose
		fCostMean = optEdaCamParamHumPs(oHumPs, oCamExtMatx, iIter, true);

		// check if iteration needs to stop
		if ((m_oCfg.getProcStFrmCnt() == m_nFrmCnt) || (nPsEst3dMaxIterNum <= iIter) ||
			((0 < iIter) && ((fCostMeanPrev * fStpCritCostRat) > (fCostMeanPrev - fCostMean))))
			break;

		fCostMeanPrev = fCostMean;

		iIter++;
	}

	// update camera parameters and human pose
	m_oCamExtMatxUpd = oCamExtMatx;
	m_voHumPsPrev.push_back(oHumPs);
	if ((0 < nPsEst3dTmpConstRefTyp) ? (nPsEst3dTmpConstRefTyp < m_voHumPsPrev.size()) : (1 < m_voHumPsPrev.size()))
		m_voHumPsPrev.erase(m_voHumPsPrev.begin());
}

double CPsEst3d::optEdaCamParamHumPs(CHumPs& oHumPs, cv::Matx34d& oCamExtMatx, int iIter, bool bOptHumPsFlg)
{
	// EMNA-global
	int nInitPop, nSelPop, nPsEst3dMaxGenNum, iGen = 0, iProc;
	float fStpCritCostRat, fStpCritCostVal;
	double fCost, fCostMean, fCostMeanPrev;
	//double fCostStd;
	bool bProc25, bProc50, bProc75; 
	std::vector<int> vnInitPop = m_oCfg.getPsEst3dInitPop(), vnSelPop = m_oCfg.getPsEst3dSelPop(), vnPsEst3dMaxGenNum = m_oCfg.getPsEst3dMaxGenNum();
	std::vector<float> vfStpCritCostRat = m_oCfg.getPsEst3dStpCritCostRat(), vfStpCritCostVal = m_oCfg.getPsEst3dStpCritCostVal();
	std::vector<cv::Point3f> voJnt3dPt;
	std::vector<cv::Point2f> voJnt2dPt;
	std::vector<COptParam> voOptParam;
	COptParam::SParamRng sParamRng;

	// selected population should be less than initial population
	if (!bOptHumPsFlg)
	{
		nInitPop = vnInitPop[0];
		nSelPop = vnSelPop[0];
		fStpCritCostRat = vfStpCritCostRat[0];
		fStpCritCostVal = vfStpCritCostVal[0];
		nPsEst3dMaxGenNum = vnPsEst3dMaxGenNum[1];
	}
	else
	{
		nInitPop = vnInitPop[1];
		nSelPop = vnSelPop[1];
		fStpCritCostRat = vfStpCritCostRat[1];
		fStpCritCostVal = vfStpCritCostVal[1];
		if (m_oCfg.getProcStFrmCnt() == m_nFrmCnt)
			nPsEst3dMaxGenNum = vnPsEst3dMaxGenNum[0];
		else
			nPsEst3dMaxGenNum = vnPsEst3dMaxGenNum[2];
	}

	CV_Assert(nSelPop < nInitPop);

	// initialize the ranges of parameters
	sParamRng = initEdaParamRng(oHumPs, oCamExtMatx, bOptHumPsFlg);

	// generate the initial population
	for (int i = 0; i < nInitPop; i++)
		voOptParam.push_back(genOptParamRand(sParamRng));

	// store the 3D joint points and 2D joint points to compute reprojection error in camera parameters optimization
	if (!bOptHumPsFlg)
		str3d2dJntPt(oHumPs, voJnt3dPt, voJnt2dPt);

	if (!bOptHumPsFlg)
		std::printf("Start EDA optimization for camera parameters...\n");
	else
		std::printf("Start EDA optimization for 3D human pose...\n");

	while (true)
	{
		if (!bOptHumPsFlg)
			printf("==== frame #%d - iter. (cam. param.) #%d - generation #%d: ====\n", m_nFrmCnt, iIter, iGen);
		else
			printf("==== frame #%d - iter. (hum. pose) #%d - generation #%d: ====\n", m_nFrmCnt, iIter, iGen);
		iProc = 0;
		bProc25 = false;
		bProc50 = false;
		bProc75 = false;
		fCostMean = 0.0;
		//fCostStd = 0.0;

		// apply the cost function for each member of the initial population and calculate the mean value 
		for (int i = 0; i < nInitPop; i++)
		{
			if (!bOptHumPsFlg)
				fCost = calcCamParamOptCost(voOptParam[i], voJnt3dPt, voJnt2dPt);
			else
				fCost = calcHumPsOptCost(voOptParam[i], oHumPs, oCamExtMatx);
			voOptParam[i].setCost(fCost);
			fCostMean += fCost;
			iProc++;

			if ((((float)iProc / (float)nInitPop) > 0.25) && (!bProc25)) { std::printf("25%%..."); bProc25 = true; }
			if ((((float)iProc / (float)nInitPop) > 0.50) && (!bProc50)) { std::printf("50%%..."); bProc50 = true; }
			if ((((float)iProc / (float)nInitPop) > 0.75) && (!bProc75)) { std::printf("75%%..."); bProc75 = true; }
		}

		fCostMean /= nInitPop;

		std::printf("100%%!\n");
		std::printf("current mean = %.7f\n", fCostMean);

		//// calculate the standard deviation
		//for (int i = 0; i < nInitPop; i++)
		//{
		//	fCost = voOptParam[i].getCost();
		//	fCostStd += (fCost - fCostMean) * (fCost - fCostMean);
		//}
		//fCostStd = std::sqrt(fCostStd / nInitPop);
		//std::printf("current standard deviation = %f\n", fCostStd);

		// sort the list of optimization parameters to get the selected population
		std::stable_sort(voOptParam.begin(), voOptParam.end(), compCost);
		voOptParam.erase(voOptParam.begin() + nSelPop, voOptParam.end());

		// calcualte the new range for EDA optimization
		sParamRng = calcEdaParamRng(voOptParam);

		// check if generation needs to stop
		if ((nPsEst3dMaxGenNum <= iGen) || 
			((0 < iGen) && ((fCostMeanPrev * fStpCritCostRat) > (fCostMeanPrev - fCostMean))) ||
			(fStpCritCostVal > fCostMean))
			break;

		fCostMeanPrev = fCostMean;

		// generate new initial population
		std::vector<COptParam>().swap(voOptParam);
		for (int i = 0; i < nInitPop; i++)
			voOptParam.push_back(genOptParamRand(sParamRng));

		iGen++;
	}

	// update 3D human pose and camera extrinsic parameters
	updHumPsCamExtMat(oHumPs, oCamExtMatx, sParamRng, bOptHumPsFlg);

	std::printf("\n");

	return fCostMean;
}

COptParam::SParamRng CPsEst3d::initEdaParamRng(CHumPs oHumPs, cv::Matx34d oCamExtMatx, bool bOptHumPsFlg)
{
	//CJntPt oJntPtPrev;
	int iJntPtDistMin = -1, nPsEst3dTmpConstRefTyp = m_oCfg.getPsEst3dTmpConstRefTyp();
	float fPsEst2dScrThld = m_oCfg.getPsEst2dScrThld(), fPsEst3dRotRng = m_oCfg.getPsEst3dRotRng(), fPsEst3dTntRng = m_oCfg.getPsEst3dTntRng(),
		fPsEst3dJnt3dMovRng = m_oCfg.getPsEst3dJnt3dMovRng(), fJntPtDepMean, fJntPtDepMax, fJntPtDepMin, fJntPtDistMin = FLT_MAX, fJntPtDist;
	cv::Point3f oJnt3dPt;
	cv::Mat oCamIntMat = m_oCfg.getCamIntMat();
	CJntPt oJntPtCurr, oJntPtPrev;
	COptParam::SParamRng sParamRng;
	std::vector<float> vfPsEst3dJnt3dInitRng = m_oCfg.getPsEst3dJnt3dInitRng();

	// convert camera extrinsic matrix to rotation vector and translation vector
	cv::Mat oCamRotVec, oCamTntMat;
	cvtExtMatx2RotVecTntMat(oCamExtMatx, oCamRotVec, oCamTntMat);

	// at the iteration of optimization of camera parameters
	if (!bOptHumPsFlg)
	{
		if (m_oCfg.getProcStFrmCnt() < m_nFrmCnt)
		{
			for (int i = 0; i < 3; i++)
			{
				sParamRng.ovRotVecMean[i] = oCamRotVec.at<double>(i);
				sParamRng.ovRotVecMax[i] = oCamRotVec.at<double>(i) + fPsEst3dRotRng;
				sParamRng.ovRotVecMin[i] = oCamRotVec.at<double>(i) - fPsEst3dRotRng;
				sParamRng.ovTntMatMean[i] = oCamTntMat.at<double>(i);
				sParamRng.ovTntMatMax[i] = oCamTntMat.at<double>(i) + fPsEst3dTntRng;
				sParamRng.ovTntMatMin[i] = oCamTntMat.at<double>(i) - fPsEst3dTntRng;
			}
		}
		// at the first frame
		// no need for optimization of camera parameters
		else
		{
			for (int i = 0; i < 3; i++)
			{
				sParamRng.ovRotVecMean[i] = oCamRotVec.at<double>(i);
				sParamRng.ovRotVecMax[i] = oCamRotVec.at<double>(i);
				sParamRng.ovRotVecMin[i] = oCamRotVec.at<double>(i);
				sParamRng.ovTntMatMean[i] = oCamTntMat.at<double>(i);
				sParamRng.ovTntMatMax[i] = oCamTntMat.at<double>(i);
				sParamRng.ovTntMatMin[i] = oCamTntMat.at<double>(i);
			}
		}
	}
	// no need for optimization of camera parameters
	else
	{
		for (int i = 0; i < 3; i++)
		{
			sParamRng.ovRotVecMean[i] = oCamRotVec.at<double>(i);
			sParamRng.ovRotVecMax[i] = oCamRotVec.at<double>(i);
			sParamRng.ovRotVecMin[i] = oCamRotVec.at<double>(i);
			sParamRng.ovTntMatMean[i] = oCamTntMat.at<double>(i);
			sParamRng.ovTntMatMax[i] = oCamTntMat.at<double>(i);
			sParamRng.ovTntMatMin[i] = oCamTntMat.at<double>(i);
		}
	}

	// at the iteration of optimization of human pose
	if (bOptHumPsFlg)
	{
		if (m_oCfg.getProcStFrmCnt() < m_nFrmCnt)
		{
			for (int i = 0; i < m_nJntPtNum; i++)
			{
				oJntPtCurr = oHumPs.getJntPt(i);

				if (0 < nPsEst3dTmpConstRefTyp)
					oJntPtPrev = m_voHumPsPrev[0].getJntPt(i);

				// the joint point has been tracked in the previous frame
				if ((fPsEst2dScrThld < oJntPtCurr.getPsEst2dScr()) &&
					(((0 >= nPsEst3dTmpConstRefTyp) && (0 < m_voJnt3dPtAvg[i][3])) ||
					((0 < nPsEst3dTmpConstRefTyp) && (fPsEst2dScrThld < oJntPtPrev.getPsEst2dScr()))))
				{
					if (0 < nPsEst3dTmpConstRefTyp)
						oJnt3dPt = tnt3dPtW2C(oJntPtPrev.get3dPt(), oCamExtMatx);
					else
					{
						oJnt3dPt = cv::Point3f(m_voJnt3dPtAvg[i][0], m_voJnt3dPtAvg[i][1], m_voJnt3dPtAvg[i][2]);
						oJnt3dPt = tnt3dPtW2C(oJnt3dPt, oCamExtMatx);
					}

					fJntPtDepMean = oJnt3dPt.z;
					sParamRng.vfJntPtDepMean.push_back((0.0f < fJntPtDepMean) ? fJntPtDepMean : 0.0f);
					fJntPtDepMax = fJntPtDepMean + fPsEst3dJnt3dMovRng;
					sParamRng.vfJntPtDepMax.push_back((0.0f < fJntPtDepMax) ? fJntPtDepMax : 0.0f);
					fJntPtDepMin = fJntPtDepMean - fPsEst3dJnt3dMovRng;
					sParamRng.vfJntPtDepMin.push_back((0.0f < fJntPtDepMin) ? fJntPtDepMin : 0.0f);
				}
				// it is a newly discovered joint point
				else if (fPsEst2dScrThld < oJntPtCurr.getPsEst2dScr())
				{
					for (int j = 0; j < m_nJntPtNum; j++)
					{
						oJntPtPrev = m_voHumPsPrev[0].getJntPt(j);

						if ((j != i) &&
							(((0 >= nPsEst3dTmpConstRefTyp) && (0 < m_voJnt3dPtAvg[i][3])) ||
							((0 < nPsEst3dTmpConstRefTyp) && (fPsEst2dScrThld < oJntPtPrev.getPsEst2dScr()))))
						{
							if (0 < nPsEst3dTmpConstRefTyp)
								fJntPtDist = cv::norm(oJntPtCurr.get2dPt() - oJntPtPrev.get2dPt());
							else
							{
								oJnt3dPt = cv::Point3f(m_voJnt3dPtAvg[j][0], m_voJnt3dPtAvg[j][1], m_voJnt3dPtAvg[j][2]);
								fJntPtDist = cv::norm(oJntPtCurr.get2dPt() - proj3dPtW22(oJnt3dPt, oCamIntMat, oCamExtMatx));
							}

							if (fJntPtDistMin > fJntPtDist)
							{
								fJntPtDistMin = fJntPtDist;
								iJntPtDistMin = j;
							}
						}
					}

					// found a joint point with closest distance
					if (0 <= iJntPtDistMin)
					{
						if (0 < nPsEst3dTmpConstRefTyp)
							oJnt3dPt = tnt3dPtW2C(m_voHumPsPrev[0].getJntPt(iJntPtDistMin).get3dPt(), oCamExtMatx);
						else
						{
							oJnt3dPt = cv::Point3f(m_voJnt3dPtAvg[iJntPtDistMin][0], m_voJnt3dPtAvg[iJntPtDistMin][1], m_voJnt3dPtAvg[iJntPtDistMin][2]);
							oJnt3dPt = tnt3dPtW2C(oJnt3dPt, oCamExtMatx);
						}

						fJntPtDepMean = oJnt3dPt.z;
						sParamRng.vfJntPtDepMean.push_back((0.0f < fJntPtDepMean) ? fJntPtDepMean : 0.0f);
						fJntPtDepMax = fJntPtDepMean + vfPsEst3dJnt3dInitRng[0];
						sParamRng.vfJntPtDepMax.push_back((0.0f < fJntPtDepMax) ? fJntPtDepMax : 0.0f);
						fJntPtDepMin = fJntPtDepMean - vfPsEst3dJnt3dInitRng[0];
						sParamRng.vfJntPtDepMin.push_back((0.0f < fJntPtDepMin) ? fJntPtDepMin : 0.0f);
					}
					// all other joint points are missing
					else
					{
						sParamRng.vfJntPtDepMean.push_back(vfPsEst3dJnt3dInitRng[1] / 2.0f);
						sParamRng.vfJntPtDepMax.push_back(vfPsEst3dJnt3dInitRng[1]);
						sParamRng.vfJntPtDepMin.push_back(0.0f);
					}
				}
				// the joint point is missing
				else
				{
					sParamRng.vfJntPtDepMean.push_back(0.0f);
					sParamRng.vfJntPtDepMax.push_back(0.0f);
					sParamRng.vfJntPtDepMin.push_back(0.0f);
				}
			}
		}
		// at the first frame
		// wide search for 3D joint points
		else
		{

			for (int i = 0; i < m_nJntPtNum; i++)
			{
				sParamRng.vfJntPtDepMean.push_back((vfPsEst3dJnt3dInitRng[1] + vfPsEst3dJnt3dInitRng[0]) / 2.0f);
				sParamRng.vfJntPtDepMax.push_back(vfPsEst3dJnt3dInitRng[1]);
				sParamRng.vfJntPtDepMin.push_back(vfPsEst3dJnt3dInitRng[0]);
			}
		}
	}
	// no need for optimization of joint points
	else
	{
		for (int i = 0; i < m_nJntPtNum; i++)
		{
			oJntPtCurr = oHumPs.getJntPt(i);

			if (0 < nPsEst3dTmpConstRefTyp)
				oJntPtPrev = m_voHumPsPrev[0].getJntPt(i);

			// the joint point has been tracked in the previous frame
			if ((fPsEst2dScrThld < oJntPtCurr.getPsEst2dScr()) &&
				(((0 >= nPsEst3dTmpConstRefTyp) && (0 < m_voJnt3dPtAvg[i][3])) ||
				((0 < nPsEst3dTmpConstRefTyp) && (fPsEst2dScrThld < oJntPtPrev.getPsEst2dScr()))))
			{
				if (0 < nPsEst3dTmpConstRefTyp)
					oJnt3dPt = tnt3dPtW2C(oJntPtPrev.get3dPt(), oCamExtMatx);
				else
				{
					oJnt3dPt = cv::Point3f(m_voJnt3dPtAvg[i][0], m_voJnt3dPtAvg[i][1], m_voJnt3dPtAvg[i][2]);
					oJnt3dPt = tnt3dPtW2C(oJnt3dPt, oCamExtMatx);
				}

				sParamRng.vfJntPtDepMean.push_back(oJnt3dPt.z);
				sParamRng.vfJntPtDepMax.push_back(oJnt3dPt.z);
				sParamRng.vfJntPtDepMin.push_back(oJnt3dPt.z);
			}
			else
			{
				sParamRng.vfJntPtDepMean.push_back(0.0f);
				sParamRng.vfJntPtDepMax.push_back(0.0f);
				sParamRng.vfJntPtDepMin.push_back(0.0f);
			}
		}
	}

	CV_Assert(m_nJntPtNum == sParamRng.vfJntPtDepMax.size());
	CV_Assert(m_nJntPtNum == sParamRng.vfJntPtDepMin.size());
	CV_Assert(m_nJntPtNum == sParamRng.vfJntPtDepMean.size());

	return sParamRng;
}

COptParam::SParamRng CPsEst3d::calcEdaParamRng(std::vector<COptParam> voOptParam)
{
	int nOptParamNum = voOptParam.size(), nParamNum = 6 + (3 * m_nJntPtNum), iParam, iOptParam = 0;
	float fParamVar = 0.0f, fParamMean, fParamMax, fParamMin;
	float* afParamMean = (float*)calloc(nParamNum, sizeof(float));
	float* afParamData = (float*)calloc((nParamNum * nOptParamNum), sizeof(float));
	std::vector<COptParam>::iterator ivoOptParam;
	COptParam::SParamRng sParamRng;

	// calculate means of parameters
	for (ivoOptParam = voOptParam.begin(); ivoOptParam != voOptParam.end(); ivoOptParam++)
	{
		iParam = 0;

		// rotation parameters
		for (int i = 0; i < 3; i++)
		{
			afParamData[(iParam * nOptParamNum) + iOptParam] = ivoOptParam->getRotParam(i);
			afParamMean[i] += afParamData[(iParam * nOptParamNum) + iOptParam];
			iParam++;
		}

		// translation parameters
		for (int i = 0; i < 3; i++)
		{
			afParamData[(iParam * nOptParamNum) + iOptParam] = ivoOptParam->getTntParam(i);
			afParamMean[i + 3] += afParamData[(iParam * nOptParamNum) + iOptParam];
			iParam++;
		}

		// depths of joint points to the image plane
		for (int i = 0; i < m_nJntPtNum; i++)
		{
			afParamData[(iParam * nOptParamNum) + iOptParam] = ivoOptParam->getJntPtDep(i);
			afParamMean[i + 6] += afParamData[(iParam * nOptParamNum) + iOptParam];
			iParam++;
		}

		iOptParam++;
	}

	for (int i = 0; i < nParamNum; i++)
		afParamMean[i] /= nOptParamNum;

	// compute the standard deviations

	// rotation parameters
	for (int i = 0; i < 3; i++)
	{
		iParam = i;
		fParamVar = 0.0f;
		for (iOptParam = 0; iOptParam < nOptParamNum; iOptParam++)
			fParamVar += (afParamData[(iParam * nOptParamNum) + iOptParam] - afParamMean[iParam]) *
			(afParamData[(iParam * nOptParamNum) + iOptParam] - afParamMean[iParam]);
		fParamVar /= nOptParamNum;
		sParamRng.ovRotVecMean[i] = afParamMean[iParam];
		sParamRng.ovRotVecMax[i] = afParamMean[iParam] + (2 * std::sqrt(fParamVar));
		sParamRng.ovRotVecMin[i] = afParamMean[iParam] - (2 * std::sqrt(fParamVar));
	}

	// translation parameters
	for (int i = 0; i < 3; i++)
	{
		iParam = i + 3;
		fParamVar = 0.0f;
		for (iOptParam = 0; iOptParam < nOptParamNum; iOptParam++)
			fParamVar += (afParamData[(iParam * nOptParamNum) + iOptParam] - afParamMean[iParam]) *
			(afParamData[(iParam * nOptParamNum) + iOptParam] - afParamMean[iParam]);
		fParamVar /= nOptParamNum;
		sParamRng.ovTntMatMean[i] = afParamMean[iParam];
		sParamRng.ovTntMatMax[i] = afParamMean[iParam] + (2 * std::sqrt(fParamVar));
		sParamRng.ovTntMatMin[i] = afParamMean[iParam] - (2 * std::sqrt(fParamVar));
	}

	// depths of joint points to the image plane
	for (int i = 0; i < m_nJntPtNum; i++)
	{
		iParam = i + 6;
		fParamVar = 0.0f;
		for (iOptParam = 0; iOptParam < nOptParamNum; iOptParam++)
			fParamVar += (afParamData[(iParam * nOptParamNum) + iOptParam] - afParamMean[iParam]) *
			(afParamData[(iParam * nOptParamNum) + iOptParam] - afParamMean[iParam]);
		fParamVar /= nOptParamNum;
		fParamMean = afParamMean[iParam];
		sParamRng.vfJntPtDepMean.push_back((0.0f < fParamMean) ? fParamMean : 0.0f);
		fParamMax = fParamMean + (2 * std::sqrt(fParamVar));
		sParamRng.vfJntPtDepMax.push_back((0.0f < fParamMax) ? fParamMax : 0.0f);
		fParamMin = fParamMean - (2 * std::sqrt(fParamVar));
		sParamRng.vfJntPtDepMin.push_back((0.0f < fParamMin) ? fParamMin : 0.0f);
	}

	std::free(afParamMean);
	std::free(afParamData);

	return sParamRng;
}

COptParam CPsEst3d::genOptParamRand(COptParam::SParamRng sParamRng)
{
	COptParam oOptParam;

	for (int i = 0; i < 3; i++)
	{
		oOptParam.setRotParam(i, (float)get_rand_num(sParamRng.ovRotVecMax[i], sParamRng.ovRotVecMin[i], rand()));
		oOptParam.setTntParam(i, (float)get_rand_num(sParamRng.ovTntMatMax[i], sParamRng.ovTntMatMin[i], rand()));
	}

	for (int i = 0; i < m_nJntPtNum; i++)
		oOptParam.addJntPtDep((float)get_rand_num(sParamRng.vfJntPtDepMax[i], sParamRng.vfJntPtDepMin[i], rand()));

	return oOptParam;
}

double CPsEst3d::calcCamParamOptCost(COptParam oOptParam, std::vector<cv::Point3f> voJnt3dPt, std::vector<cv::Point2f> voJnt2dPt)
{
	int nValNum = voJnt3dPt.size();
	double fReprojErr = 0.0;
	cv::Mat oCamIntMat = m_oCfg.getCamIntMat();
	std::vector<cv::Point2f> voJntProj2dPt;

	// compute reprojection error
	if (voJnt3dPt.size())
		cv::projectPoints(voJnt3dPt, oOptParam.getRotVec(), oOptParam.getTntMat(), oCamIntMat, cv::Mat::zeros(4, 1, CV_64F), voJntProj2dPt);

	for (int i = 0; i < nValNum; i++)
		fReprojErr += cv::norm(voJnt2dPt[i] - voJntProj2dPt[i]);

	return fReprojErr /= nValNum;
}

double CPsEst3d::calcHumPsOptCost(COptParam oOptParam, CHumPs oHumPs, cv::Matx34d oCamExtMatx)
{
	int nValNum, nPsMdl = m_oCfg.getPsMdlTyp(), nPsEst3dTmpConstRefTyp = m_oCfg.getPsEst3dTmpConstRefTyp(),
		nBdyPlnFrntDir = 0; // 1: positive normal vector is the frontal direction; -1: negative normal vector is the frontal direction; 0: undetermined
	float fJntPr3dLen, fJntPr3dLenErrRatThld = m_oCfg.getJntPr3dLenErrRatThld(), fPsEst2dScrThld = m_oCfg.getPsEst2dScrThld(),
		fPsEst3dDist2BdyPlnThld = m_oCfg.getPsEst3dDist2BdyPlnThld();
	double fSptErr = 0.0, fTmpPsErr = 0.0, fCamBdyPlnVec, fCosVec, fDist2BdyPln, fDist2BdyPlnMean = 0.0, fDist2BdyPlnStd = 0.0, fDist2BdyPlnErr = 0.0,
		fAngErr = 0.0, fSptErrTemp;
	CJntPt oJntPt0, oJntPt1;
	cv::Vec3d ovBdyPlnNormVec;
	cv::Point3d oBdyPlnCent;
	cv::Mat oCamIntMat = m_oCfg.getCamIntMat(), oValJnt3dPtMat, oCamTntVec;
	cv::PCA oPcaJnt3dPt;
	std::vector<float> vfJntPr3dLen = m_oCfg.getJntPr3dLen();
	std::vector<double> vfJnt3dPtDist2BdyPln;
	std::vector<cv::Point3f> voJnt3dPt(m_nJntPtNum), voValJnt3dPt;

	for (int i = 0; i < m_nJntPtNum; i++)
	{
		oJntPt0 = oHumPs.getJntPt(i);
		voJnt3dPt[i] = bkproj2dPt22W(oJntPt0.get2dPt(), oOptParam.getJntPtDep(i), oCamIntMat, oCamExtMatx);
	}

	// spatial constancy
	nValNum = 0;

	for (int i = 0; i < (m_nJntPtNum - 1); i++)
	{
		if (0 == m_oCfg.getPsMdlTyp() || 2 == m_oCfg.getPsMdlTyp())
		{
			oJntPt0 = oHumPs.getJntPt(PS_COCO_PRS[i*2]);
			oJntPt1 = oHumPs.getJntPt(PS_COCO_PRS[(i*2)+1]);
		}
		else if (1 == m_oCfg.getPsMdlTyp())
		{
			oJntPt0 = oHumPs.getJntPt(PS_MPI_PRS[i*2]);
			oJntPt1 = oHumPs.getJntPt(PS_MPI_PRS[(i*2)+1]);
		}
		else if (3 == m_oCfg.getPsMdlTyp())
		{
			oJntPt0 = oHumPs.getJntPt(PS_INFANT_PRS[i * 2]);
			oJntPt1 = oHumPs.getJntPt(PS_INFANT_PRS[(i * 2) + 1]);
		}

		if ((fPsEst2dScrThld < oJntPt0.getPsEst2dScr()) && (fPsEst2dScrThld < oJntPt1.getPsEst2dScr()))
		{
			if (0 == m_oCfg.getPsMdlTyp() || 2 == m_oCfg.getPsMdlTyp())
				fJntPr3dLen = cv::norm(voJnt3dPt[PS_COCO_PRS[i*2]] - voJnt3dPt[PS_COCO_PRS[(i*2)+1]]);
			else if (1 == m_oCfg.getPsMdlTyp())
				fJntPr3dLen = cv::norm(voJnt3dPt[PS_MPI_PRS[i*2]] - voJnt3dPt[PS_MPI_PRS[(i*2)+1]]);
			else if (3 == m_oCfg.getPsMdlTyp())
				fJntPr3dLen = cv::norm(voJnt3dPt[PS_INFANT_PRS[i * 2]] - voJnt3dPt[PS_INFANT_PRS[(i * 2) + 1]]);

			// fuzzy error to relax the restriction of human pose prior
			fSptErrTemp = std::abs(fJntPr3dLen - vfJntPr3dLen[i]) / vfJntPr3dLen[i];
			fSptErr += (fJntPr3dLenErrRatThld < fSptErrTemp) ? (fSptErrTemp - fJntPr3dLenErrRatThld) : 0.0;

			nValNum++;
		}
	}
	
	if (0 < nValNum)
		fSptErr /= nValNum;

	// temporal constancy
	if (m_oCfg.getProcStFrmCnt() < m_nFrmCnt)
	{
		nValNum = 0;

		for (int i = 0; i < m_nJntPtNum; i++)
		{
			oJntPt0 = oHumPs.getJntPt(i);

			if (0 < nPsEst3dTmpConstRefTyp)
				oJntPt1 = m_voHumPsPrev[0].getJntPt(i);
			
			//	fTmpPsErr += cv::norm(voJnt3dPt[i] - oJntPt1.get3dPt());
			if ((fPsEst2dScrThld < oJntPt0.getPsEst2dScr()) &&
				(((0 >= nPsEst3dTmpConstRefTyp) && (0 < m_voJnt3dPtAvg[i][3])) ||
				((0 < nPsEst3dTmpConstRefTyp) && (fPsEst2dScrThld < oJntPt1.getPsEst2dScr()))))
			{
				if (0 < nPsEst3dTmpConstRefTyp)
					fTmpPsErr += cv::norm(voJnt3dPt[i] - oJntPt1.get3dPt());
				else
					fTmpPsErr += cv::norm(voJnt3dPt[i] - cv::Point3f(m_voJnt3dPtAvg[i][0], m_voJnt3dPtAvg[i][1], m_voJnt3dPtAvg[i][2]));

				nValNum++;
			}
		}

		if (0 < nValNum)
		{
			fTmpPsErr /= nValNum;
			fTmpPsErr *= m_oCfg.getPsEst3dTmpConstSclFac() / m_oCfg.getPsEst3dJnt3dMovMaxDist();
		}
	}

	// 3D plane fitting of joint points
	nValNum = 0;

	for (int i = 0; i < m_nJntPtNum; i++)
	{
		if (fPsEst2dScrThld < oHumPs.getJntPt(i).getPsEst2dScr())
			nValNum++;
	}

	// construct a buffer used by the PCA analysis
	oValJnt3dPtMat = cv::Mat(nValNum, 3, CV_64FC1);
	nValNum = 0;

	for (int i = 0; i < m_nJntPtNum; i++)
	{
		if (fPsEst2dScrThld < oHumPs.getJntPt(i).getPsEst2dScr())
		{
			oValJnt3dPtMat.at<double>(nValNum, 0) = voJnt3dPt[i].x;
			oValJnt3dPtMat.at<double>(nValNum, 1) = voJnt3dPt[i].y;
			oValJnt3dPtMat.at<double>(nValNum, 2) = voJnt3dPt[i].z;
			voValJnt3dPt.push_back(voJnt3dPt[i]);
			nValNum++;

			// assume the camera is always looking towards the frontal human body
			//// face is detected in COCO standard
			//if (0 == m_oCfg.getPsMdlTyp())
			//{
			//	// nose and/or eyes are detected
			//	if ((0 == i) || (14 == i) || (15 == i))
			//		nBdyPlnFrntDir = 1;
			//}
		}
	}

	// perform PCA analysis
	if (nValNum > 0)
		oPcaJnt3dPt = cv::PCA(oValJnt3dPtMat, cv::Mat(), cv::PCA::Flags::DATA_AS_ROW);

	if (2 < oPcaJnt3dPt.eigenvectors.rows)
	{
		// store the center of the 3D body plane
		oBdyPlnCent = cv::Point3d(oPcaJnt3dPt.mean.at<double>(0, 0),
			oPcaJnt3dPt.mean.at<double>(0, 1), oPcaJnt3dPt.mean.at<double>(0, 2));

		// store the third eigenvector (normal vector to the 3D body plane)
		ovBdyPlnNormVec = cv::Vec3d(oPcaJnt3dPt.eigenvectors.at<double>(2, 0),
			oPcaJnt3dPt.eigenvectors.at<double>(2, 1), oPcaJnt3dPt.eigenvectors.at<double>(2, 2));

		// determine the frontal direction of the fitted 3D body plane
		//if (1 == nBdyPlnFrntDir)
		//{
		oCamTntVec = getTntVecInExtMatx(oCamExtMatx);
		fCamBdyPlnVec = (ovBdyPlnNormVec[0] * (oCamTntVec.at<double>(0) - oBdyPlnCent.x)) +
			(ovBdyPlnNormVec[1] * (oCamTntVec.at<double>(1) - oBdyPlnCent.y)) +
			(ovBdyPlnNormVec[2] * (oCamTntVec.at<double>(2) - oBdyPlnCent.z));

		if (0.0 < fCamBdyPlnVec)
			nBdyPlnFrntDir = 1;
		else if (0.0 > fCamBdyPlnVec)
		{
			nBdyPlnFrntDir = -1;
			ovBdyPlnNormVec *= nBdyPlnFrntDir;
		}
		//}

		// calculate the mean of distances to the 3D body plane
		for (int i = 0; i < nValNum; i++)
		{
			fDist2BdyPln = std::abs((ovBdyPlnNormVec[0] * (voValJnt3dPt[i].x - oBdyPlnCent.x)) +
				(ovBdyPlnNormVec[1] * (voValJnt3dPt[i].y - oBdyPlnCent.y)) +
				(ovBdyPlnNormVec[2] * (voValJnt3dPt[i].z - oBdyPlnCent.z)));
			vfJnt3dPtDist2BdyPln.push_back(fDist2BdyPln);
			fDist2BdyPlnMean += fDist2BdyPln;
		}

		if (0 < nValNum)
			fDist2BdyPlnMean /= nValNum;

		// calculate the standard deviation of distances to the 3D body plane
		for (int i = 0; i < nValNum; i++)
			fDist2BdyPlnStd += (vfJnt3dPtDist2BdyPln[i] - fDist2BdyPlnMean) * (vfJnt3dPtDist2BdyPln[i] - fDist2BdyPlnMean);

		if (0 < nValNum)
		{
			fDist2BdyPlnStd = std::sqrt(fDist2BdyPlnStd / nValNum);
			fDist2BdyPlnErr = ((fPsEst3dDist2BdyPlnThld > fDist2BdyPlnStd) ? 0.0 : ((fDist2BdyPlnStd - fPsEst3dDist2BdyPlnThld) / fPsEst3dDist2BdyPlnThld));
		}

		// angle constraints
		nValNum = 0;

		// angles between neck and shoulder
		calcAngErr(cv::Vec3i(0, 1, 2), oHumPs, voJnt3dPt, COS_VEC_RNG_NK_SHLDR, fAngErr, nValNum);
		calcAngErr(cv::Vec3i(0, 1, 5), oHumPs, voJnt3dPt, COS_VEC_RNG_NK_SHLDR, fAngErr, nValNum);

		// angles between shoulder and hip
		calcAngErr(cv::Vec3i(2, 1, 8), oHumPs, voJnt3dPt, COS_VEC_RNG_SHLDR_HP, fAngErr, nValNum);
		calcAngErr(cv::Vec3i(5, 1, 11), oHumPs, voJnt3dPt, COS_VEC_RNG_SHLDR_HP, fAngErr, nValNum);

		// know the frontal direction of 3D body plane
		if (0 != nBdyPlnFrntDir)
		{
			// angles between limbs and the 3D body plane
			calcAngErr(cv::Vec2i(4, 3), oHumPs, voJnt3dPt, ovBdyPlnNormVec, fAngErr, nValNum);
			calcAngErr(cv::Vec2i(7, 6), oHumPs, voJnt3dPt, ovBdyPlnNormVec, fAngErr, nValNum);
			calcAngErr(cv::Vec2i(9, 10), oHumPs, voJnt3dPt, ovBdyPlnNormVec, fAngErr, nValNum);
			calcAngErr(cv::Vec2i(12, 13), oHumPs, voJnt3dPt, ovBdyPlnNormVec, fAngErr, nValNum);
		}

		if (0 < nValNum)
			fAngErr /= nValNum;
	}

	return (fSptErr + 1.0) * ((fTmpPsErr * m_oCfg.getPsEst3dTmpConstRegParam()) + 1.0) *
		((fDist2BdyPlnErr * m_oCfg.getPsEst3dDist2BdyPlnRegParam()) + 1.0) * 
		((fAngErr * m_oCfg.getPsEst3dAngCstrRegParam()) + 1.0);
}

void CPsEst3d::str3d2dJntPt(CHumPs oHumPs, std::vector<cv::Point3f>& voJnt3dPt, std::vector<cv::Point2f>& voJnt2dPt)
{
	CJntPt oJntPt0, oJntPt1;
	int nPsEst3dTmpConstRefTyp = m_oCfg.getPsEst3dTmpConstRefTyp();
	float fPsEst2dScrThld = m_oCfg.getPsEst2dScrThld();

	for (int i = 0; i < m_nJntPtNum; i++)
	{
		oJntPt0 = oHumPs.getJntPt(i);

		if (0 < nPsEst3dTmpConstRefTyp)
			oJntPt1 = m_voHumPsPrev[0].getJntPt(i);

		if ((fPsEst2dScrThld < oJntPt0.getPsEst2dScr()) &&
			(((0 >= nPsEst3dTmpConstRefTyp) && (0 < m_voJnt3dPtAvg[i][3])) ||
			((0 < nPsEst3dTmpConstRefTyp) && (fPsEst2dScrThld < oJntPt1.getPsEst2dScr()))))
		{
			voJnt2dPt.push_back(oJntPt0.get2dPt());

			if (0 < nPsEst3dTmpConstRefTyp)
				voJnt3dPt.push_back(oJntPt1.get3dPt());
			else
				voJnt3dPt.push_back(cv::Point3f(m_voJnt3dPtAvg[i][0], m_voJnt3dPtAvg[i][1], m_voJnt3dPtAvg[i][2]));
		}
	}
}

void CPsEst3d::calcAngErr(cv::Vec3i ovIdx, CHumPs oHumPs, std::vector<cv::Point3f> voJnt3dPt, cv::Vec2d ovCosVecRng, double& fAngErr, int& nValNum)
{
	CV_Assert(ovCosVecRng[1] >= ovCosVecRng[0]);

	cv::Point3f oJntPt10Vec, oJntPt12Vec;
	float fPsEst2dScrThld = m_oCfg.getPsEst2dScrThld();
	double fCosVec;

	if ((fPsEst2dScrThld < oHumPs.getJntPt(ovIdx[0]).getPsEst2dScr()) && 
		(fPsEst2dScrThld < oHumPs.getJntPt(ovIdx[1]).getPsEst2dScr()) && 
		(fPsEst2dScrThld < oHumPs.getJntPt(ovIdx[2]).getPsEst2dScr()))
	{
		oJntPt10Vec = voJnt3dPt[ovIdx[0]] - voJnt3dPt[ovIdx[1]];
		oJntPt12Vec = voJnt3dPt[ovIdx[2]] - voJnt3dPt[ovIdx[1]];
		fCosVec = calcCosVec(oJntPt10Vec, oJntPt12Vec);
		nValNum++;

		if ((ovCosVecRng[0] > std::abs(fCosVec)) || (ovCosVecRng[1] < std::abs(fCosVec)))
			fAngErr += 1;
	}
}

void CPsEst3d::calcAngErr(cv::Vec2i ovIdx, CHumPs oHumPs, std::vector<cv::Point3f> voJnt3dPt, cv::Vec3d ovJntPtPlnNormVec, double& fAngErr, int& nValNum)
{
	float fCosVec, fPsEst2dScrThld = m_oCfg.getPsEst2dScrThld();
	cv::Point3f oJnt3dPt0, oJnt3dPt1;

	if ((fPsEst2dScrThld < oHumPs.getJntPt(ovIdx[0]).getPsEst2dScr()) &&
		(fPsEst2dScrThld < oHumPs.getJntPt(ovIdx[1]).getPsEst2dScr()))
	{
		oJnt3dPt0 = voJnt3dPt[ovIdx[0]];
		oJnt3dPt1 = voJnt3dPt[ovIdx[1]];
		fCosVec = calcCosVec(ovJntPtPlnNormVec[0], (oJnt3dPt0 - oJnt3dPt1));
		nValNum++;

		if (COS_VEC_THLD_BDY_PLN_JNT_PR > fCosVec)
			fAngErr += 1;
	}
}

void CPsEst3d::updHumPsCamExtMat(CHumPs& oHumPs, cv::Matx34d& oCamExtMatx, COptParam::SParamRng sParamRng, bool bOptHumPsFlg)
{
	int nPsEst3dTmpConstRefTyp = m_oCfg.getPsEst3dTmpConstRefTyp();
	float fPsEst2dScrThld = m_oCfg.getPsEst2dScrThld();
	cv::Point3f oJnt3dPt;
	cv::Mat oCamIntMat = m_oCfg.getCamIntMat();
	std::vector<CJntPt> voJntPt;

	// update camera extrinsic parameters
	if (!bOptHumPsFlg)
		cvtRotVecTntMat2ExtMatx(cv::Mat(sParamRng.ovRotVecMean), cv::Mat(sParamRng.ovTntMatMean), oCamExtMatx);
	// update 3D joint points
	else
	{
		voJntPt = oHumPs.getJntPtLs();

		for (int i = 0; i < m_nJntPtNum; i++)
		{
			if (fPsEst2dScrThld < voJntPt[i].getPsEst2dScr())
			{
				oJnt3dPt = bkproj2dPt22W(voJntPt[i].get2dPt(), sParamRng.vfJntPtDepMean[i], oCamIntMat, oCamExtMatx);
				voJntPt[i].set3dPt(oJnt3dPt);

				// running average
				if (0 > nPsEst3dTmpConstRefTyp)
				{
					m_voJnt3dPtAvg[i][0] = ((m_voJnt3dPtAvg[i][0] * m_voJnt3dPtAvg[i][3]) + oJnt3dPt.x) / (m_voJnt3dPtAvg[i][3] + 1);
					m_voJnt3dPtAvg[i][1] = ((m_voJnt3dPtAvg[i][1] * m_voJnt3dPtAvg[i][3]) + oJnt3dPt.y) / (m_voJnt3dPtAvg[i][3] + 1);
					m_voJnt3dPtAvg[i][2] = ((m_voJnt3dPtAvg[i][2] * m_voJnt3dPtAvg[i][3]) + oJnt3dPt.z) / (m_voJnt3dPtAvg[i][3] + 1);
					m_voJnt3dPtAvg[i][3] += 1;
				}
				// initial frame
				else if ((0 == nPsEst3dTmpConstRefTyp) && (m_oCfg.getProcStFrmCnt() == m_nFrmCnt))
				{
					if (m_oCfg.getProcStFrmCnt() == m_nFrmCnt)
					{
						m_voJnt3dPtAvg[i][0] = oJnt3dPt.x;
						m_voJnt3dPtAvg[i][1] = oJnt3dPt.y;
						m_voJnt3dPtAvg[i][2] = oJnt3dPt.z;
						if (fPsEst2dScrThld < voJntPt[i].getPsEst2dScr())
							m_voJnt3dPtAvg[i][3] = 1.0;
						else
							m_voJnt3dPtAvg[i][3] = 0.0;
					}
				}
			}
		}

		oHumPs.setJntPtLs(voJntPt);
	}
}

void CPsEst3d::outTxt(void)
{
	int iLstHumPs = m_voHumPsPrev.size() - 1;
	CJntPt oJntPt;
	cv::Point3f oJnt3dPt;
	cv::Mat oCamRotVec, oCamTntMat, oCamTntVec;
	FILE* pfOutPsEst3dTxt;
	FILE* pfOutCamParamTxt;

	if (m_oCfg.getOutPsEst3dFlg())
	{
		pfOutPsEst3dTxt = std::fopen(m_oCfg.getOutPsEst3dPth(), "a");

		std::fprintf(pfOutPsEst3dTxt, "%d", m_nFrmCnt);

		for (int i = 0; i < m_nJntPtNum; i++)
		{
			oJntPt = m_voHumPsPrev[iLstHumPs].getJntPt(i);
			oJnt3dPt = oJntPt.get3dPt();
			std::fprintf(pfOutPsEst3dTxt, ",%.3f,%.3f,%.3f,%.5f", oJnt3dPt.x, oJnt3dPt.y, oJnt3dPt.z, oJntPt.getPsEst2dScr());
		}

		std::fprintf(pfOutPsEst3dTxt, "\n");

		std::fclose(pfOutPsEst3dTxt);
	}

	if (m_oCfg.getOutCamParamFlg())
	{
		cvtExtMatx2RotVecTntMat(m_oCamExtMatxUpd, oCamRotVec, oCamTntMat);
		cvtTntMat2Vec(oCamRotVec, oCamTntMat, oCamTntVec);

		pfOutCamParamTxt = std::fopen(m_oCfg.getCamParamPth(), "a");
		std::fprintf(pfOutCamParamTxt, "%d,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n",
			m_nFrmCnt, oCamRotVec.at<double>(0), oCamRotVec.at<double>(1), oCamRotVec.at<double>(2),
			oCamTntVec.at<double>(0), oCamTntVec.at<double>(1), oCamTntVec.at<double>(2));
		std::fclose(pfOutCamParamTxt);
	}
}
