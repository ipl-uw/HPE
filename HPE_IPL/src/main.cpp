//-----------------------------------------------------------------------------------
//
//  Copyright (c) 2018 Zheng Tang <zhtang@uw.edu>.  All rights reserved.
//
//  Description:
//      Implementation of 3D human pose estimation with 2D human pose estimation
//      and visual odometry
//
//-----------------------------------------------------------------------------------

#include "Cfg.h"
#include "PsEst3d.h"
//#include "System.h"

int main(int argc, char *argv[])
{
	int nFrmCnt, nProcFrmNum = 0;
	bool bPrevTstpFlg = false;
	float fFrmRt;
	cv::Matx34d oCamExtMatx;
	cv::VideoCapture oVdoCap;
	cv::VideoWriter oVdoWrt;
	cv::Mat oImgFrm, oImgMsk;
	cv::Size oFrmSz;
	CHumPs oHumPs;
	CVisOdom oVisOdom;
	CPsEst2d oPsEst2d;
	CPsEst3d oPsEst3d;
	CCfg oCfg;
	oCfg.ldCfgFl();

	//ORB_SLAM2::System SLAM(".\\HPE_IPL\\ORBvoc.txt", ".\\HPE_IPL\\KITTI03.yaml", ORB_SLAM2::System::MONOCULAR, true);

	// set starting time stamp
	const auto oTmSt = std::chrono::high_resolution_clock::now();

	// set starting frame count
    nFrmCnt = 0;

	cv::namedWindow("current frame", cv::WINDOW_NORMAL);

	// read video source
	// from video file
	if (0 == oCfg.getInVdoTyp())
		oVdoCap = cv::VideoCapture(oCfg.getInVdoPth());
	// from image files
	else if (1 == oCfg.getInVdoTyp())
	{
        char acInImgNm[128] = {};
        std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
        char acInImgPth[128] = {};
        std::sprintf(acInImgPth, oCfg.getFrmFlrPth());
        std::strcat(acInImgPth, acInImgNm);
		oImgFrm = cv::imread(acInImgPth, cv::IMREAD_COLOR);
	}
	// from IP camera
	else if (2 == oCfg.getInVdoTyp())
		oVdoCap.open(oCfg.getInVdoPth());
	// from USB camera
	else if (3 == oCfg.getInVdoTyp())
		oVdoCap = cv::VideoCapture(oCfg.getInCamIdx());
	else
	{
		std::cout << "Error: Unknown camera type" << std::endl;
		return 0;
	}

	// handle error in reading video source
	if ((1 != oCfg.getInVdoTyp()) && (!oVdoCap.isOpened()))
	{
		std::cout << "Error: The video is not captured properly" << std::endl;
		return 0;
	}

	// set frame size
	if (1 == oCfg.getInVdoTyp())
	{
		oFrmSz.width = oImgFrm.cols;
		oFrmSz.height = oImgFrm.rows;
	}
	else
	{
		oFrmSz.width = (int)oVdoCap.get(cv::CAP_PROP_FRAME_WIDTH);
		oFrmSz.height = (int)oVdoCap.get(cv::CAP_PROP_FRAME_HEIGHT);
	}

	// resize frame if necessary
	if ((0 >= oCfg.getRszFrmHei()) || (2 == oCfg.getInPsEst2dTyp()))
		oCfg.setFrmSz(oFrmSz);
	else
	{
		oFrmSz = cv::Size((((float)oFrmSz.width / (float)oFrmSz.height) * oCfg.getRszFrmHei()), oCfg.getRszFrmHei());
		oCfg.setFrmSz(oFrmSz);
	}

	// set video frame rate
	if (0 == oCfg.getInVdoTyp())
		oCfg.setFrmRt(oVdoCap.get(cv::CAP_PROP_FPS));
	else
		oCfg.setFrmRt(oCfg.getOvrdFrmRt());

	// create video writer for output video
	if (oCfg.getOutVdoFlg())
	{
		if (0 == oCfg.getInVdoTyp())
			oVdoWrt = cv::VideoWriter(oCfg.getOutVdoPth(), cv::VideoWriter::fourcc('M', 'P', '4', '2'), (double)oVdoCap.get(cv::CAP_PROP_FPS), oFrmSz);
		else
			oVdoWrt = cv::VideoWriter(oCfg.getOutVdoPth(), cv::VideoWriter::fourcc('M', 'P', '4', '2'), oCfg.getFrmRt(), oFrmSz);
	}

	// create directory for output images
	if (oCfg.getOutImgFlg())
	{
		_mkdir(oCfg.getOutImgFlrPth());	// in Windows
		//mkdir(oCfg.getOutImgFlrPth(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);	// in Linux
	}

	// set starting frame count
    nFrmCnt = 0;

	// initialize 2D pose estimator
	oPsEst2d.initialize(oCfg);

	// only in Release mode
	// preprocess by OpenPose
	if (2 == oCfg.getInPsEst2dTyp())
	{
		fFrmRt = oPsEst2d.preProcOp();

		if ((2 == oCfg.getInVdoTyp()) || (3 == oCfg.getInVdoTyp()))
		{
			oCfg.setFrmRt(fFrmRt);
			oCfg.incProcStFrmCnt();
		}

		oPsEst2d.initialize(oCfg);
	}

	// initialize visual odometry
	if (oCfg.getCamMovFlg() && ((0 == oCfg.getInCalTyp()) || (1 == oCfg.getInCalTyp())))
		oVisOdom.initialize(oCfg);

	// initialize 3D pose estimator
	oPsEst3d.initialize(oCfg);

	// regular human pose estimation
	std::printf("\nStart human pose estimation...\n");

	while (true)
	{
		// for debug
		//if (-1 < nFrmCnt)
		//	cv::waitKey(0);

		// read video frame
		if (0 == oCfg.getInVdoTyp())
			oVdoCap >> oImgFrm;
		else if ((1 == oCfg.getInVdoTyp()) || (2 == oCfg.getInVdoTyp()) || (3 == oCfg.getInVdoTyp()))
        {
            char acInImgNm[128] = {};
			char acInImgPth[128] = {};

			if (1 == oCfg.getInVdoTyp())
				std::sprintf(acInImgNm, "%06d.jpg", nFrmCnt);
			else if ((2 == oCfg.getInVdoTyp()) || (3 == oCfg.getInVdoTyp()))
				std::sprintf(acInImgNm, "%012d_rendered.jpg", nFrmCnt);

            std::sprintf(acInImgPth, oCfg.getFrmFlrPth());
            std::strcat(acInImgPth, acInImgNm);
            oImgFrm = cv::imread(acInImgPth, cv::IMREAD_COLOR);
        }
		
		if ((0 == oCfg.getInPsEst2dTyp()) || (1 == oCfg.getInPsEst2dTyp()))
		{
			// perform camera undistortion on the frame image
			if (oCfg.getCamDistFlg())
				cv::undistort(oImgFrm, oImgFrm, oCfg.getCamIntMat(), oCfg.getDistCoeffMat());

			// resize video frame if necessary
			if (0 < oCfg.getRszFrmHei())
				cv::resize(oImgFrm, oImgFrm, oFrmSz);
		}

		// end the process if conditions are met
		if ((oImgFrm.empty()) || ((0 < oCfg.getProcFrmNum()) && (oCfg.getProcFrmNum() < nProcFrmNum)))
			break;

		// manually end the process by pressing "Esc"
		int nKeyboardIn = cv::waitKey(1);	// read keyboard event
		if (nKeyboardIn == 27)
			break;

		// showing frame count
		std::printf("frame %06d\n", nFrmCnt);

        if (oCfg.getProcStFrmCnt() <= nFrmCnt)
        {
			// run 2D pose estimation
			oHumPs = oPsEst2d.process(oImgFrm, oImgMsk, nFrmCnt);

            // run visual odometry
			if (oCfg.getCamMovFlg() && ((0 == oCfg.getInCalTyp()) || (1 == oCfg.getInCalTyp())))
				oCamExtMatx = oVisOdom.process(oImgFrm, oImgMsk, nFrmCnt);

			// run 3D pose estimation
			oPsEst3d.process(oHumPs, oCamExtMatx, nFrmCnt);

			// update camera extrinsic matrices
			if (oCfg.getCamMovFlg() && (1 == oCfg.getInCalTyp()))
				oVisOdom.updCamExtMat(oCamExtMatx);

			// output 2D pose estimation results
			oPsEst2d.output(oImgFrm, oImgMsk);

            // output visual odometry results
			if (oCfg.getCamMovFlg() && (1 == oCfg.getInCalTyp()))
				oVisOdom.output(oImgFrm);

			// output 3D pose estimation results
			oPsEst3d.output();

            cv::imshow("current frame", oImgFrm);
            cv::waitKey(1);

            // write (plotted) frame to output video
            if (oCfg.getOutVdoFlg())
                oVdoWrt.write(oImgFrm);

            // save output (plotted) frame image
            if (oCfg.getOutImgFlg())
            {
                char acOutImgNm[128] = {};
                std::sprintf(acOutImgNm, "%06d.jpg", nFrmCnt);
                char acOutImgPth[128] = {};
                std::sprintf(acOutImgPth, oCfg.getOutImgFlrPth());
                std::strcat(acOutImgPth, acOutImgNm);
                cv::imwrite(acOutImgPth, oImgFrm);
            }

            nProcFrmNum++;
        }

		nFrmCnt++;
	}
	
	// set ending time stamp
	const auto oTmNd = std::chrono::high_resolution_clock::now();

	// compute total time
	const auto fTtlTmSec = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(oTmNd - oTmSt).count() * 1e-9;
	std::cout << "3D pose estimation successfully finished. Total time: " << std::to_string(fTtlTmSec) << " seconds." << std::endl;

	cv::waitKey(0);
	cv::destroyWindow("current frame");

	return 0;
}
