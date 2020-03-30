# HPE_IPL

This is an implementation of 3D human pose estimation and action recognition through iterative evolutionary optimization of 3D joint points and camera parameters. It assumes there is a dominant human object in the scene, who is always facing towards the camera. 2D human pose estimation and tracking are included as well. Visual odometry is also included as an optional component.

HPE_IPL is joint work with Zheng (Thomas) Tang and Prof. Jenq-Neng Hwang from the Information Processing Lab at the Department of Electrical Engineering, University of Washington. 

## How It Works

**At every frame, the 2D skeleton of a human object is first derived through a deep convolutional neural network.** We can support both 18-point COCO standard and 15-point MPI standard. The 2D poses are stored as JSON files and read from an input folder. There can be more than one human object detected at each frame, in which the object with the largest area is chosen as the dominant object of interest. **Then the user can choose to apply optical-flow-based tracking to the recovery missing joint points.** A tracked joint point is accepted when the detected joint point has a low estimation score or it is considered far away from the previous location. For a missing joint point to be recovered, it has to be detected in a number of consecutive frames. The user can optionally choose to output a mask of the 2D skeleton, which is input into visual odometry to help remove feature points on the body of the object. 

**Then, the user can choose to perform visual odometry to determine the movement (both rotation and translation) of the camera.** The scaling factor and the camera intrinsic parameters need to be pre-defined in the configuration. The feature points in the previous frame are tracked by optical flow, from which we can derive the essential matrix and get the camera extrinsic matrix. **If the user choose to apply RANSAC in visual odometry, the camera pose is estimated based on a 5-point RANSAC algorithm to minimize reprojection error.** If too few feature points are tracked in the current frame, extra feature points will be detected and added into the list. Currently we only support FAST feature detection. 

**Then, 3D pose estimation is executed with both the 2D joint points and camera pose from visual odometry as input. A two-stage iterative EDA (Estimation of Distribution Algorithm) optimization scheme is proposed for both 3D pose estimation and camera pose refinement.** An introduction of EDA is given [here](http://www.sc.ehu.es/acwbecae/ikerkuntza/these/Ch4.pdf). At the initial frame, since the camera parameters are set, we only optimize the 3D human pose. In other frames, we iteratively optimize the camera pose and the 3D human pose until the changing rate of the cost of human pose estimation is under a threshold or the number of iterations is too large. At the stage of optimization of camera parameters, the objective function is minimization of reprojection error based on a set of given 3D joint points. At the stage of optimization of 3D joint points, each 2D joint point is back projected to 3D at a certain depth based on a given camera extrinsic matrix. The objective function here is minimization of a combination of spatial error (bone length variance compared to a given 3D body prior), temporal error (misalignment of joint points compared to the running average, initial frame or the previous frame), standard deviation of joint points to a 3D body plane fitted by all the joint points, and the angular error of joint pairs. Their contribution to the cost function can be tuned by corresponding regularization parameters. The EMNA_global method in EDA is adopted, in which a multivariate normal distribution is used as the probability density function. The stopping criterion of EDA generation is that the cost value is smaller than a certain threshold, the changing rate of cost value is smaller than another threshold or the number of generations is too large. The range of each parameter for optimization should be pre-defined. **Without visual odometry, the camera pose to be optimized is the previous estimated camera extrinsic matrix.** 

**Finally, used the pre-trained neural network classifier to predict action in every frame (13 actions included).

## Getting Started

These instructions will get the user a copy of the project up and running on a local machine for development and testing purposes. 

### Prerequisites

0. Windows 8  and 10
1. Visual Studio
2. NVIDIA graphics card with at least 1.6 GB available
3. At least 2 GB of free RAM memory
4. CPU with at least 8 cores
5. CUDA and cuDNN
6. Pytorch
7. Caffe, OpenCV, and Caffe prerequisites (included as 3rd-party software components in the package)
8. MATLAB (for 3D visualization only)

The code has been tested on Windows 10 64 bit with Visual Studio 2017 (v141), CUDA 9.1, ,cuDNN 7.1, Pytorch 1.0.1 and MATLAB R2017b. 

### Installing

Build the VC++ solution in Release mode in Visual Studio. All the source code is under `.\src`. The 3rd-party software components (headers, libraries and DLLs) are included in `.\3rdparty`.

When running the 2D/3D joints solution, make sure that you have the configuration file `cfg.json` set properly under the `.\data` folder. All the required input files and folders should be at their corresponding locations set in the configuration file. In the `.\data` folder, an example of input and output is given. 

When retrain the classifier, please follow the desciption of training data format and use `.\src\classifier\main\train.py`, trainig weights will be stored in `.\src\classifier\weights\`.

When running prediction of classifier, please use `.\src\classifier\main\jsonlabel.py` to generate your own label json file `..\MyOwnLabel.json` and your own 2D/3D joints json file `..\MIMMMyOwn.json`, then use `.\src\classifier\main\demo.py` to generate prediction and visualize the results in `.\data\demo\text\%6d.jpg` (you nned to have folder `.\data\demo\` with several images already which you can get from copying all images in `.\data\outImg\` or from two scripts `.\src\plt3d.m` and `.\src\cmbImg.m`). 

To visualize the output 3D joint points and the camera trajectory, an example is given under `.\src\plt3d.m`. 
To comblime original images and the outputs of 3D joint points, an example is given under `.\src\cmbImg.m`.

### Input/Output Format

For input/output camera parameters in text, the format is as follows:
_<frame count>,<rotation0>,<rotation1>,<rotation2>,<translationX>,<translationY>,<translationZ>_
The unit of rotation is neither in radians nor in degrees, but it reflects the change of the rotation vector from Rodrigues transform, whose value is between 0 and 1. The unit of translation is in millimeters.

For the results of 2D pose estimation in JSON, the format is as follows:
_"pose_keypoints": [<x of joint point 0>, <y of joint point 0>, <score of joint point 0>,..., <x of joint point n>, <y of joint point n>, <score of joint point n>],_
For input/output 2D pose estimation in text, the format is as follows:
_<frame count>,<x of joint point 0>,<y of joint point 0>,<score of joint point 0>,...,<x of joint point n>,<y of joint point n>,<score of joint point n>_
The unit of x and y coordinates is in pixels. The estimation score is between 0 and 1. Higher score indicates higher confidence of estimation.

The joint points are listed in order according to different standards. 
There are 18 joint points in COCO standard as follows. 
```
const std::map<unsigned int, std::string> PS_COCO_BD_PTS
{
	{ 0,  "Nose" },
	{ 1,  "Neck" },
	{ 2,  "RShoulder" },
	{ 3,  "RElbow" },
	{ 4,  "RWrist" },
	{ 5,  "LShoulder" },
	{ 6,  "LElbow" },
	{ 7,  "LWrist" },
	{ 8,  "RHip" },
	{ 9,  "RKnee" },
	{ 10, "RAnkle" },
	{ 11, "LHip" },
	{ 12, "LKnee" },
	{ 13, "LAnkle" },
	{ 14, "REye" },
	{ 15, "LEye" },
	{ 16, "REar" },
	{ 17, "LEar" },
	{ 18, "Background" }
};
```

The joint pairs are listed in order as follows.
```
const int PS_COCO_PRS[] = { 1,2,   1,5,   2,3,   3,4,   5,6,   6,7,   1,8,   8,9,   9,10,  1,11,  11,12, 12,13,  1,0,   0,14, 14,16,  0,15, 15,17 };
```

There are 15 joint points in MPI standard as follows. 
```
const std::map<unsigned int, std::string> PS_MPI_BD_PTS
{
	{ 0,  "Head" },
	{ 1,  "Neck" },
	{ 2,  "RShoulder" },
	{ 3,  "RElbow" },
	{ 4,  "RWrist" },
	{ 5,  "LShoulder" },
	{ 6,  "LElbow" },
	{ 7,  "LWrist" },
	{ 8,  "RHip" },
	{ 9,  "RKnee" },
	{ 10, "RAnkle" },
	{ 11, "LHip" },
	{ 12, "LKnee" },
	{ 13, "LAnkle" },
	{ 14, "Chest" },
	{ 15, "Background" }
};
```

The joint pairs are listed in order as follows.
```
const int PS_COCO_PRS[] = { 0,1,   1,2,   2,3,   3,4,   1,5,   5,6,   6,7,   1,14,  14,8,  8,9,  9,10,  14,11, 11,12, 12,13 };
```

For output 3D pose estimation in text, the format is as follows:
_<frame count>,<X of joint point 0>,<Y of joint point 0>,<Z of joint point 0>,<score of joint point 0>,...,<X of joint point n>,<Y of joint point n>,<Z of joint point n>,<score of joint point n>_
The unit of 3D coordinates is in millimeters. The estimation score is between 0 and 1. Higher score indicates higher confidence of estimation. All the results are in the world coordinate system. 

To translate a 3D point in world coordinate to a 3D point in camera coordinate, the user can apply the following function.
```
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
```

## Tips on Configuration

The list of configuration parameters and their descriptions can be found in `.\data\cfg.json`. The user is responsible for fine-tuning the parameters to generate ideal performance. Some tips on how to adjust them are summarized as follows.

### General Information

The user first needs to provide the proper input and output paths of files and folders. In `inCamTyp`, s/he can determine whether to run visual odometry or only rely on camera pose optimization. The camera parameters can be input as txt file as well. Similarly, `inPsEst2dTyp` defines the type of input 2D pose estimation. The user can choose to preprocess the video by [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) to generate the results of 2D pose estimation. Either COCO or MPI pose model can be selected at `psMdlTyp`. The MPI standard has not been tested yet. Then, there are a few flags of output options. The rest of the configuration parameters should all be intuitive to understand.

### Camera Parameters

In general, to accelerate the computation, the user can safely rely on the camera pose estimated in 3D pose estimation. To apply visual odometry, the user needs to provide the accurate scaling factor `visOdomSclFac` and other parameters about FAST feature points. Smaller threshold of `visOdomFastThld` will generate more feature points. When the number of feature points is fewer than `visOdomMinFeatNum`, a new set of feature points will be detected to add into the list for tracking. The user can choose to apply RANSAC estimation of extrinsic camera parameters at `visOdomRansacFlg`, which is described [here](https://docs.opencv.org/3.3.1/d9/d0c/group__calib3d.html#ga50620f0e26e02caa2e9adc07b5fbf24e). Features points on the human body can be removed by enabling the mask from 2D pose estimation at `visOdomMsk2dPsFlg`. The camera intrinsic parameters, initial camera extrinsic parameters and distortion coefficients should be provided under all circumstances. The parameters in the example are based on the camera  of Samsung Galaxy S5 (original resolution of 1920x1080). **When the frames are resized, the focal lengths and principal point coordinates should be scaled by the same factor.** 

### 2D Pose Estimation

Some parameters of OpenPose are provided. **To increase processing speed, the user can reduce the size of `opNetRes` (must be multiples of 16), increase `opSclGap` and/or reduce `opSclNum`, but the accuracy will be negatively influenced.** Depending on which GPU(s) that the user wants to use, s/he can tune`opGpuNum` and `opGpuStCnt`. `trkMs2dJntPtFlg` should usually be enabled to remove false positives and filling in missing joint points. There is an estimation score with each detected 2D joint point (between 0 and 1). When the estimation score is lower than `psEst2dScrThld`, the joint point is considered as a false positive. Similarly, when the distance between the estimated joint point and the corresponding joint point in the previous frame is larger than `psEst2dDistThld`, the joint point is also treated as false positive. **The unit is 2D pixel here, so remember to change the threshold when the frames are reized. If the detected 2D point points are very noisy, try to increase score threshold and reduce distance threshold to rely more on tracking, and vice versa.** To recover missing joint points, they have to be detected in a few consecutive frames defined at `psEst2dContTmSecThld`. **If the frame rate is too low, the user may want to increase the time threshold. Please tune the parameters above to avoid all the false crossings of joint pairs.** 

### 3D Pose Estimation

In `jntPr3dLen`, define the length between each pair of joint points in millimeters. The given example shows the joint points derived from the 3D body prior provided by ArchieMD Inc. in COCO standard. Small error in the lengths of joint pairs is allowable, where the error ratio threshold is controlled by `jntPr3dLenErrRatThld`. The initial populations in EDA are defined in `psEst3dInitPop`. **Reducing the values here will significantly improve the computation speed, but the samples in the populations may not properly reflect the intended distributions, so the performance will be negatively affected.** Similarly the selected population from the initial population with the lowest cost values is defined in `psEst3dSelPop`. The stopping criterion for the changing ratios and the values of the cost function is determined in `psEst3dStpCritCostRat` and `psEst3dStpCritCostVal` respectively. **The larger the values are, the easier the optimization can converge, so the speed can be boosted, but the performance will be worse.** The maximum number of iterations `psEst3dMaxIterNum` is for iterative optimization between two stages. The maximum number of generations `psEst3dMaxGenNum` is for EDA optimization within each stage. The ranges for camera pose estimation are given in `psEst3dRotRng` and `psEst3dTntRng`. **Note that the range of rotation is neither in radians nor in degrees, but it reflects the change of the rotation vector from Rodrigues transform, whose value is between 0 and 1.** Similarly, `psEst3dJnt3dMovRng` and `psEst3dJnt3dInitRng` represent the ranges of depth of each 3D joint point to the camera, where the former is for the regular case and the latter is for the initial case. In the given example, the medic is assumed to be 2 to 4 meters away from the casualty. **When the frame rate is decreased, all the ranges should be enlarged. Larger ranges will cover wider scope to search for the optimal parameters, but make it hard to converge.** For the temporal error term in human pose optimization, there are three types of reference in `psEst3dTmpConstRefTyp`. **When the human is not moving much, the running average or the initial frame should be applied to enforce maximum continuity of joint points. Otherwise, use the human pose at the past serveral frames away as the reference.** The average 3D distance is scaled by a factor of `psEst3dTmpConstSclFac`. For the term of joint point distance to the body plane in human pose optimization, the standard deviation is scaled by a factor of `psEst3dDist2JntPtPlnThld`. The terms in the cost function of human pose estimation are controlled by three regularization parameters. **Usually, when the human is not moving much, `psEst3dTmpConstRegParam` and `psEst3dDist2BdyPlnRegParam` can be set high to restrict the movement of joint points.** 

### Training the classifier

In `train.py` the train() funciton is where you can specify the length of input vector `inp`, origianlly `inp = 56` which means 14 joints with values of `gloabalX, globalY, globalZ, visibility`. Once you changed the input format, you can redesign the input vector size but you will also need to chage the `COCOtoStrange` in `jsonlabel.py` to specify how COCO joint format correspond to the new joints format since the we use the openpose to generate COCO format 18 joints.
In the train() function, you can specify the number of output classes `out`, originally `out = 13` with classes in the following order `0:None, 1:Sitting With Support, 2:Forearm Support (1), 3:Prone Prop, 4:Supported Standing (2), 5:Sitting With Arm Support, 6:Supine Lying (3), 7:Supine Lying (2), 8:Prone Mobility, 9:Forearm Support (2), 10:Supine Lying (1), 11:Pull to Sit, 12:NOS`. If you want to change name or number of the classes, you can specify them in `self.mark_set` in `procMara.py.`
In the train() function, you can also specify the number of neurons in each hidden layer, originally `neuronNum = 256`. The `bsize` stands for batch size, which makes your training process faster, originally `bsize = 32`.
After you train the classifier, your model with weights will be stored under the `.\src\classifier\net\` folder.


### Predict action

In `demo.py`, in demo() function, you need to specify the start frame number `StartFrame` and end frame number `EndFrame`. If you also want to visualize the action prediciton result in images, you can set `text_on_image = True` otherwise, set it to be False. Also, you need to specify the model and weight file `model_path` which is under the `.\src\classifier\net\` folder.

### MATLAB Functions

When plotting the 3D visualization using `.\src\plt3d.m`, set the parameters defined at the beginning. The file and folder paths are all the same as the default settings in `.\data\cfg.json`. In this example, the window size `cfgSmhWinSz` to smooth the camera positions is 1 second. The score threshold of 2D pose estimation `psEstScrThld` should be set to be the same or lower than `psEst2dScrThld` in `.\data\cfg.json`. The viewing perspective `viewAng` is defined [here](https://www.mathworks.com/help/matlab/ref/view.html). `frmCntPlt` controls whether to plot every frame or only a specific frame. The specific standard (COCO/MPI) needs to be switched if necessary. 

## Acknowledgments

This research is supported by research grants from ArchieMD Inc.