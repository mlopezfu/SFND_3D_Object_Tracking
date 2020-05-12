# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 
Rubric
## FP.1 Match 3D Objects
Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.
This function is declared in camFusion.hpp as 
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame);
So we need 2 dataframes and a collection of matches, so Yolo and key point and descriptor detection must be performed.

First we perform Yolo detection, then a set of bounding boxes are created and stored in DataFrame structure:
<img src="images/yolo_0.png" width="779" />

After that, a set of keypoints and descriptors are detected in the images.
<img src="images/keypoints.png" width="779" />

And we try to identify each bounding box between images.
<img src="images/bbBoxAndKeyPoints.png" width="779" />

So we make this call
map<int, int> bbBestMatches;
matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1));


## FP.2 Compute Lidar-based TTC
Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.
This function is declared in camFusion.hpp
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC); 
So we need the LIDAR data from the previous frame, the LIDAR data from the actual frame and the time between frames, so we can calculate TTC.
We have a lot of LIDAR DATA, but not all the data will be needed.
<img src="images/LIDAR_ceiling_wo_cropping.png" width="779" />

<img src="images/LIDAR_front_wo_cropping.png" width="779" />
So we perform a cropping of the data, so only a small subset of point in the area of interest are used.

<img src="images/LIDAR_ceiling.png" width="779" />

<img src="images/LIDAR_front.png" width="779" />
After that, we can calculate the distance in the X coordinate.
In the first frame, the X will be d0 and in the last frame X will be d1.
<img src="https://video.udacity-data.com/topher/2019/April/5cbf582d_draggedimage/draggedimage.png" width="779" />
We will take all the X points and divide between the number of points. 
After that, we can solve TTC in that equation.(3)
<img src="https://video.udacity-data.com/topher/2019/April/5cbf5862_draggedimage-1/draggedimage-1.png" width="779" />

## FP.3 Associate Keypoint Correspondences with Bounding Boxes
Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.

Is called in line 390 of FinalProject_Camera.cpp
Implemented from line 135 to line 161 of camFusion_Student.cpp 
## FP.4 Compute Camera-based TTC
Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.
Its an slightly different approach from the code in the course.

Is called in line 391 of FinalProject_Camera.cpp
Implemented from line 166 to line 200 of camFusion_Student.cpp 
## FP.5 Performance Evaluation 1
Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.

## FP.6 Performance Evaluation 2
Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.







## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
