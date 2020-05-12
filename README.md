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
## FP.4 Compute Camera-based TTC
Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.
## FP.5 Performance Evaluation 1
Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.

## FP.6 Performance Evaluation 2
Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.

| Det/Desc | 01-02   | 02-03   | 03-04   | 04-05   | 05-06   | 06-07   | 07-08   | 08-09   | 09-10| 10-11| 11-12| 12-13| 13-14| 14-15| 15-16| 16-17| 17-18| 18-19| 
| :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: | :-------: |
|HARRIS/BRISK|L: 12.516 s</br> C: 15.069 s|L: 12.614 s</br> C: 14.288 s|L: 14.091 s</br> C: 17.019 s|L: 16.689 s</br> C: 15.300 s|L: 15.908 s</br> C: 26.505 s|L: 12.679 s</br> C: 17.862 s|L: 11.984 s</br> C: 22.059 s|L: 13.124 s</br> C: 15.877 s|L: 13.024 s</br> C: 13.791 s|L: 11.175 s</br> C: 15.938 s|L: 12.809 s</br> C: 19.543 s|L: 8.960 s</br> C: 15.341 s|L: 9.964 s</br> C: 12.476 s|L: 9.599 s</br> C: 21.036 s|L: 8.574 s</br> C: 15.905 s|L: 9.516 s</br> C: 15.733 s|L: 9.547 s</br> C: 12.973 s|L: 8.399 s</br> C: 21.403 s|
|HARRIS/BRIEF|L: 12.516 s</br> C: 13.972 s|L: 12.614 s</br> C: 26.975 s|L: 14.091 s</br> C: 18.840 s|L: 16.689 s</br> C: 16.628 s|L: 15.908 s</br> C: 23.092 s|L: 12.679 s</br> C: 22.324 s|L: 11.984 s</br> C: 26.815 s|L: 13.124 s</br> C: 20.493 s|L: 13.024 s</br> C: 16.754 s|L: 11.175 s</br> C: 18.594 s|L: 12.809 s</br> C: 15.489 s|L: 8.960 s</br> C: 14.201 s|L: 9.964 s</br> C: 18.219 s|L: 9.599 s</br> C: 21.822 s|L: 8.574 s</br> C: 22.449 s|L: 9.516 s</br> C: 16.449 s|L: 9.547 s</br> C: 20.433 s|L: 8.399 s</br> C: 45.479 s|
|HARRIS/FREAK|L: 12.516 s</br> C: 18.153 s|L: 12.614 s</br> C: 16.638 s|L: 14.091 s</br> C: 15.216 s|L: 16.689 s</br> C: 15.710 s|L: 15.908 s</br> C: 20.762 s|L: 12.679 s</br> C: 18.666 s|L: 11.984 s</br> C: 36.011 s|L: 13.124 s</br> C: 20.703 s|L: 13.024 s</br> C: 17.795 s|L: 11.175 s</br> C: 13.604 s|L: 12.809 s</br> C: 14.773 s|L: 8.960 s</br> C: 14.153 s|L: 9.964 s</br> C: 11.617 s|L: 9.599 s</br> C: 14.705 s|L: 8.574 s</br> C: 15.618 s|L: 9.516 s</br> C: 17.665 s|L: 9.547 s</br> C: 23.603 s|L: 8.399 s</br> C: 18.003 s|
|HARRIS/SIFT|L: 12.516 s</br> C: 14.835 s|L: 12.614 s</br> C: 26.975 s|L: 14.091 s</br> C: 18.743 s|L: 16.689 s</br> C: 13.865 s|L: 15.908 s</br> C: 22.433 s|L: 12.679 s</br> C: 20.922 s|L: 11.984 s</br> C: 24.083 s|L: 13.124 s</br> C: 21.615 s|L: 13.024 s</br> C: 23.604 s|L: 11.175 s</br> C: 17.171 s|L: 12.809 s</br> C: 15.841 s|L: 8.960 s</br> C: 16.976 s|L: 9.964 s</br> C: 17.143 s|L: 9.599 s</br> C: 19.935 s|L: 8.574 s</br> C: 19.414 s|L: 9.516 s</br> C: 17.747 s|L: 9.547 s</br> C: 15.958 s|L: 8.399 s</br> C: 23.696 s|
|HARRIS/ORB|L: 12.516 s</br> C: 14.569 s|L: 12.614 s</br> C: 23.000 s|L: 14.091 s</br> C: 21.304 s|L: 16.689 s</br> C: 15.775 s|L: 15.908 s</br> C: 29.563 s|L: 12.679 s</br> C: 22.407 s|L: 11.984 s</br> C: 48.488 s|L: 13.124 s</br> C: 28.413 s|L: 13.024 s</br> C: 19.313 s|L: 11.175 s</br> C: 16.082 s|L: 12.809 s</br> C: 15.825 s|L: 8.960 s</br> C: 15.830 s|L: 9.964 s</br> C: 15.323 s|L: 9.599 s</br> C: 27.422 s|L: 8.574 s</br> C: 29.195 s|L: 9.516 s</br> C: 21.503 s|L: 9.547 s</br> C: 22.388 s|L: 8.399 s</br> C: 33.877 s|
|SHITOMASI/BRISK|L: 12.516 s</br> C: 16.467 s|L: 12.614 s</br> C: 17.528 s|L: 14.091 s</br> C: 15.138 s|L: 16.689 s</br> C: 15.175 s|L: 15.908 s</br> C: 15.277 s|L: 12.679 s</br> C: 16.227 s|L: 11.984 s</br> C: 17.301 s|L: 13.124 s</br> C: 19.708 s|L: 13.024 s</br> C: 21.230 s|L: 11.175 s</br> C: 21.485 s|L: 12.809 s</br> C: 24.222 s|L: 8.960 s</br> C: 20.089 s|L: 9.964 s</br> C: 26.891 s|L: 9.599 s</br> C: 23.387 s|L: 8.574 s</br> C: 26.047 s|L: 9.516 s</br> C: 29.343 s|L: 9.547 s</br> C: 22.007 s|L: 8.399 s</br> C: 24.124 s|
|SHITOMASI/BRIEF|L: 12.516 s</br> C: 16.828 s|L: 12.614 s</br> C: 17.455 s|L: 14.091 s</br> C: 14.889 s|L: 16.689 s</br> C: 15.889 s|L: 15.908 s</br> C: 14.826 s|L: 12.679 s</br> C: 16.633 s|L: 11.984 s</br> C: 17.108 s|L: 13.124 s</br> C: 18.488 s|L: 13.024 s</br> C: 18.449 s|L: 11.175 s</br> C: 24.519 s|L: 12.809 s</br> C: 22.789 s|L: 8.960 s</br> C: 20.859 s|L: 9.964 s</br> C: 25.613 s|L: 9.599 s</br> C: 26.363 s|L: 8.574 s</br> C: 28.010 s|L: 9.516 s</br> C: 31.358 s|L: 9.547 s</br> C: 25.321 s|L: 8.399 s</br> C: 27.436 s|
|SHITOMASI/FREAK|L: 12.516 s</br> C: 18.042 s|L: 12.614 s</br> C: 17.770 s|L: 14.091 s</br> C: 15.331 s|L: 16.689 s</br> C: 15.571 s|L: 15.908 s</br> C: 16.047 s|L: 12.679 s</br> C: 17.105 s|L: 11.984 s</br> C: 16.758 s|L: 13.124 s</br> C: 17.014 s|L: 13.024 s</br> C: 17.985 s|L: 11.175 s</br> C: 20.354 s|L: 12.809 s</br> C: 20.856 s|L: 8.960 s</br> C: 21.542 s|L: 9.964 s</br> C: 24.879 s|L: 9.599 s</br> C: 21.005 s|L: 8.574 s</br> C: 29.258 s|L: 9.516 s</br> C: 23.062 s|L: 9.547 s</br> C: 21.875 s|L: 8.399 s</br> C: 25.403 s|
|SHITOMASI/SIFT|L: 12.516 s</br> C: 15.872 s|L: 12.614 s</br> C: 16.792 s|L: 14.091 s</br> C: 16.209 s|L: 16.689 s</br> C: 16.423 s|L: 15.908 s</br> C: 14.817 s|L: 12.679 s</br> C: 16.195 s|L: 11.984 s</br> C: 17.188 s|L: 13.124 s</br> C: 18.977 s|L: 13.024 s</br> C: 19.965 s|L: 11.175 s</br> C: 23.365 s|L: 12.809 s</br> C: 30.360 s|L: 8.960 s</br> C: 24.478 s|L: 9.964 s</br> C: 33.083 s|L: 9.599 s</br> C: 30.611 s|L: 8.574 s</br> C: 35.360 s|L: 9.516 s</br> C: 33.975 s|L: 9.547 s</br> C: 28.842 s|L: 8.399 s</br> C: 31.091 s|
|SHITOMASI/ORB|L: 12.516 s</br> C: 16.461 s|L: 12.614 s</br> C: 17.527 s|L: 14.091 s</br> C: 16.302 s|L: 16.689 s</br> C: 15.726 s|L: 15.908 s</br> C: 15.213 s|L: 12.679 s</br> C: 17.529 s|L: 11.984 s</br> C: 16.968 s|L: 13.124 s</br> C: 17.205 s|L: 13.024 s</br> C: 19.508 s|L: 11.175 s</br> C: 23.936 s|L: 12.809 s</br> C: 24.226 s|L: 8.960 s</br> C: 22.111 s|L: 9.964 s</br> C: 29.365 s|L: 9.599 s</br> C: 28.082 s|L: 8.574 s</br> C: 31.848 s|L: 9.516 s</br> C: 27.313 s|L: 9.547 s</br> C: 25.573 s|L: 8.399 s</br> C: 30.192 s|
|FAST/BRISK|L: 12.516 s</br> C: 17.527 s|L: 12.614 s</br> C: 16.542 s|L: 14.091 s</br> C: 16.923 s|L: 16.689 s</br> C: 17.747 s|L: 15.908 s</br> C: 17.181 s|L: 12.679 s</br> C: 16.885 s|L: 11.984 s</br> C: 17.015 s|L: 13.124 s</br> C: 17.219 s|L: 13.024 s</br> C: 16.970 s|L: 11.175 s</br> C: 17.467 s|L: 12.809 s</br> C: 18.408 s|L: 8.960 s</br> C: 16.977 s|L: 9.964 s</br> C: 16.315 s|L: 9.599 s</br> C: 20.452 s|L: 8.574 s</br> C: 20.475 s|L: 9.516 s</br> C: 20.859 s|L: 9.547 s</br> C: 21.442 s|L: 8.399 s</br> C: 22.845 s|
|FAST/BRIEF|L: 12.516 s</br> C: 17.856 s|L: 12.614 s</br> C: 17.920 s|L: 14.091 s</br> C: 17.520 s|L: 16.689 s</br> C: 16.740 s|L: 15.908 s</br> C: 16.427 s|L: 12.679 s</br> C: 15.664 s|L: 11.984 s</br> C: 16.412 s|L: 13.124 s</br> C: 16.471 s|L: 13.024 s</br> C: 15.291 s|L: 11.175 s</br> C: 17.227 s|L: 12.809 s</br> C: 17.593 s|L: 8.960 s</br> C: 17.223 s|L: 9.964 s</br> C: 17.702 s|L: 9.599 s</br> C: 19.369 s|L: 8.574 s</br> C: 21.392 s|L: 9.516 s</br> C: 18.646 s|L: 9.547 s</br> C: 17.944 s|L: 8.399 s</br> C: 21.722 s|
|FAST/FREAK|L: 12.516 s</br> C: 18.458 s|L: 12.614 s</br> C: 17.598 s|L: 14.091 s</br> C: 17.625 s|L: 16.689 s</br> C: 18.383 s|L: 15.908 s</br> C: 16.922 s|L: 12.679 s</br> C: 16.830 s|L: 11.984 s</br> C: 18.178 s|L: 13.124 s</br> C: 16.909 s|L: 13.024 s</br> C: 16.315 s|L: 11.175 s</br> C: 16.972 s|L: 12.809 s</br> C: 18.331 s|L: 8.960 s</br> C: 16.406 s|L: 9.964 s</br> C: 17.688 s|L: 9.599 s</br> C: 19.014 s|L: 8.574 s</br> C: 18.945 s|L: 9.516 s</br> C: 22.016 s|L: 9.547 s</br> C: 19.288 s|L: 8.399 s</br> C: 22.330 s|
|FAST/SIFT|L: 12.516 s</br> C: 18.774 s|L: 12.614 s</br> C: 17.835 s|L: 14.091 s</br> C: 18.049 s|L: 16.689 s</br> C: 16.797 s|L: 15.908 s</br> C: 18.241 s|L: 12.679 s</br> C: 15.863 s|L: 11.984 s</br> C: 17.817 s|L: 13.124 s</br> C: 17.328 s|L: 13.024 s</br> C: 17.758 s|L: 11.175 s</br> C: 17.703 s|L: 12.809 s</br> C: 20.873 s|L: 8.960 s</br> C: 21.157 s|L: 9.964 s</br> C: 19.863 s|L: 9.599 s</br> C: 19.695 s|L: 8.574 s</br> C: 22.806 s|L: 9.516 s</br> C: 21.456 s|L: 9.547 s</br> C: 20.985 s|L: 8.399 s</br> C: 23.081 s|
|FAST/ORB|L: 12.516 s</br> C: 18.253 s|L: 12.614 s</br> C: 17.341 s|L: 14.091 s</br> C: 17.592 s|L: 16.689 s</br> C: 17.032 s|L: 15.908 s</br> C: 17.634 s|L: 12.679 s</br> C: 16.878 s|L: 11.984 s</br> C: 17.979 s|L: 13.124 s</br> C: 16.357 s|L: 13.024 s</br> C: 17.386 s|L: 11.175 s</br> C: 19.213 s|L: 12.809 s</br> C: 17.941 s|L: 8.960 s</br> C: 18.320 s|L: 9.964 s</br> C: 17.622 s|L: 9.599 s</br> C: 18.498 s|L: 8.574 s</br> C: 20.815 s|L: 9.516 s</br> C: 20.571 s|L: 9.547 s</br> C: 19.378 s|L: 8.399 s</br> C: 22.948 s|
|BRISK/BRISK|L: 12.516 s</br> C: 18.333 s|L: 12.614 s</br> C: 18.744 s|L: 14.091 s</br> C: 18.242 s|L: 16.689 s</br> C: 17.956 s|L: 15.908 s</br> C: 18.716 s|L: 12.679 s</br> C: 16.733 s|L: 11.984 s</br> C: 15.471 s|L: 13.124 s</br> C: 18.046 s|L: 13.024 s</br> C: 16.271 s|L: 11.175 s</br> C: 16.795 s|L: 12.809 s</br> C: 22.339 s|L: 8.960 s</br> C: 17.297 s|L: 9.964 s</br> C: 17.049 s|L: 9.599 s</br> C: 18.787 s|L: 8.574 s</br> C: 19.863 s|L: 9.516 s</br> C: 21.495 s|L: 9.547 s</br> C: 22.534 s|L: 8.399 s</br> C: 24.610 s|
|BRISK/BRIEF|L: 12.516 s</br> C: 17.060 s|L: 12.614 s</br> C: 17.727 s|L: 14.091 s</br> C: 18.193 s|L: 16.689 s</br> C: 15.131 s|L: 15.908 s</br> C: 17.803 s|L: 12.679 s</br> C: 15.563 s|L: 11.984 s</br> C: 15.553 s|L: 13.124 s</br> C: 16.311 s|L: 13.024 s</br> C: 17.344 s|L: 11.175 s</br> C: 18.482 s|L: 12.809 s</br> C: 20.697 s|L: 8.960 s</br> C: 20.300 s|L: 9.964 s</br> C: 18.353 s|L: 9.599 s</br> C: 19.777 s|L: 8.574 s</br> C: 23.625 s|L: 9.516 s</br> C: 20.633 s|L: 9.547 s</br> C: 21.229 s|L: 8.399 s</br> C: 25.588 s|
|BRISK/FREAK|L: 12.516 s</br> C: 19.048 s|L: 12.614 s</br> C: 19.845 s|L: 14.091 s</br> C: 16.744 s|L: 16.689 s</br> C: 16.942 s|L: 15.908 s</br> C: 17.242 s|L: 12.679 s</br> C: 15.571 s|L: 11.984 s</br> C: 17.981 s|L: 13.124 s</br> C: 16.288 s|L: 13.024 s</br> C: 14.165 s|L: 11.175 s</br> C: 16.662 s|L: 12.809 s</br> C: 16.602 s|L: 8.960 s</br> C: 16.329 s|L: 9.964 s</br> C: 14.871 s|L: 9.599 s</br> C: 16.797 s|L: 8.574 s</br> C: 18.690 s|L: 9.516 s</br> C: 21.684 s|L: 9.547 s</br> C: 20.988 s|L: 8.399 s</br> C: 25.010 s|
|BRISK/SIFT|L: 12.516 s</br> C: 19.848 s|L: 12.614 s</br> C: 22.951 s|L: 14.091 s</br> C: 21.024 s|L: 16.689 s</br> C: 19.296 s|L: 15.908 s</br> C: 18.753 s|L: 12.679 s</br> C: 15.073 s|L: 11.984 s</br> C: 14.578 s|L: 13.124 s</br> C: 18.914 s|L: 13.024 s</br> C: 15.671 s|L: 11.175 s</br> C: 15.173 s|L: 12.809 s</br> C: 20.268 s|L: 8.960 s</br> C: 20.706 s|L: 9.964 s</br> C: 17.616 s|L: 9.599 s</br> C: 18.315 s|L: 8.574 s</br> C: 17.957 s|L: 9.516 s</br> C: 19.241 s|L: 9.547 s</br> C: 25.229 s|L: 8.399 s</br> C: 25.399 s|
|BRISK/ORB|L: 12.516 s</br> C: 19.009 s|L: 12.614 s</br> C: 19.639 s|L: 14.091 s</br> C: 19.605 s|L: 16.689 s</br> C: 18.268 s|L: 15.908 s</br> C: 18.834 s|L: 12.679 s</br> C: 17.250 s|L: 11.984 s</br> C: 17.547 s|L: 13.124 s</br> C: 17.821 s|L: 13.024 s</br> C: 15.939 s|L: 11.175 s</br> C: 17.253 s|L: 12.809 s</br> C: 19.952 s|L: 8.960 s</br> C: 21.684 s|L: 9.964 s</br> C: 16.432 s|L: 9.599 s</br> C: 18.784 s|L: 8.574 s</br> C: 21.028 s|L: 9.516 s</br> C: 20.137 s|L: 9.547 s</br> C: 24.767 s|L: 8.399 s</br> C: 28.771 s|
|ORB/BRISK|L: 12.516 s</br> C: 27.353 s|L: 12.614 s</br> C: 36.207 s|L: 14.091 s</br> C: 164.835 s|L: 16.689 s</br> C: -inf s|L: 15.908 s</br> C: -inf s|L: 12.679 s</br> C: 113.334 s|L: 11.984 s</br> C: 96.956 s|L: 13.124 s</br> C: 387.253 s|L: 13.024 s</br> C: 370.395 s|L: 11.175 s</br> C: -inf s|L: 12.809 s</br> C: -inf s|L: 8.960 s</br> C: 31.751 s|L: 9.964 s</br> C: -inf s|L: 9.599 s</br> C: 121.201 s|L: 8.574 s</br> C: -inf s|L: 9.516 s</br> C: -inf s|L: 9.547 s</br> C: -inf s|L: 8.399 s</br> C: -inf s|
|ORB/BRIEF|L: 12.516 s</br> C: 25.995 s|L: 12.614 s</br> C: 23.671 s|L: 14.091 s</br> C: 26.583 s|L: 16.689 s</br> C: 24.340 s|L: 15.908 s</br> C: 35.126 s|L: 12.679 s</br> C: 38.920 s|L: 11.984 s</br> C: 29.979 s|L: 13.124 s</br> C: 78.781 s|L: 13.024 s</br> C: 44.653 s|L: 11.175 s</br> C: 50.866 s|L: 12.809 s</br> C: 583.008 s|L: 8.960 s</br> C: 32.316 s|L: 9.964 s</br> C: 1074.201 s|L: 9.599 s</br> C: 101.179 s|L: 8.574 s</br> C: 43.778 s|L: 9.516 s</br> C: 29.875 s|L: 9.547 s</br> C: 411.720 s|L: 8.399 s</br> C: 86.044 s|
|ORB/FREAK|L: 12.516 s</br> C: 10.521 s|L: 12.614 s</br> C: 13.618 s|L: 14.091 s</br> C: 21.107 s|L: 16.689 s</br> C: 21.569 s|L: 15.908 s</br> C: 23.614 s|L: 12.679 s</br> C: -inf s|L: 11.984 s</br> C: -inf s|L: 13.124 s</br> C: -inf s|L: 13.024 s</br> C: -inf s|L: 11.175 s</br> C: -inf s|L: 12.809 s</br> C: -57.224 s|L: 8.960 s</br> C: 129.952 s|L: 9.964 s</br> C: 264.666 s|L: 9.599 s</br> C: -inf s|L: 8.574 s</br> C: -105.129 s|L: 9.516 s</br> C: -inf s|L: 9.547 s</br> C: -1462.981 s|L: 8.399 s</br> C: -32.333 s|
|ORB/SIFT|L: 12.516 s</br> C: 27.876 s|L: 12.614 s</br> C: 22.116 s|L: 14.091 s</br> C: 34.090 s|L: 16.689 s</br> C: 50.148 s|L: 15.908 s</br> C: 12643.334 s|L: 12.679 s</br> C: 109.084 s|L: 11.984 s</br> C: 54.799 s|L: 13.124 s</br> C: 406.252 s|L: 13.024 s</br> C: 30.889 s|L: 11.175 s</br> C: 104.613 s|L: 12.809 s</br> C: 169.832 s|L: 8.960 s</br> C: 31.675 s|L: 9.964 s</br> C: 84.790 s|L: 9.599 s</br> C: 42.537 s|L: 8.574 s</br> C: 147.313 s|L: 9.516 s</br> C: -inf s|L: 9.547 s</br> C: 1357750.884 s|L: 8.399 s</br> C: -inf s|
|ORB/ORB|L: 12.516 s</br> C: 34.801 s|L: 12.614 s</br> C: 24.853 s|L: 14.091 s</br> C: 48.953 s|L: 16.689 s</br> C: 206.960 s|L: 15.908 s</br> C: -inf s|L: 12.679 s</br> C: -inf s|L: 11.984 s</br> C: 147.809 s|L: 13.124 s</br> C: -inf s|L: 13.024 s</br> C: 72.608 s|L: 11.175 s</br> C: 352.844 s|L: 12.809 s</br> C: 50.965 s|L: 8.960 s</br> C: 33.985 s|L: 9.964 s</br> C: -inf s|L: 9.599 s</br> C: 90.856 s|L: 8.574 s</br> C: 1794.466 s|L: 9.516 s</br> C: 84.339 s|L: 9.547 s</br> C: -inf s|L: 8.399 s</br> C: -inf s|
|AKAZE/BRISK|L: 12.516 s</br> C: 16.142 s|L: 12.614 s</br> C: 14.956 s|L: 14.091 s</br> C: 16.642 s|L: 16.689 s</br> C: 17.957 s|L: 15.908 s</br> C: 16.371 s|L: 12.679 s</br> C: 15.571 s|L: 11.984 s</br> C: 15.376 s|L: 13.124 s</br> C: 14.550 s|L: 13.024 s</br> C: 15.610 s|L: 11.175 s</br> C: 14.497 s|L: 12.809 s</br> C: 17.495 s|L: 8.960 s</br> C: 17.147 s|L: 9.964 s</br> C: 16.869 s|L: 9.599 s</br> C: 15.780 s|L: 8.574 s</br> C: 17.584 s|L: 9.516 s</br> C: 19.158 s|L: 9.547 s</br> C: 16.926 s|L: 8.399 s</br> C: 17.311 s|
|AKAZE/BRIEF|L: 12.516 s</br> C: 17.951 s|L: 12.614 s</br> C: 16.428 s|L: 14.091 s</br> C: 17.680 s|L: 16.689 s</br> C: 17.505 s|L: 15.908 s</br> C: 15.798 s|L: 12.679 s</br> C: 15.168 s|L: 11.984 s</br> C: 15.528 s|L: 13.124 s</br> C: 15.246 s|L: 13.024 s</br> C: 15.156 s|L: 11.175 s</br> C: 13.933 s|L: 12.809 s</br> C: 17.069 s|L: 8.960 s</br> C: 15.962 s|L: 9.964 s</br> C: 15.607 s|L: 9.599 s</br> C: 16.620 s|L: 8.574 s</br> C: 16.685 s|L: 9.516 s</br> C: 17.800 s|L: 9.547 s</br> C: 18.026 s|L: 8.399 s</br> C: 16.913 s|
|AKAZE/FREAK|L: 12.516 s</br> C: 16.776 s|L: 12.614 s</br> C: 15.943 s|L: 14.091 s</br> C: 17.505 s|L: 16.689 s</br> C: 18.420 s|L: 15.908 s</br> C: 16.114 s|L: 12.679 s</br> C: 16.252 s|L: 11.984 s</br> C: 14.908 s|L: 13.124 s</br> C: 15.013 s|L: 13.024 s</br> C: 15.507 s|L: 11.175 s</br> C: 14.687 s|L: 12.809 s</br> C: 16.875 s|L: 8.960 s</br> C: 15.132 s|L: 9.964 s</br> C: 16.393 s|L: 9.599 s</br> C: 15.728 s|L: 8.574 s</br> C: 17.021 s|L: 9.516 s</br> C: 18.818 s|L: 9.547 s</br> C: 17.235 s|L: 8.399 s</br> C: 16.593 s|
|AKAZE/SIFT|L: 12.516 s</br> C: 16.927 s|L: 12.614 s</br> C: 16.706 s|L: 14.091 s</br> C: 18.081 s|L: 16.689 s</br> C: 18.607 s|L: 15.908 s</br> C: 16.004 s|L: 12.679 s</br> C: 16.970 s|L: 11.984 s</br> C: 14.540 s|L: 13.124 s</br> C: 13.722 s|L: 13.024 s</br> C: 14.136 s|L: 11.175 s</br> C: 13.433 s|L: 12.809 s</br> C: 16.052 s|L: 8.960 s</br> C: 14.932 s|L: 9.964 s</br> C: 15.502 s|L: 9.599 s</br> C: 15.259 s|L: 8.574 s</br> C: 16.550 s|L: 9.516 s</br> C: 16.392 s|L: 9.547 s</br> C: 18.749 s|L: 8.399 s</br> C: 16.781 s|
|AKAZE/ORB|L: 12.516 s</br> C: 16.816 s|L: 12.614 s</br> C: 15.571 s|L: 14.091 s</br> C: 17.262 s|L: 16.689 s</br> C: 18.526 s|L: 15.908 s</br> C: 17.473 s|L: 12.679 s</br> C: 17.339 s|L: 11.984 s</br> C: 15.358 s|L: 13.124 s</br> C: 16.399 s|L: 13.024 s</br> C: 16.780 s|L: 11.175 s</br> C: 15.394 s|L: 12.809 s</br> C: 16.907 s|L: 8.960 s</br> C: 15.806 s|L: 9.964 s</br> C: 15.906 s|L: 9.599 s</br> C: 16.030 s|L: 8.574 s</br> C: 16.925 s|L: 9.516 s</br> C: 16.754 s|L: 9.547 s</br> C: 18.204 s|L: 8.399 s</br> C: 16.636 s|
|AKAZE/AKAZE|L: 12.516 s</br> C: 18.427 s|L: 12.614 s</br> C: 17.613 s|L: 14.091 s</br> C: 17.929 s|L: 16.689 s</br> C: 18.597 s|L: 15.908 s</br> C: 16.670 s|L: 12.679 s</br> C: 17.075 s|L: 11.984 s</br> C: 15.021 s|L: 13.124 s</br> C: 13.996 s|L: 13.024 s</br> C: 14.707 s|L: 11.175 s</br> C: 14.367 s|L: 12.809 s</br> C: 15.816 s|L: 8.960 s</br> C: 15.949 s|L: 9.964 s</br> C: 15.554 s|L: 9.599 s</br> C: 16.724 s|L: 8.574 s</br> C: 15.655 s|L: 9.516 s</br> C: 16.263 s|L: 9.547 s</br> C: 16.650 s|L: 8.399 s</br> C: 16.253 s|
|SIFT/BRISK|L: 12.516 s</br> C: 15.145 s|L: 12.614 s</br> C: 14.919 s|L: 14.091 s</br> C: 15.814 s|L: 16.689 s</br> C: 17.424 s|L: 15.908 s</br> C: 17.251 s|L: 12.679 s</br> C: 14.785 s|L: 11.984 s</br> C: 17.358 s|L: 13.124 s</br> C: 19.452 s|L: 13.024 s</br> C: 19.420 s|L: 11.175 s</br> C: 20.478 s|L: 12.809 s</br> C: 23.313 s|L: 8.960 s</br> C: 23.708 s|L: 9.964 s</br> C: 24.076 s|L: 9.599 s</br> C: 28.684 s|L: 8.574 s</br> C: 24.188 s|L: 9.516 s</br> C: 23.223 s|L: 9.547 s</br> C: 30.236 s|L: 8.399 s</br> C: 34.382 s|
|SIFT/BRIEF|L: 12.516 s</br> C: 17.202 s|L: 12.614 s</br> C: 14.687 s|L: 14.091 s</br> C: 17.735 s|L: 16.689 s</br> C: 17.614 s|L: 15.908 s</br> C: 19.700 s|L: 12.679 s</br> C: 15.791 s|L: 11.984 s</br> C: 16.478 s|L: 13.124 s</br> C: 20.917 s|L: 13.024 s</br> C: 21.276 s|L: 11.175 s</br> C: 25.486 s|L: 12.809 s</br> C: 22.927 s|L: 8.960 s</br> C: 24.022 s|L: 9.964 s</br> C: 26.151 s|L: 9.599 s</br> C: 24.188 s|L: 8.574 s</br> C: 27.737 s|L: 9.516 s</br> C: 35.396 s|L: 9.547 s</br> C: 42.534 s|L: 8.399 s</br> C: 35.989 s|
|SIFT/FREAK|L: 12.516 s</br> C: 16.172 s|L: 12.614 s</br> C: 15.939 s|L: 14.091 s</br> C: 16.345 s|L: 16.689 s</br> C: 17.935 s|L: 15.908 s</br> C: 17.646 s|L: 12.679 s</br> C: 14.689 s|L: 11.984 s</br> C: 16.360 s|L: 13.124 s</br> C: 18.272 s|L: 13.024 s</br> C: 20.559 s|L: 11.175 s</br> C: 21.703 s|L: 12.809 s</br> C: 18.738 s|L: 8.960 s</br> C: 28.550 s|L: 9.964 s</br> C: 23.504 s|L: 9.599 s</br> C: 26.098 s|L: 8.574 s</br> C: 20.555 s|L: 9.516 s</br> C: 24.405 s|L: 9.547 s</br> C: 29.519 s|L: 8.399 s</br> C: 31.401 s|
|SIFT/SIFT|L: 12.516 s</br> C: 16.167 s|L: 12.614 s</br> C: 15.199 s|L: 14.091 s</br> C: 15.742 s|L: 16.689 s</br> C: 16.314 s|L: 15.908 s</br> C: 16.823 s|L: 12.679 s</br> C: 16.928 s|L: 11.984 s</br> C: 18.470 s|L: 13.124 s</br> C: 24.610 s|L: 13.024 s</br> C: 24.058 s|L: 11.175 s</br> C: 34.900 s|L: 12.809 s</br> C: 56.897 s|L: 8.960 s</br> C: 39.906 s|L: 9.964 s</br> C: 29.822 s|L: 9.599 s</br> C: 49.011 s|L: 8.574 s</br> C: 31.448 s|L: 9.516 s</br> C: 25.371 s|L: 9.547 s</br> C: 46.536 s|L: 8.399 s</br> C: 40.225 s|







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
