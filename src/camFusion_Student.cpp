
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// Associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    double sum = 0;
    for (cv::DMatch match : kptMatches) {
        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt)) {
            boundingBox.kptMatches.push_back(match);
            cv::KeyPoint kpCurr = kptsCurr.at(match.trainIdx);
            cv::KeyPoint kpPrev = kptsPrev.at(match.queryIdx);
            double dist = cv::norm(kpCurr.pt - kpPrev.pt);
            sum += dist;
        }
    }
    double mean = sum / boundingBox.kptMatches.size();
    cout << "size before erase " << boundingBox.kptMatches.size() << endl;
    constexpr double ratio = 1.25;
    int borradosMas=0,borradosMenos=0;
    if(boundingBox.kptMatches.size()>50)
    {
        for (auto it = boundingBox.kptMatches.begin(); it < boundingBox.kptMatches.end();) {
            cv::KeyPoint kpCurr = kptsCurr.at(it->trainIdx);
            cv::KeyPoint kpPrev = kptsPrev.at(it->queryIdx);
            double dist = cv::norm(kpCurr.pt - kpPrev.pt);

            if (dist >= mean * ratio){// || ) {
                boundingBox.kptMatches.erase(it);
                borradosMas++;
            }
            else {
                it++;
            }
        }
    }
    cout << "size " << boundingBox.kptMatches.size() << " Mas " << borradosMas << " Menos " << borradosMenos << endl;
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distRatios;
    double medianRatio=0;
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1) {
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);
        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2) {
            //double distCurr = cv::norm(kptsCurr.at(it1->trainIdx).pt - kptsCurr.at(it2->trainIdx).pt);
            //double distPrev = cv::norm(kptsPrev.at(it1->queryIdx).pt - kptsPrev.at(it2->queryIdx).pt);
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);
            
            double minDist = 60;  
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);
            //cout << "distCurr " << distCurr << " distPrev " << distPrev << endl;
            // Avoid division by zero and apply the threshold
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist) {
                
                double distRatio = distCurr / distPrev;
                //cout << "distCurr " << distCurr << " distPrev " << distPrev << " distratio "<< distRatio <<  endl;
                //if(distRatio!=1)
                distRatios.push_back(distRatio);
                medianRatio+=distRatio;
            }
        }
    }

    // Only continue if the vector of distRatios is not empty
    if (distRatios.size() == 0)
    {
        TTC = std::numeric_limits<double>::quiet_NaN();
        return;
    }

    // As with computeTTCLidar, use the median as a reasonable method of excluding outliers
    //std::sort(distRatios.begin(), distRatios.end());
    
    double medianDistRatio = medianRatio/distRatios.size();//distRatios[distRatios.size() / 2];

    // Finally, calculate a TTC estimate based on these 2D camera features
    TTC = (-1.0 / frameRate) / (1 - medianDistRatio);
    cout << "TTC " << TTC << " frameRate " << frameRate << " medianDistRatio "<< medianDistRatio <<  endl;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double t = (double)cv::getTickCount();

    double d0;
    for(auto punto:lidarPointsPrev)
    {
        d0+=punto.x;
    }
    d0=d0/lidarPointsPrev.size();
    double d1;
    for(auto punto:lidarPointsCurr)
    {
        d1+=punto.x;
    }
    d1=d1/lidarPointsCurr.size();

    TTC = d1 * (1.0 / frameRate) / (d0 - d1);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "TTC Lidar in " << 1000 * t / 1.0 << " ms" << endl;
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    double t = (double)cv::getTickCount();

    std::multimap<int, int> mmap {};
    int maxPrevBoxID = 0;

    for (const auto& prevBox : prevFrame.boundingBoxes) {
        std::map<int, int> m;
        for (const auto& currBox : currFrame.boundingBoxes) {


            for (const auto &match : matches) {

                const auto &prevKeyPoint = prevFrame.keypoints[match.queryIdx].pt;
                if (prevBox.roi.contains(prevKeyPoint)) {
                    const auto &currKeyPoint = currFrame.keypoints[match.trainIdx].pt;

                    if (currBox.roi.contains(currKeyPoint)) {

                        if(0 == m.count(currBox.boxID)) {
                            m[currBox.boxID] = 1;
                        }
                        else {
                            m[currBox.boxID]++;
                        }
                    }
                }
                //mmap.insert({currBoxID, prevBoxID});

                //maxPrevBoxID = std::max(maxPrevBoxID, prevBoxID);
            } // eof iterating all matches
        } // eof iterating all current bounding boxes
        map<int,int>::iterator it;
        int maximo=0;
        int boxId=-1;
        for (it=m.begin();it!=m.end();it++)
        {
            if(it->second>maximo)
            {
                maximo=it->second;
                boxId=it->first;
            }
        }

        bbBestMatches[prevBox.boxID] = boxId;

    } // eof iterating all previous bounding boxes
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "matchBoundingBoxes in " << 1000 * t / 1.0 << " ms" << endl;
}
