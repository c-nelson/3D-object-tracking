
#include <iostream>
#include <algorithm>
#include <numeric>
#include <set>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>

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


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame, cv::Mat &img)
{

  // TODO: MAKE A CHECK FOR IF KEYPOINT IN WITHIN 2 BOUNDING BOXES
  std::multimap<int, int> bb_matches;
  std::vector<cv::KeyPoint> c_pts;
  std::vector<cv::KeyPoint> p_pts;
  std::vector<cv::DMatch> in_matches;
  std::vector<cv::DMatch> match_copy = matches;
  // for all matches
  for(auto it = match_copy.begin(); it != match_copy.end(); ++it) {
    bool match_placed = false;
    int curr_index = it->queryIdx;
    int prev_index = it->trainIdx;
    auto curr_frame_bbs = currFrame.boundingBoxes;
    auto prev_frame_bbs = prevFrame.boundingBoxes;
    cout << "SIZE" << bb_matches.size() << endl;
    cout << "SIZE CPTS" << c_pts.size() << endl;
    for(auto curr_bb_it = curr_frame_bbs.begin(); !match_placed && curr_bb_it != curr_frame_bbs.end(); ++curr_bb_it){
      for(auto prev_bb_it = prev_frame_bbs.begin(); !match_placed && prev_bb_it != prev_frame_bbs.end(); ++prev_bb_it){
        if(curr_bb_it->roi.contains(currFrame.keypoints[curr_index].pt) && prev_bb_it->roi.contains(prevFrame.keypoints[prev_index].pt) && !match_placed){
          bb_matches.insert(std::pair<int, int>(curr_bb_it->boxID, prev_bb_it->boxID));
          match_placed = true;
          in_matches.push_back((*it));
          c_pts.push_back(currFrame.keypoints[curr_index]);
          p_pts.push_back(prevFrame.keypoints[prev_index]);
        }
      }
    }
    //if(!match_placed) match_copy.erase(it);
    // store box ids in multimap
  }
  std::set<int> keys;
  for(auto matches_it = bb_matches.begin(); matches_it != bb_matches.end(); ++matches_it){
    keys.insert(matches_it->first);
  }
  int num_boxes = keys.size();

  for(auto key_it = keys.begin(); num_boxes > 0 && key_it != keys.end(); ++key_it){
    std::pair <std::multimap<int,int>::iterator, std::multimap<int,int>::iterator> ret;
    ret = bb_matches.equal_range(*key_it);
    int max = -1;
    int max_id = -1;
    for(auto key_range = ret.first; key_range != ret.second; ++key_range){
      std::vector<int> counts(keys.size(), 0);
      for(int i = 0; i < counts.size(); i++){
        if(key_range->second == i) counts[i]++;
        if(counts[i] > max) {
          max = counts[i];
          max_id = i;
        }
      }
    }
    bbBestMatches.insert(std::pair<int, int>(*key_it, max_id));
  }
    std::cout << "======== best matches =======" << std::endl;
    for(auto best_it = bbBestMatches.begin(); best_it != bbBestMatches.end(); ++best_it){
      std::cout << "id1: " << best_it->first << "\tid2: " << best_it->second  << endl;
    }
  //}
  bool bVis = true;
  if(bVis) {

    cv::Mat visImg = img.clone();
    cv::Mat c_img = currFrame.cameraImg.clone();
    cv::Mat p_img = prevFrame.cameraImg.clone();

    cv::drawKeypoints(c_img, c_pts, c_img);
    cv::drawKeypoints(p_img, p_pts, p_img);
    //cv::drawMatches(p_img, p_pts, c_img, c_pts, in_matches, visImg, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    string windowName = "keypoints";
    cv::namedWindow( windowName, 2 );
    cv::imshow( windowName, c_img );
    cv::waitKey(0); // wait for key to be pressed
  }
}
