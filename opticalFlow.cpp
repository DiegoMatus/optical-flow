/**
 * @function goodFeaturesToTrack_Demo.cpp
 * @brief Demo code for detecting corners using Shi-Tomasi method
 * @author OpenCV team
 */

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src2, src_gray1, src_gray2;

int maxCorners = 350;
int maxTrackbar = 500;

typedef Vec<int, 3> Vec3b;
RNG rng(12345);
const char* source_window = "ImageA";
const char* source_windowB = "ImageB";
const char* source_windowR = "Results";

/// Function header
void opticalFlow( int, void* );
void averageRGB(Size);
void canalB(Size);
void canalG(Size);
void canalR(Size);

/**
 * @function main
 */
int main( int, char** argv ){
  int op = atoi(argv[3]);
  /// Load source image and convert it to gray
  src = imread( argv[1], 1 );
  src2 = imread( argv[2], 1 );
  Size tamanio = src.size();

  cvtColor( src, src_gray1, COLOR_BGR2GRAY );
  cvtColor( src2, src_gray2, COLOR_BGR2GRAY );

  if(strcmp(argv[3], "RGB") == 0)
      averageRGB(tamanio);
  else if (strcmp(argv[3], "R") == 0)
      canalR(tamanio);
  else if (strcmp(argv[3], "G") == 0)
      canalG(tamanio);
  else if (strcmp(argv[3], "B") == 0)
      canalB(tamanio);
  else if (strcmp(argv[3], "GRAY") == 0)
      ;
  /// Create Window
  namedWindow( source_window, WINDOW_AUTOSIZE );

  /// Create Trackbar to set the number of corners
  createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, opticalFlow );

  imshow( source_window, src );

  opticalFlow( 0, 0 );

  waitKey(0);
  return(0);
}

void averageRGB(Size tamanio){
  int average;
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = src.ptr<Point3_<uchar> >(i,j);
        average = (p->x + p->y + p->z)/3;

        src_gray1.at<uchar>(i,j) = average;
      }
  }
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = src2.ptr<Point3_<uchar> >(i,j);
        average = (p->x + p->y + p->z)/3;

        src_gray2.at<uchar>(i,j) = average;
      }
  }
}

void canalB(Size tamanio){
  int average;
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = src.ptr<Point3_<uchar> >(i,j);

        src_gray1.at<uchar>(i,j) = p->x;

      }
  }
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = src2.ptr<Point3_<uchar> >(i,j);

        src_gray2.at<uchar>(i,j) = p->x;
      }
  }
}

void canalG(Size tamanio){
  int average;
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = src.ptr<Point3_<uchar> >(i,j);

        src_gray1.at<uchar>(i,j) = p->y;
      }
  }
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = src2.ptr<Point3_<uchar> >(i,j);

        src_gray2.at<uchar>(i,j) = p->y;
      }
  }
}

void canalR(Size tamanio){
  int average;
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = src.ptr<Point3_<uchar> >(i,j);

        src_gray1.at<uchar>(i,j) = p->z;
      }
  }
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = src2.ptr<Point3_<uchar> >(i,j);

        src_gray2.at<uchar>(i,j) = p->z;
      }
  }
}

//Function for estimate the euclidean distance beetwen both points.
bool cornerMotion(Point2f p1, Point2f p2){
  int euclidean_distance = sqrt(pow((p2.x - p1.x),2) + pow((p2.y - p1.y),2));
  if (euclidean_distance >= 1)
    return true;

  return false;
}

//Function that write the coordenates in txt file
void writeCorners(vector<Point2f> corners, vector<Point2f> nextPts){
  FILE *file;
  char corner[50];

  if(!(file=fopen("coordenadas.txt", "w+"))){
    exit(0);
  }

  for(int i=0; i<corners.size(); i++){
    if(cornerMotion(corners[i], nextPts[i])){
        sprintf(corner, "(%f,%f) - (%f,%f)\n", corners[i].x, corners[i].y, nextPts[i].x, nextPts[i].y);
        //printf("%s\n", corner.c_str());
        fputs(corner, file);
    }
  }

  fclose(file);
}

/**
 * @function opticalFlow.cpp
 * @brief Apply Shi-Tomasi corner detector
 */
void opticalFlow( int, void* ){
  if( maxCorners < 1 ) { maxCorners = 1; }

  /// Parameters for Shi-Tomasi algorithm
  vector<Point2f> corners;
  vector<Point2f> nextPts;
  vector<uchar> status;
  vector<float> err;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = 0;
  double k = 0.04;

    /// Copy the source image
  Mat copy, copy2;
  copy = src.clone();
  copy2 = src2.clone();

  /// Apply corner detection
  goodFeaturesToTrack( src_gray1,
               corners,
               maxCorners,
               qualityLevel,
               minDistance,
               Mat(),
               blockSize,
               useHarrisDetector,
               k );


  /// Found the corners
  Size subPixWinSize(10,10), winSize(31,31);
  TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
 
  cornerSubPix(
    src_gray1,
    corners,
    subPixWinSize,
    Size(-1, 1),
    termcrit
  );

  ///Call the Lucas Kanade algorith
  calcOpticalFlowPyrLK(
      src_gray1,
      src_gray2,
      corners,
      nextPts,
      status,
      err,
      winSize,
      3,
      termcrit,
      0,
      0.001
  );

  //Now make some image of what we are looking at:
  writeCorners(corners, nextPts);

  for( int i=0; i<corners.size(); i++){
    if(status[i]==0 || err[i]>550){
      printf("Error is %f\n", err[i]);
      continue;
    }
    if (cornerMotion(corners[i], nextPts[i])){
      CvPoint p0 = cvPoint(
        cvRound( corners[i].x ),
        cvRound( corners[i].y )
      );
      CvPoint p1 = cvPoint(
        cvRound( nextPts[i].x),
        cvRound( nextPts[i].y)
      );
      arrowedLine( copy2, p0, p1, Scalar(100, 200, 150), 2, 8, 0, 0.2);
    }
  }

  /// Draw corners detected
  cout<<"** Number of corners detected: "<<corners.size()<<endl;
  int r = 2;
  for( size_t i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(255, 170, 155), -1, 8, 0 ); }

  namedWindow(source_window, WINDOW_AUTOSIZE);
  namedWindow(source_windowB, WINDOW_AUTOSIZE);
  namedWindow(source_windowR, WINDOW_AUTOSIZE);
  imshow(source_window, copy);
  imshow(source_windowB, src_gray2);
  imshow(source_windowR, copy2);
}
