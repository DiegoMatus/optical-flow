//Pyramid L-K optical flow code
#include <cv.h>
#include <cxcore.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src_gray;

int maxTrackbar = 100;
int MAX_CORNERS = 23;

RNG rng(12345);
const char* source_window = "Image";

/// Function header
void goodFeaturesToTrack_Demo( int, void* );

int main(int argc, char** argv){
	//Initialize, load two images from the file
	//system, and allocate the images and other
	//structures we will need for results.
	IplImage* imgA = cvLoadImage( "frame1.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	IplImage* imgB = cvLoadImage( "frame2.jpg", CV_LOAD_IMAGE_GRAYSCALE);

	CvSize img_sz = cvGetSize( imgA );
	int win_size = 10;

	IplImage* imgC = cvLoadImage( "OpticalFlow1.jpg",CV_LOAD_IMAGE_UNCHANGED);

	//The first thing we need to do is get the
	//features we want to track.
	IplImage* eig_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );
	IplImage* tmp_image = cvCreateImage( img_sz, IPL_DEPTH_32F, 1 );

	int corner_count = MAX_CORNERS;
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];

	goodFeaturesToTrack_Demo( 0, 0 );
	
	cvFindCornerSubPix(
		imgA,
		cornersA,
		corner_count,
		cvSize(win_size,win_size),
		cvSize(-1, 1),
		cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03)
	);

	//Call the Lucas Kanade algorithm
	char features_found[ MAX_CORNERS ];
	float feature_errors[ MAX_CORNERS ];

	CvSize pyr_sz = cvSize( imgA->width+8, imgB->height/3 );

	IplImage* pyrA = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );
	IplImage* pyrB = cvCreateImage( pyr_sz, IPL_DEPTH_32F, 1 );

	CvPoint2D32f* cornersB = new CvPoint2D32f[ MAX_CORNERS ];

	cvCalcOpticalFlowPyrLK(
		imgA,
		imgB,
		pyrA,
		pyrB,
		cornersA,
		cornersB,
		corner_count,
		cvSize( win_size, win_size),
		5,
		features_found,
		feature_errors,
		cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3),
		0
	);

	//Now make some image of what we are looking at:
	for( int i=0; i<corner_count; i++){
		if(features_found[i]==0 || feature_errors[i]>550){
			printf("Error is %f\n", feature_errors[i]);
			continue;
		}
		printf("Got it\n");
		CvPoint p0 = cvPoint(
			cvRound( cornersA[i].x ),
			cvRound( cornersA[i].y )
		);
		CvPoint p1 = cvPoint(
			cvRound( cornersB[i].x),
			cvRound( cornersB[i].y)
		);
		cvLine( imgC, p0, p1, CV_RGB(255,0,0), 2);
	}

	cvNamedWindow("ImageA", 0);
	cvNamedWindow("ImageB", 0);
	cvNamedWindow("LKpyr_OpticalFlow", 0);

	cvShowImage("ImageA", imgA);
	cvShowImage("ImageB", imgB);
	cvShowImage("LKpyr_OpticalFlow", imgC);

	cvWaitKey(0);

	return 0;
}

void goodFeaturesToTrack_Demo( int, void* )
{
	if( MAX_CORNERS < 1 ) { MAX_CORNERS = 1; }

	/// Parameters for Shi-Tomasi algorithm
	vector<Point2f> corners;
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;

	/// Copy the source image
	Mat copy;
	copy = src.clone();

	/// Apply corner detection
	goodFeaturesToTrack( imgA,
	           corners,
	           MAX_CORNERS,
	           qualityLevel,
	           minDistance,
	           Mat(),
	           blockSize,
	           useHarrisDetector,
	           k );

  /// Draw corners detected
  cout<<"** Number of corners detected: "<<corners.size()<<endl;
  int r = 4;
  for( size_t i = 0; i < corners.size(); i++ )
     { circle( copy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)), -1, 8, 0 ); }

  /// Show what you got
  namedWindow( source_window, WINDOW_AUTOSIZE );
  imshow( source_window, copy );
}