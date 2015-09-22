#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;

//Headers
bool cornerMotion(Point2f, Point2f);
void writeCorners(vector<Point2f>, vector<Point2f>);
void canalX(Size, char*, Mat*, Mat*, Mat*, Mat*);
Mat getChannel(Mat, int);

/*-------------------help functions------------------*/

Mat getChannel(Mat image, char *channel){
  int average;
  Mat hsv, output;
  vector<Mat> channels(3);

  if (strcmp(channel, "H") == 0 || strcmp(channel, "S") == 0 || strcmp(channel, "V") == 0){
    cvtColor( image, hsv, COLOR_BGR2HSV);
    image = hsv.clone();
  }

  split(image, channels);

  //GRAY section
  if(strcmp(channel, "GRAY") == 0){
    cvtColor( image, output, COLOR_BGR2GRAY );
    return output;
  }

  //RGB section
  if(strcmp(channel, "R") == 0 || strcmp(channel, "V") == 0)
    return channels[2];
  if(strcmp(channel, "G") == 0)
    return channels[1];
  if(strcmp(channel, "B") == 0)
    return channels[0];
  /*if(strcmp(channel, "RGB") == 0){
    merge(channels[0], channels[1], channels[2], 3, output);
    return output;
  }*/

  //HSV section
  if(strcmp(channel, "H") == 0){
    return channels[0];
  }
  if(strcmp(channel, "S") == 0){
    return channels[1];
  }
  if(strcmp(channel, "V") == 0){
    return channels[2];
  }
  /*if(strcmp(channel, "HSV") == 0){
    cvtColor( hsv, output, COLOR_HSV2GRAY);
    return output;
  }*/
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

/*-----------------color functions---------------*/
void canalX(Size tamanio, char *canal, Mat *imageA, Mat *imageB, Mat *frame1, Mat *frame2){
  int average;
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = imageA->ptr<Point3_<uchar> >(i,j);
        
        if (strcmp(canal, "R") == 0)
          frame1->at<uchar>(i,j) = p->z;
        else if (strcmp(canal, "G") == 0)
          frame1->at<uchar>(i,j) = p->y;
        else if (strcmp(canal, "B") == 0)
          frame1->at<uchar>(i,j) = p->x;
        else if (strcmp(canal, "RGB") == 0){
          average = (p->x + p->y + p->z)/3;
          frame1->at<uchar>(i,j) = average;
        }else if (strcmp(canal, "GRAY") == 0)
          break;
      }
  }
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = imageB->ptr<Point3_<uchar> >(i,j);

        if (strcmp(canal, "R") == 0)
          frame2->at<uchar>(i,j) = p->z;
        else if (strcmp(canal, "G") == 0)
          frame2->at<uchar>(i,j) = p->y;
        else if (strcmp(canal, "B") == 0)
          frame2->at<uchar>(i,j) = p->x;
        else if (strcmp(canal, "RGB") == 0){
          average = (p->x + p->y + p->z)/3;
          frame2->at<uchar>(i,j) = average;
        }else if (strcmp(canal, "GRAY") == 0)
          break;
      }
  }
}

/*Return an image (Mat) with the optical flow's stimate vectors.
Mat drawVectors(vector<Point2f> corners, vector<Point2f> nextPts, vector<uchar> status, vector<float> err){
  Mat image = src2.clone();
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
      arrowedLine( image, p0, p1, Scalar(100, 200, 150), 2, 8, 0, 0.2);
    }
  }
  return image;
}*/

/*void averageRGB(Size tamanio){
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
*/