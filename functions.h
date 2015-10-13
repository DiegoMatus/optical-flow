#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <math.h>

using namespace std;
using namespace cv;

struct Vector {
  Point2f p1;
  Point2f p2;
} ;

//Headers
Mat getChannel(Mat, int);
float euclideanDistance(Vector);
void writeCorners(vector<Point2f>, vector<Point2f>);
vector<Vector> getGoodVectors(vector<Point2f>, vector<Point2f>, vector<uchar>, vector<float>);
//Mat average(vector<Mat>);
//void putMat(Size, char, Mat*);
bool isOutlier(Size, vector<Vector>, Vector, float, float, float);
//void canalX(Size, char*, Mat*, Mat*, Mat*, Mat*);

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

  //R G B and H S V section
  if(strcmp(channel, "R") == 0 || strcmp(channel, "V") == 0)
    return channels[2];
  if(strcmp(channel, "G") == 0 || strcmp(channel, "S") == 0)
    return channels[1];
  if(strcmp(channel, "B") == 0 || strcmp(channel, "H") == 0)
    return channels[0];
  if(strcmp(channel, "RGB") == 0 || strcmp(channel, "HSV") == 0){
    return output;
  }
}

//Function for estimate average from three (n) channels
/*Mat average(vector<Mat> channels){
      for (int i = 0; i < image.size().height; ++i)
    {
      for (int j = 0; j < image.size().width; ++j)
      {
        Vec3b hsv2=image.at<Vec3b>(i,j);
        if ( hsv2.val[0] = 150)
            hsv2.val[0] = 200;
        printf("[%d]", hsv2.val[0]);
      }
        printf("\n");
  return channels[0]; 
}*/

//Function for estimate the euclidean distance beetwen both points.
float euclideanDistance(Vector v){
  float euclidean_distance = sqrt(pow((v.p2.x - v.p1.x),2) + pow((v.p2.y - v.p1.y),2));
  return euclidean_distance;
}

float getAngle(Vector v){
  float PI = 3.14159265;
  float angle = atan2((v.p2.x-v.p1.x), (v.p2.y-v.p1.y)) * 180 / PI;
  angle += 270;
  if (angle > 360)
    angle -= 360;
  return angle;
}

vector<Vector> getGoodVectors(vector<Point2f> corners, vector<Point2f> nextPts, vector<uchar> status, vector<float> err){
  vector<Vector> vectors;
  Vector v;
  int position = 0;

  for(int i=0; i<corners.size(); i++){
    if(status[i]==0 || err[i]>550){
      printf("Error is %f\n", err[i]);
      continue;
    }
    v.p1 = corners[i];
    v.p2 = nextPts[i];

    if(euclideanDistance(v) > 2){ //Desplazamiento mayor a 1px.
      vectors.push_back(v);
      position++;
    }
  }
  return vectors;
}

Vector getMaxVector(vector<Vector> vectors){
  Vector v;
  float magnitude, max_vector = 0;
  for (int i = 0; i < vectors.size(); i++)
  {
    magnitude = euclideanDistance(vectors[i]);
    if (magnitude > max_vector)
    {
      max_vector = magnitude;
      v.p1.x = vectors[i].p1.x;
      v.p1.y = vectors[i].p1.y;
      v.p2.x = vectors[i].p2.x;
      v.p2.y = vectors[i].p2.y;
    }
  }
  return v;
}

//Function that write the coordenates in txt file
void writeCorners(vector<Vector> vectors){
  FILE *file;
  char vector[75];
  float magnitude, angle;

  if(!(file=fopen("coordenadas.csv", "w+"))){
    exit(0);
  }

  for(int i=0; i<vectors.size(); i++){
        magnitude = euclideanDistance(vectors[i]);
        angle = getAngle(vectors[i]);
        sprintf(vector, "%f,%f,%f,%f,%f,%f\n", vectors[i].p1.x, vectors[i].p1.y, vectors[i].p2.x, vectors[i].p2.y, magnitude, angle);
        fputs(vector, file);
  }

  fclose(file);
}

Vector getWindow(Size window_size, Point2f begin, float radio){
  Vector window;
  window.p1.x = begin.x - radio;
  window.p1.y = begin.y - radio;
  window.p2.x = begin.x + radio;
  window.p2.y = begin.y + radio;

  if (window.p1.x < 0)
    window.p1.x = 0;
  if (window.p1.y < 0)
    window.p1.y = 0;
  if (window.p2.x > window_size.width)
    window.p2.x = window_size.width;
  if (window.p2.y > window_size.height)
    window.p2.y = window_size.height;

  return window;
}

bool inRange(float value, float begin_range, float end_range){
  bool in_range = false;
  if (value > begin_range && value < end_range)
  {
    in_range = true;
  }
  return in_range;
}

Point2f getRange(float value, float error_margin){
  Point2f range;
  range.x = value - error_margin;
  range.y = value + error_margin;

  return range;
}

vector<Vector> getWindowVectors(vector<Vector> vectors, Vector window){
  vector<Vector> window_vectors;
  for (int i = 0; i < vectors.size(); i++){
    if (vectors[i].p1.x > window.p1.x && vectors[i].p1.y > window.p1.y && vectors[i].p2.x < window.p2.x && vectors[i].p2.y < window.p2.y)
    {
      window_vectors.push_back(vectors[i]);
    }
  }
  return window_vectors;
}

int getSimilarVectors(vector<Vector> window_vectors, Point2f angle_range, Point2f magnitude_range){
  float angle, magnitude, new_range;
  int similar_vectors = -1;

  for (int i = 0; i < window_vectors.size(); i++){
    angle = getAngle(window_vectors[i]);
    magnitude = euclideanDistance(window_vectors[i]);
    if (angle_range.x < 0)
    {
      new_range = angle_range.x + 360;
      if (((angle > new_range) || (angle > 0 && angle < angle_range.y)) && (magnitude > magnitude_range.x && magnitude < magnitude_range.y))
      {
        similar_vectors++;
      }
    }else if (angle_range.y > 360)
    {
      new_range = angle_range.y - 360;
      if (((angle < new_range) || (angle < 360 && angle > angle_range.x)) && (magnitude > magnitude_range.x && magnitude < magnitude_range.y))
      {
        similar_vectors++;
      }
    }else if ((angle > angle_range.x && angle < angle_range.y) && (magnitude > magnitude_range.x && magnitude < magnitude_range.y)){
      similar_vectors++;
    }
  }
  return similar_vectors;
}

bool isOutlier(Size window_size, vector<Vector> vectors, Vector v, float radio, float error_angle, float error_magnitude){
  bool outlier = false;
  int similar_vectors;
  vector<Vector> window_vectors;

  float angle = getAngle(v);
  Point2f angle_range = getRange(angle, error_angle);

  float magnitude = euclideanDistance(v);
  Point2f magnitude_range = getRange(magnitude, error_magnitude);

  printf("\nVector a analizar: [%f,%f] - [%f,%f]\nÁngulo: %f\tMagnitud: %f\n",
          v.p1.x, v.p1.y, v.p2.x, v.p2.y, angle, magnitude);

  Vector window = getWindow(window_size, v.p1, radio);
  window_vectors = getWindowVectors(vectors, window);
  similar_vectors = getSimilarVectors(window_vectors, angle_range, magnitude_range);

  printf("Ventana a analizar: [%f,%f] - [%f,%f]\tRadio: %f\nMargen de error °: %f\tRango del ángulo: %f-%f\nMargen de error px: %f\tRango de la magnitud: %f-%f\n\n", 
          window.p1.x, window.p1.y, window.p2.x, window.p2.y, radio, error_angle, angle_range.x, angle_range.y, error_magnitude, magnitude_range.x, magnitude_range.y);
  printf("Total de vectores: %d\nTotal de vectores en la ventana: %d\nTotal de vectores similares; %d\n\n",
          (int)vectors.size(), (int)window_vectors.size(), similar_vectors);

  //Ajustar angulos que se pasen de 360°|0° (360 - angle o angle - 360)
  if (similar_vectors > 0)
  {
      printf("No es outlier!!! :3 \n");
  }else{
      printf("It's outlier!!! :O \n");
      outlier = true;
  }

  return outlier;
}

//-----------------color functions---------------
/*void canalX(Size tamanio, char *canal, Mat *imageA, Mat *imageB, Mat *frame1, Mat *frame2){
  int average;
  for (int i = 0; i < tamanio.height; i++){
      for (int j = 0; j < tamanio.width; j++){
        Point3_<uchar>* p = imageA->ptr<Point3_<uchar> >(i,j);
        
        if (strcmp(canal, "H") == 0)
          frame1->at<uchar>(i,j) = p->z;
        else if (strcmp(canal, "S") == 0)
          frame1->at<uchar>(i,j) = p->y;
        else if (strcmp(canal, "V") == 0)
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

        if (strcmp(canal, "H") == 0)
          frame2->at<uchar>(i,j) = p->z;
        else if (strcmp(canal, "S") == 0)
          frame2->at<uchar>(i,j) = p->y;
        else if (strcmp(canal, "V") == 0)
          frame2->at<uchar>(i,j) = p->x;
        else if (strcmp(canal, "RGB") == 0){
          average = (p->x + p->y + p->z)/3;
          frame2->at<uchar>(i,j) = average;
        }else if (strcmp(canal, "GRAY") == 0)
          break;
      }
  }
}*/

/*Return an image (Mat) with the optical flow's stimate vectors.
Mat drawVectors(vector<Point2f> corners, vector<Point2f> nextPts, vector<uchar> status, vector<float> err){
  Mat image = src2.clone();
  for( int i=0; i<corners.size(); i++){
    if(status[i]==0 || err[i]>550){
      printf("Error is %f\n", err[i]);
      continue;
    }
    if (euclideanDistance(v)){
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