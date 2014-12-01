#include "lib/CameraInterface.h"
#include <cmath>

#define SCALE 2

double area(vector<Point> *);

void CameraInterface::image_callback(
  const sensor_msgs::Image::ConstPtr &msg)
{
  CameraImage *ptr = lastImage;
  lastImage = new CameraImage(msg);
  process();
  if (ptr != NULL)
    delete ptr;
}

void CameraInterface::process()
{
  int cols = lastImage->width;
  int rows = lastImage->height;

  Mat hsv_frame(cols, rows, CV_8UC4);
  Mat thresholded(cols, rows, CV_8UC1);

  Mat image = lastImage -> image;

  cvtColor(image, hsv_frame, CV_RGB2HSV);
  inRange(hsv_frame, hsv_min, hsv_max, thresholded);

  vector <vector <Point> > contours, approximation;
  vector <Vec4i> hierarchy;
  findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  approximation.resize(contours.size());
  for (int k = 0; k < contours.size(); k++)
  {
    approxPolyDP(Mat(contours[k]), approximation[k], 1, true);
  }
  for ( int i = 0; i < approximation.size(); i++ )
  {
    if (approximation[i].size() == 3)
    {
      double areaNum = area(&approximation[i]);
      if (areaNum > 25)
      {
        Scalar color = Scalar(255, 255, 255);
        cout << approximation[i] << endl;
        cout << "Found triangle of area: " << areaNum << endl;
        drawContours( thresholded, approximation, i, color, 2, 8, hierarchy, 0, Point() );
      }
    }
  }
  namedWindow("contours", 1);
  imshow("contours", thresholded);
  waitKey(3);

  //Detect triangle.

}

void CameraInterface::generateMinMax(Scalar ideal, Scalar range, enum hsv_type type)
{
  double h = max((int)(ideal.val[H] - range.val[H]), 0);
  double s = max((int)(ideal.val[S] - range.val[S]), 0);
  double v = max((int)(ideal.val[V] - range.val[V]), 0);

  hsv_min = Scalar(h, s, v);

  h = min((int)(ideal.val[H] + range.val[H]), 255);
  s = min((int)(ideal.val[S] + range.val[S]), 255);
  v = min((int)(ideal.val[V] + range.val[V]), 255);

  hsv_max = Scalar(h, s, v);
}

double area(vector<Point> *vertices)
{
  if (vertices->size() != 3)
    return -1;
  double A[2] = {(*vertices)[0].x, (*vertices)[0].y};
  double B[2] = {(*vertices)[1].x, (*vertices)[1].y};
  double C[2] = {(*vertices)[2].x, (*vertices)[2].y};

  return abs(A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2.0;
}