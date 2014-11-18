#include "lib/CameraInterface.h"

#define SCALE 2

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
  ROS_INFO("Processing image with dimensions %d x %d",
           lastImage->width, lastImage->height);

  int cols = lastImage->width;
  int rows = lastImage->height;

  Scalar ycrcb_min = Scalar(0, 100, 100);
  Scalar ycrcb_max = Scalar(50, 255, 255);

  Mat ycrcb_frame(cols, rows, CV_8UC4);
  Mat thresholded(cols, rows, CV_8UC1);

  Mat image = lastImage -> image;

  // GaussianBlur(image, image, Size(9, 9), 1);
  cvtColor(image, ycrcb_frame, CV_RGB2HSV);
  inRange(ycrcb_frame, ycrcb_min, ycrcb_max, thresholded);

  vector <vector <Point> > contours, dest_contours;
  vector <Vec4i> hierarchy;
  findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  dest_contours.resize(contours.size());
  for (int k = 0; k < contours.size(); k++)
  {
    approxPolyDP(Mat(contours[k]), dest_contours[k], 3, true);
  }
  for ( int i = 0; i < dest_contours.size(); i++ )
  {
    Scalar color = Scalar(255, 255, 255 );
    drawContours( thresholded, dest_contours, i, color, 2, 8, hierarchy, 0, Point() );
  }

  namedWindow("contours", 1);
  imshow("contours", thresholded);
  waitKey(3);

  //Detect triangle.
  if (dest_contours.size() == 3)
    cout << "Found it?" << endl;

}