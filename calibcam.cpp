#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include <stdio.h>
#include <iostream>
#include <string>

std::string filepath = "/home/jamil/build/images-1920/gambar";
int counter = 0;

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,9}; 
int fieldSize = 31;

int main()
{
  // Creating vector to store vectors of 3D points for each checkerboard image

  // cv::glob(path, images, false);

  cv::Mat frame, gray, framesave;
  // vector to store the pixel coordinates of detected checker board corners 
  std::vector<cv::Point2f> corner_pts;
  bool success;

  cv::TermCriteria critia(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 200, 0.0001);

  // Looping over all the images in the directory
  cv::VideoCapture cap;
  while(true){
    cap.open(0,cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(cv::CAP_PROP_FOURCC , cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.read(frame);
    framesave = frame.clone();
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

    // cv::imshow("gray", gray);

    // Finding checker board corners
    // If desired number of corners are found in the image then success = true  
    success = cv::findChessboardCorners(gray,cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts,  cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS | cv::CALIB_CB_NORMALIZE_IMAGE);

    /*
     * If desired number of corner are detected,
     * we refine the pixel coordinates and display 
     * them on the images of checker board
    */
    if(success)
    {

      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),critia);

      // Displaying the detected corner points on the checker board
      cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts,success);
      std::cout << "asep\n";

      }
    cv::imshow("Image",frame);
    char key = cv::waitKey(1);
    if(key == 's'){
      counter++;
      filepath = filepath + std::to_string(counter) + ".jpg";
      cv::imwrite(filepath,frame);
    }
    else if(key == 'q'){
      break;
    }
  }

  cv::destroyAllWindows();
  return 0;
}