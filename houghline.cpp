#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <cstring>
#include <ctime>
#include <limits.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <iomanip>
#include <signal.h>

using namespace std;
using namespace cv;

int main(int argc, char** argv){
    Mat dst, cdst, edges, Resize, cdstP;

    Mat image = imread("./Goal.jpg", IMREAD_GRAYSCALE);

    if(image.empty())
        cout << "gambar tidak ada" << endl;

    // imshow("Result", image);
    // waitKey(0);

    threshold(image, image, 128, 255, THRESH_BINARY);

    GaussianBlur(image, image, Size(5,5),0);
    
    // Find the horizontal lines
    vector<Vec4i> lines;
    HoughLinesP(image, lines, 1, CV_PI / 180, 100, 50, 10);

    vector<Point2f> linePoints;
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i line = lines[i];
        linePoints.push_back(Point2f(line[0], line[1]));
        linePoints.push_back(Point2f(line[2], line[3]));
    }

    // Find the corners of the goal
    vector<Point2f> corners;
    for (size_t i = 0; i < linePoints.size(); i += 2) {
        corners.push_back((linePoints[i] + linePoints[i + 1]) / 2);
    }

    // Draw the corners of the goal
    for (size_t i = 0; i < corners.size(); i++) {
        circle(image, corners[i], 5, Scalar(255, 0, 0), -1);
    }

    Rect rect = Rect(corners[0], corners[2]);
    rectangle(image, rect, Scalar(255,0,0),5);

    // Show the image
    imshow("Gawang Sepak Bola", image);
    waitKey(0);

    // Mat gray = cvtColor(Resize, dst, COLOR_GRAY2BGR);
    // Canny(Resize, edges, 50, 200);
    // vector<Vec4i> lines;
    // cvtColor(Resize, cdst, COLOR_GRAY2GR);
    // cdstP = cdst.clone();

    //     vector<Vec2f> lines; // will hold the results of the detection
    // HoughLinesP(edges, lines, 1, CV_PI/180, thresh, 10, 250 ); // runs the actual detection

    // for (size_t i=0; i<lines.size(); i++) {
    //     Vec4i l = lines[i];
    //     line(edges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 3, LINE_AA);
    // }
    // Show result image
    // imshow("Result Image", edges);

    // // Draw the lines
    // for( size_t i = 0; i < lines.size(); i++ )
    // {
    //     float rho = lines[i][0], theta = lines[i][1];
    //     Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a*rho, y0 = b*rho;
    //     pt1.x = cvRound(x0 + 1000*(-b));
    //     pt1.y = cvRound(y0 + 1000*(a));
    //     pt2.x = cvRound(x0 - 1000*(-b));
    //     pt2.y = cvRound(y0 - 1000*(a));
    //     line( cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
    // }
    // // Probabilistic Line Transform
    // vector<Vec4i> linesP; // will hold the results of the detection
    // HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
    // // Draw the lines
    // for( size_t i = 0; i < linesP.size(); i++ )
    // {
    //     Vec4i l = linesP[i];
    //     line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    // }
    // Show results
    // imshow("Source", src);
    // imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    // imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
    // Wait and Exit

    // waitKey(0);

    // char key = waitKey(1);
    // if(key == 'q') break;

    return 0;

}