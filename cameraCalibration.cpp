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

#define ul		  31
#define H       0
#define S       1
#define V       2
#define E       0
#define d       1
#define C       2
#define RECT    0
#define CROSS   1
#define ELIPSE  2

using namespace std;
using namespace cv;

char file[] = {"/home/jamil/build/hsv_data/omni0.ul"};

Point initialClickPoint, currentMousePoint;
bool mouseIsDragging, mouseMove, rectangleSelected;
Rect rectangleROI;
vector<int> H_ROI, S_ROI, V_ROI;

int wMin[3]={0,0,0},wMax[3]={255,255,255},ED[3]={0,0,0};
int mode = 0;
Mat erosion_src, dilation_src, erosion_dst, dilation_dst;

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,9}; 
int fieldSize = 31;

//========== TRACKBAR ==========// 
void trackbar(){
  createTrackbar("H/Y MIN", "trackbar", &wMin[H], 255, 0);
  createTrackbar("S/U MIN", "trackbar", &wMin[S], 255, 0);
  createTrackbar("V MIN", "trackbar", &wMin[V], 255, 0);
  createTrackbar("H/Y MAX", "trackbar", &wMax[H], 255, 0);
  createTrackbar("S/U MAX", "trackbar", &wMax[S], 255, 0);
  createTrackbar("V MAX", "trackbar", &wMax[V], 255, 0);
  createTrackbar("E", "trackbar", &ED[E], 100, 0);
  createTrackbar("D", "trackbar", &ED[d], 100, 0);
  createTrackbar("MC", "trackbar", &ED[C], 100, 0);
}
void trackBar_update(){
  setTrackbarPos("H/Y MIN", "trackbar", wMin[H]);
  setTrackbarPos("S/U MIN", "trackbar", wMin[S]);
  setTrackbarPos("V MIN", "trackbar", wMin[V]);
  setTrackbarPos("H/Y MAX", "trackbar", wMax[H]);
  setTrackbarPos("S/U MAX", "trackbar", wMax[S]);
  setTrackbarPos("V MAX", "trackbar", wMax[V]);
  setTrackbarPos("E", "trackbar", ED[E]);
  setTrackbarPos("D", "trackbar", ED[d]);
  setTrackbarPos("MC", "trackbar", ED[C]);
}

void expData(int MinH, int MinS, int MinV, int MaxH, int MaxS, int MaxV, int Er, int Dl, int Cl)
{
	ofstream output (file);
	if(output.is_open()){
		cout<<"Alhamdulillah"<<endl;
		output << MinH<<";"<<MinS<<";"<<MinV<<";"<<endl;
		output << MaxH<<";"<<MaxS<<";"<<MaxV<<";"<<endl;
		output << Er<<";"<<Dl<<";"<<Cl<<";"<<endl;
	}else cout<<"Gak isok bukak!"<<endl;
	output.close();
}

void impData()
{
	ifstream buka(file);
	string line;
	int j=0, buf=0, data=0;

	if (buka.is_open()){
		while (buka.good()){
			char buffer[5] = {};
			getline(buka,line);
			for(int i=0; i<line.length(); i++){
				buffer[j] = line[i];
				if(line[i]==';'){
					j = 0;
					buf = atoi(buffer);
					if(data<3){
						wMin[data] = buf;printf("wMin : %d\n", wMin[data]);
					}else if(data<6){
						wMax[data%3] = buf;printf("wMax : %d\n", wMax[data%3]);
					}else if(data<9){
						ED[data%6] = buf;printf("ED : %d\n", ED[data%6]);
					}
					data++;
					continue;
				}
				j++;
			}
		}
		buka.close();
	}
}

void recordHSV_Values(Mat img, Mat hsv_frame){
if(mouseMove == false && rectangleSelected == true){
    if(H_ROI.size()>0)  H_ROI.clear();
    if(S_ROI.size()>0)  S_ROI.clear();
    if(V_ROI.size()>0)  V_ROI.clear();
    if(rectangleROI.width>1 || rectangleROI.height>1){
      for(int i = rectangleROI.x; i<rectangleROI.x + rectangleROI.width; i++){
        for(int j = rectangleROI.y; j<rectangleROI.y + rectangleROI.height; j++ ){
          H_ROI.push_back((int)hsv_frame.at<Vec3b>(j,i)[0]);
          S_ROI.push_back((int)hsv_frame.at<Vec3b>(j,i)[1]);
          V_ROI.push_back((int)hsv_frame.at<Vec3b>(j,i)[2]);
        }
      }
    }
    if(H_ROI.size()>0){
      wMin[H] = *min_element(H_ROI.begin(), H_ROI.end());
      wMax[H] = *max_element(H_ROI.begin(), H_ROI.end());
    }
    if(S_ROI.size()>0){
      wMin[S] = *min_element(S_ROI.begin(), S_ROI.end());
      wMax[S] = *max_element(S_ROI.begin(), S_ROI.end());
    }
    if(V_ROI.size()>0){
      wMin[V] = *min_element(V_ROI.begin(), V_ROI.end());
      wMax[V] = *max_element(V_ROI.begin(), V_ROI.end());
    }

    trackBar_update();
    rectangleSelected = false;
  }

  if(mouseMove == true)
    rectangle(img, initialClickPoint, Point(currentMousePoint.x, currentMousePoint.y), Scalar(0, 0, 255), 1, 8, 0);

}

//========== VISON FUNCTION ==========//
Mat Erosion(Mat img){
  erosion_src = img;
  int erosion_type;
  if(mode==RECT) erosion_type = MORPH_RECT;
  else if(mode==CROSS) erosion_type = MORPH_CROSS;
  else if(mode==ELIPSE) erosion_type = MORPH_ELLIPSE;

  Mat element = getStructuringElement(erosion_type, Size(2*ED[E] + 1, 2*ED[E]+1 ), Point(ED[E], ED[E]) );
  erode(erosion_src, erosion_dst, element);
  return erosion_dst;
}

Mat Dilation(Mat img)
{
  dilation_src = img;

  int dilation_type;
  if(mode==RECT) dilation_type = MORPH_RECT;
  else if(mode==CROSS) dilation_type = MORPH_CROSS;
  else if(mode==ELIPSE) dilation_type = MORPH_ELLIPSE;

  Mat element = getStructuringElement( dilation_type,
                                     Size( 2*ED[d] + 1, 2*ED[d]+1 ),
                                     Point( ED[d], ED[d] ) );

  dilate(dilation_src, dilation_dst, element );
  morphologyEx( dilation_dst, dilation_dst, MORPH_CLOSE, element, Point(-1,-1), ED[C] ); 

  return dilation_dst;
}

Mat GetThresImage(Mat imgFrame, Mat img){
  Mat imHSV, thres;
  cvtColor(img, imHSV, COLOR_BGR2HSV);

  recordHSV_Values(imgFrame, imHSV);
  inRange(imHSV, Scalar(wMin[H],wMin[S],wMin[V]), Scalar(wMax[H],wMax[S],wMax[V]), thres);
  return thres;

}

bool isContourNearBorder(const vector<Point>& contour, const Mat& frame, int threshold = 5) {
    for (const auto& point : contour) {
        if (point.x <= threshold || point.y <= threshold || point.x >= frame.cols - threshold || point.y >= frame.rows - threshold) {
            return true;
        }
    }
    return false;
}

int main()
{
  mouseIsDragging = mouseMove = rectangleSelected = false;

//   // Creating vector to store vectors of 3D points for each checkerboard image
//   vector<vector<Point3f> > objpoints;

//   // Creating vector to store vectors of 2D points for each checkerboard image
//   vector<vector<Point2f> > imgpoints;

//   // Defining the world coordinates for 3D points
//   vector<Point3f> objp;
//   for(int i{0}; i<CHECKERBOARD[1]; i++)
//   {
//     for(int j{0}; j<CHECKERBOARD[0]; j++)
//       objp.push_back(Point3f(j*fieldSize,i*fieldSize,0));
//   }

//   // Extracting path of individual image stored in a given directory
//   vector<String> images;
//   // Path of the folder containing checkerboard images
//   string path = "./images-1920/*.jpg";

//   glob(path, images, false);

//   Mat frame, gray;
//   // vector to store the pixel coordinates of detected checker board corners 
//   vector<Point2f> corner_pts;
//   bool success;

//   TermCriteria critia(TermCriteria::EPS | TermCriteria::MAX_ITER, 200, 0.0001);

//   // Looping over all the images in the directory
//   for(int i{0}; i<images.size(); i++)
//   {
//     frame = imread(images[i]);
//     cvtColor(frame,gray,COLOR_BGR2GRAY);

//     // imshow("gray", gray);

//     // Finding checker board corners
//     // If desired number of corners are found in the image then success = true  
//     success = findChessboardCorners(gray,Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts,  CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS | CALIB_CB_NORMALIZE_IMAGE);

//     if(success)
//     {

//       // refining pixel coordinates for given 2d points.
//       cornerSubPix(gray,corner_pts,Size(11,11), Size(-1,-1),critia);

//       // Displaying the detected corner points on the checker board
//       drawChessboardCorners(frame, Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts,success);

//       objpoints.push_back(objp);
//       imgpoints.push_back(corner_pts);
//       cout << "next .. " << endl;
//     }
//     else{
//       cout << "not detect " << endl;
//     }

//     if(gray.empty())
//       cout << "gambar tidak ada" << endl;
    

//     // imshow("Image",frame);
//     // waitKey(0);
//   }

//   destroyAllWindows();

//   Mat K, xi, D, idx;
//   int flags = 0;
//   Mat rvec, tvec;

//   cout << "kalibrasi .. " << endl;
//   double rms = omnidir::calibrate(objpoints, imgpoints, Size(gray.rows,gray.cols), K, xi, D, rvec, tvec, flags, critia, idx);

// //=================SIMPAN DATA CALIBOMNI===================//
//   FileStorage fs("calibration.yml", FileStorage::WRITE);

//   fs << "camera_matrix" << K;
//   fs << "distortion_coefficients" << D;
//   fs << "mei_coefficients" << xi;
//   fs << "rotation_vector" << rvec;
//   fs << "translation_vector" << tvec;

//   fs.release(); // Tutup file storage
// //=========================================================//
  
  //================BACA DATA CALIBOMNI===============//
  FileStorage fs("calibration.yml", FileStorage::READ);

  Mat K, xi, D, idx;
  int flags = 0;
  Mat rvec, tvec;


  fs["camera_matrix"] >> K;
  fs["distortion_coefficients"] >> D;
  fs["mei_coefficients"] >> xi;
  fs["rotation_vector"] >> rvec;
  fs["translation_vector"] >> tvec;

  fs.release();
  //==================================================//

  Mat frametest = imread("./test/imgsfield1920.jpg");
  file[ul] = '0';
  namedWindow("trackbar");
  trackbar();
  impData();
  trackBar_update();

  while(true){
    Mat undistorted;

    Size new_size;
    Matx33f Knew;
    new_size.width = 775;
    new_size.height = 775;
    Knew = Matx33f(new_size.width/20, 0, new_size.width/2,
                  0, new_size.height/20, new_size.height/2,
                  0, 0, 1);

    omnidir::undistortImage(frametest, undistorted, K, D, xi, omnidir::RECTIFY_PERSPECTIVE, Knew, new_size);

    

    Mat thresh   = GetThresImage(undistorted, undistorted);
    Mat erosion = Erosion(thresh);
    Mat dilation= Dilation(erosion);

    // Mencari piksel hijau terluar
    Point outermostGreenPixel(-1, -1); // Inisialisasi koordinat piksel hijau terluar

    // Perulangan untuk memeriksa setiap baris
    for (int y = 0; y < dilation.rows; ++y) {
        bool foundWhitePixel = false; // Flag untuk menandai apakah piksel putih sudah ditemukan pada baris ini
        Point currentWhitePixel(-1, -1); // Koordinat piksel putih saat ini

        // Perulangan untuk memeriksa setiap kolom pada baris ini
        for (int x = 0; x < dilation.cols; ++x) {
            // Ambil nilai piksel pada posisi (x, y)
            Vec3b pixel = dilation.at<Vec3b>(y, x);
            // Periksa apakah piksel putih
            if (pixel[0] == 255 && pixel[1] == 255 && pixel[2] == 255) {
              // cout << "masuk sini" << endl;
                if (!foundWhitePixel) { // Jika ini adalah piksel putih pertama yang ditemukan pada baris ini
                    currentWhitePixel = Point(x, y); // Simpan koordinatnya
                    foundWhitePixel = true; // Set flag menjadi true
                } else { // Jika ini bukan piksel putih pertama pada baris ini
                    if (currentWhitePixel.x == x - 1) { // Periksa apakah piksel putih saat ini sejajar dengan piksel putih sebelumnya
                        currentWhitePixel = Point(x, y); // Perbarui koordinat piksel putih
                    }
                }
            }
        }

      // Jika ditemukan piksel putih pada baris ini
        if (foundWhitePixel) {
            if (outermostGreenPixel.x == -1 || currentWhitePixel.y < outermostGreenPixel.y) {
                outermostGreenPixel = currentWhitePixel; // Perbarui koordinat piksel hijau terluar
            }
        }
    }

    Point center(undistorted.cols / 2, undistorted.rows / 2);

    // Temukan persamaan garis sumbu x (horizontal)
    double slopeX = 0; // Kemiringan garis sumbu x (horizontal)
    double yInterceptX = center.y; // Intersep y dari garis sumbu x

    // Temukan persamaan garis sumbu y (vertikal)
    double slopeY = 0; // Kemiringan garis sumbu y (vertikal)
    double xInterceptY = center.x; // Intersep x dari garis sumbu y

    // Gambar titik hijau terluar jika ditemukan
    if (outermostGreenPixel.x != -1 && outermostGreenPixel.y != -1 && outermostGreenPixel != center) {
      // cout << "Koordinat pixel hijau terluar: (" << outermostGreenPixel.x << ", " << outermostGreenPixel.y << ")" << endl;
      line(undistorted, center, Point( center.x, outermostGreenPixel.y), Scalar(0, 255, 0), 2); // Tandai piksel hijau terluar dengan lingkaran hijau
      line(undistorted, center, Point( outermostGreenPixel.x, center.y), Scalar(0, 255, 255), 2);

      // Hitung kemiringan garis sumbu x
      slopeX = (double)(outermostGreenPixel.y - center.y) / (outermostGreenPixel.x - center.x);
      // Hitung intersep y dari garis sumbu x
      yInterceptX = center.y - slopeX * center.x;

      // Hitung kemiringan garis sumbu y
      slopeY = (double)(outermostGreenPixel.x - center.x) / (outermostGreenPixel.y - center.y);
      // Hitung intersep x dari garis sumbu y
      xInterceptY = center.x - slopeY * center.y;

    }
    else {
      cout << "nda ketemu" << endl;
    }

    // Temukan titik perpotongan kedua garis
    double xIntersection = (yInterceptX - xInterceptY) / (slopeY - slopeX);
    double yIntersection = slopeX * xIntersection + yInterceptX;

    // Tampilkan koordinat titik perpotongan
    cout << "Koordinat titik perpotongan: (" << xIntersection << ", " << yIntersection << ")" << endl;

    // Tandai titik perpotongan
    circle(undistorted, Point(xIntersection, yIntersection), 5, Scalar(255, 0, 255), -1); // Titik perpotongan (warna magenta)

    imshow("Result", undistorted);
    imshow("Threshold", dilation);  

    char key = waitKey(1);
    if(key == 'q'){ 
      break;
    }
    else if(key == 's'){
      cout<<"Simpan data kalibrasi "<<endl;
      expData(wMin[H],wMin[S],wMin[V],wMax[H],wMax[S],wMax[V],ED[E],ED[d],ED[C]);
    }
  }
  return 0;
}