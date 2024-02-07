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

using namespace cv;
using namespace std;

// Parameter particle filter
const int nParticles = 100;
const double sigmaX = 10;
const double sigmaY = 10;

// Fungsi untuk menghitung likelihood
double likelihood(Mat image, Point2f particle) {
  // Hitung nilai pixel hitam di sekitar partikel
  int blackPixels = countNonZero(image(Rect(particle.x - sigmaX, particle.y - sigmaY, 2 * sigmaX, 2 * sigmaY)));

  // Hitung likelihood
  return blackPixels / (2 * sigmaX * sigmaY);
}

// Fungsi untuk melakukan sampling
Point2f sample(Mat image) {
  // Buat partikel baru
  Point2f particle;
  particle.x = random() % image.cols;
  particle.y = random() % image.rows;

  // Pastikan partikel berada di dalam gambar
  while (particle.x < 0 || particle.x >= image.cols || particle.y < 0 || particle.y >= image.rows) {
    particle.x = random() % image.cols;
    particle.y = random() % image.rows;
  }

  return particle;
}

struct Particle{
    Point_<float> position;
    float weight;
};


// Fungsi untuk melakukan weight update
void weightUpdate(Mat image, vector<Particle> particles) {
  // Hitung likelihood untuk setiap partikel
  for (int i = 0; i < particles.size(); i++) {
    particles[i].weight = likelihood(image, particles[i].position);
  }
}

// Fungsi untuk melakukan resampling
void resampling(vector<Particle> &particles) {
  // Urutkan partikel berdasarkan weight
  sort(particles.begin(), particles.end(), [](Point2f a, Point2f b) { return a.weight > b.weight; });

  // Buat vector baru untuk menyimpan partikel baru
  vector<Point2f> newParticles;

  // Ambil partikel dengan weight tertinggi
  newParticles.push_back(particles[0]);

  // Ambil partikel dengan weight tertinggi secara acak
  for (int i = 1; i < nParticles; i++) {
    int index = rand() % (i + 1);
    newParticles.push_back(particles[index]);
  }

  // Ganti partikel lama dengan partikel baru
  particles = newParticles;
}

int main() {
  // Load gambar
  Mat image = imread("Goal.jpg", IMREAD_GRAYSCALE);

  // Buat vector untuk menyimpan partikel
  vector<Particle> particles;
  for (int i = 0; i < nParticles; i++) {
    particles.push_back(sample(image));
  }

  // Iterasi particle filter
  for (int i = 0; i < 100; i++) {
    // Hitung likelihood untuk setiap partikel
    weightUpdate(image, particles);

    // Lakukan resampling
    resampling(particles);
  }

  // Temukan partikel dengan weight tertinggi
  Point2f goal = particles[0];

  // Buat rectangle untuk gawang
  Rect goalRect(goal.x - sigmaX, goal.y - sigmaY, 2 * sigmaX, 2 * sigmaY);

  // Tampilkan gambar dengan rectangle
  imshow("Image", image);
  rectangle(image, goalRect, Scalar(0, 255, 0), 2);
  waitKey(0);

  return 0;
}
