#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>

#include <fstream>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <cstring>
#include <ctime>

a
int main() {
    // Ganti path_gambar dengan path gambar yang ingin Anda unwrapped
    std::string path_gambar = "/home/jamil/build/sample5.jpg";
    // Ganti path_output dengan path tempat menyimpan gambar hasil unwrapped
    std::string path_output = "/home/jamil/build/gambar_unwrapped.jpg";

    // Baca gambar
    cv::Mat fisheye_image = cv::imread(path_gambar);

    // Tentukan parameter proyeksi equirectangular
    double radius = std::min(fisheye_image.cols, fisheye_image.rows) / (2 * M_PI);
    cv::Point2f center(fisheye_image.cols / 2.0, fisheye_image.rows / 2.0);

    // Buat gambar hasil proyeksi equirectangular
    cv::Mat unwrapped_image(fisheye_image.rows, fisheye_image.cols, fisheye_image.type());

    // Tentukan matriks Knew
    cv::Matx33f Knew(fisheye_image.cols / M_PI, 0, fisheye_image.cols / 2.0,
                     0, fisheye_image.rows / M_PI, fisheye_image.rows / 2.0,
                     0, 0, 1);

    // Buat matriks proyeksi menggunakan matriks Knew
    cv::Matx33f proj_matrix = Knew;

    // Lakukan proyeksi
    for (int y = 0; y < fisheye_image.rows; ++y) {
        for (int x = 0; x < fisheye_image.cols; ++x) {
            double theta = (x - center.x) / radius;
            double phi = (y - center.y) / radius;

            int orig_x = static_cast<int>(radius * sin(theta) + center.x);
            int orig_y = static_cast<int>(radius * sin(phi) + center.y);

            if (orig_x >= 0 && orig_x < fisheye_image.cols && orig_y >= 0 && orig_y < fisheye_image.rows) {
                unwrapped_image.at<cv::Vec3b>(y, x) = fisheye_image.at<cv::Vec3b>(orig_y, orig_x);
            }
        }
    }

    // Simpan gambar yang telah diunwrapped
    cv::imwrite(path_output, unwrapped_image);

    return 0;
}
