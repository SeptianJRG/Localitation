#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

int main() {

	// //========== 1 IMAGE =========//
	// Read an image 
	Mat img = imread("./test/imgs1920.png", -1);
	 
	// Display the image.
	imshow("grayscale image", img); 
	 
	// Wait for a keystroke.   
	waitKey(0);  
	 
	// Destroys all the windows created                         
	destroyAllWindows();
	 
	// Write the image in the same directory
	imwrite("imgsfield1920.jpg", img);

	//========== ALL IMAGES ON FOLDER ==========//
    // Path to the directory containing images
//     string directory = "images-1920/";

//     // Open the directory
//     DIR *dir;
//     struct dirent *ent;
//     if ((dir = opendir(directory.c_str())) != NULL) {
//         // Iterate over all files in the directory
//         while ((ent = readdir(dir)) != NULL) {
//             // Get the current file name
//             string file_name = ent->d_name;

//             // Skip "." and ".."
//             if (file_name == "." || file_name == "..")
//                 continue;

//             // Get the full file path
//             string file_path = directory + file_name;

//             // Read the image
//             Mat img = imread(file_path, -1);
//             if (img.empty()) {
//                 cerr << "Failed to read image: " << file_path << endl;
//                 continue; // Skip to the next file if unable to read
//             }

//             // Construct the new file path with .jpg extension
//             string new_file_path = directory + file_name.substr(0, file_name.find_last_of('.')) + ".jpg";

//             // Write the image in JPG format
//             if (!imwrite(new_file_path, img)) {
//                 cerr << "Failed to write image: " << new_file_path << endl;
//                 continue; // Skip to the next file if unable to write
//             }

//             cout << "Converted: " << file_path << " to " << new_file_path << endl;
//         }
//         closedir(dir);
//     } else {
//         // could not open directory
//         cerr << "Failed to open directory: " << directory << endl;
//         return EXIT_FAILURE;
//     }

//     cout << "Conversion complete." << endl;

//     return EXIT_SUCCESS;
}
