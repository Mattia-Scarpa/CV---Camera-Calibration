// Lab 2: 26/03/2021 - Camera calibration

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;



int main(int argc, char const **argv) {

	// Importing all images path in a vector
	vector<Mat> img;
	vector<String> img_path;
	glob("../data/checkerboard_images/*.png", img_path, false);
	size_t img_count = img_path.size();
	cout << "A total of " << img_count << " images has been found!" << endl;

	// defining the vector of 2D corners coordinates for each and all images
	vector<vector<Point2f>> imgCornersPts;
	vector<Point2f> corners;
	// number of conrer in the checher board
	Size patternsize(5,6);

	vector<Point3f> objCorners;
	vector<vector<Point3f>> objCornersPoints;
	for (int i(0); i < patternsize.height; i++) {
		for (int j(0); j < patternsize.width; j++) {
			objCorners.push_back(Point3f(j*0.11,i*0.11,0));
		}
	}

	for (size_t i = 0; i < img_count; i++) {
		cout << "Looking for chessboard in image " << i+1 << " of " << img_count << "...";
		img.push_back(imread(img_path[i], IMREAD_GRAYSCALE));
		bool patternfound = findChessboardCorners(img[i], patternsize, corners);
		if (patternfound) {
			// Refining corners pixel coordinates
			cornerSubPix(img[i], corners, Size(11,11), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.0001));

			imgCornersPts.push_back(corners);
			objCornersPoints.push_back(objCorners);

			cout << " success!" << endl;
		}
		else {
			cout << "fail!" << endl;
		}
	}

	Mat cameraMatrix, distCoeffs, stdInt, stdExt, perViewErrors;
	vector<Mat> Rv, Tv;

	cout << "Calibrating camera..." << endl;
	calibrateCamera(objCornersPoints, imgCornersPts, Size(img[0].cols, img[0].rows), cameraMatrix, distCoeffs, Rv, Tv, stdInt, stdExt, perViewErrors);
	cout << "Done!" << endl;

	cout << "cameraMatrix : \n" << cameraMatrix << endl;
	cout << "distCoeffs : \n" << distCoeffs << endl;
	cout << "Rotation vector : \n" << Rv[0] << endl;
	cout << "Translation vector : \n" << Tv[0] << endl;
	//cout << "SD Intrinsic : \n" << stdInt << endl;
	//cout << "SD Extrinsic : \n" << stdExt << endl;
	//cout << "RMS reprojection errors : \n" << perViewErrors << endl;

	vector<Point2f> projPoints;
	vector<double> ProjError;
	for (int i(0); i < img_count; i++) {
		projectPoints(objCorners, Rv[i], Tv[i], cameraMatrix, distCoeffs, projPoints);
		ProjError.push_back(norm(projPoints, imgCornersPts[i]));
	}

	int best_image = min_element(ProjError.begin(),ProjError.end()) - ProjError.begin();
	int worst_image = max_element(ProjError.begin(),ProjError.end()) - ProjError.begin();

	cout << "The best image is located at : " << img_path[best_image] << endl;
	cout << "The worst image is located at : " << img_path[worst_image] << endl;

	Mat distortedImg = imread("../data/test_image.png");
	Mat undistortedImg;

	cout << "Undistorting image... " << endl;

	Mat map1, map2;
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, Size(distortedImg.cols, distortedImg.rows), CV_32FC1, map1, map2);
	remap(distortedImg, undistortedImg, map1, map2, INTER_CUBIC);

	namedWindow("Original", WINDOW_NORMAL);
	imshow("Original", distortedImg);
	namedWindow("Undistorted", WINDOW_NORMAL);
	imshow("Undistorted",undistortedImg);
	imwrite("../data/undistorted_image.png", undistortedImg);

	cout << "Press any key to quit!" << endl;
	waitKey(0);

  return 0;
}
