#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

class CamCalibration
{
public:
	CamCalibration();

private:
	ros::NodeHandle node;


	void calibration();
};

CamCalibration::CamCalibration()
{
	calibration();
}

void CamCalibration::calibration()
{
	ifstream fin("/home/wade/catkin_ws/src/calibration/src/calibration.txt");
	cout << "start finding corners......\n";
	int image_count = 0;
	Size image_size;
	Size board_size = Size(7, 6);
	vector<Point2f> image_points_buf;
	vector<vector<Point2f> > image_points_seq;
	string filename;
	int count = -1;

	while(getline(fin, filename, '#'))
	{
		image_count++;
		cout << "image_count = "<< image_count << endl;
		cout << "count = " << count << endl;

		Mat image_input = imread(filename);
		if (image_count == 1)
		{
			image_size.width = image_input.cols;
			image_size.height = image_input.rows;
		}
		imshow("camera calibration", image_input);
		waitKey(0);

		cvtColor(image_input, image_input, CV_RGB2GRAY);
		bool find = findChessboardCorners(image_input, board_size, image_points_buf);
		if (find == false)
		{
			cout << "cannot find chessboard corners" << endl;
			cout << filename << endl;
			break;
		} else {
			find4QuadCornerSubpix(image_input,image_points_buf,Size(11,11));
			image_points_seq.push_back(image_points_buf);
			drawChessboardCorners(image_input, board_size, image_points_buf, 1);
			imshow("camera calibration", image_input);
			waitKey(0);
		}
	}
	int total = image_points_seq.size();
	cout << "total =" << total << endl;
	int cornerNum = board_size.width*board_size.height;

	cout << "start calibrating......\n";
	
	Size square_size = Size(10,10);
	vector<vector<Point3f> > object_points;

	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));//inner parameter matrix
	vector<int> point_counts;//corner number in each image
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));//distortion factor
	vector<Mat> tvecsMat;//rotation vector
	vector<Mat> rvecsMat;//translation vector

	//initialize corners' coordinate
	for (int i = 0; i < image_count; ++i)
	{
		vector<Point3f> tempPointSet;
		for (int j = 0; j < board_size.height; ++j)
		{
			for (int k = 0; k < board_size.width; ++k)
			{
				Point3f realPoint;
				realPoint.x = j*square_size.width;
				realPoint.y = k*square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}

	//initialize corners number
	for (int i = 0; i < image_count; ++i)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}

	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	cout << "calibration completed!\n";

	/*//evaluate result
	cout << "start evaluation......\n";
	double total_err = 0.0;
	double err = 0.0;
	vector<Point2f> image_points2;
	cout << "calibration error in each image: \n";
	for (int i = 0; i < image_count; ++i)
	{
		vector<Point3f> tempPointSet = object_points[i];
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat= Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); ++j)
		{
			image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
	}*/
	string savename1 = "/home/wade/catkin_ws/src/calibration/result/Intrinsics.xml";
	string savename2 = "/home/wade/catkin_ws/src/calibration/result/Distortion.xml";
	FileStorage fs1(savename1, FileStorage::WRITE);
	FileStorage fs2(savename2, FileStorage::WRITE);
	fs1 << "C" << cameraMatrix;
	fs2 << "D" << distCoeffs;
	fs1.release();
	fs2.release();
}	

void image_repair(Mat &src, Mat &rst)
{
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
	string readname1 = "/home/wade/catkin_ws/src/calibration/result/Intrinsics.xml";
	string readname2 = "/home/wade/catkin_ws/src/calibration/result/Distortion.xml";
	FileStorage fr1(readname1, FileStorage::READ);
	FileStorage fr2(readname2, FileStorage::READ);
	fr1["C"] >> cameraMatrix;
	fr2["D"] >> distCoeffs;
	cout << "readin already\n";

	/*float intrinsic[3][3] = {6.3489813353361740e+02,0,3.0967058928663249e+02,0, 6.3703724435248523e+02, 2.3569828914282107e+02,0,0,1};
    float distortion[1][5] = {-3.9935975417762343e-01, 5.5992159384273890e-02, -5.2178242306101319e-03, 3.5016421357703434e-03, 4.6614473759649022e-01};
    Mat cameraMatrix = Mat(3,3,CV_32FC1,intrinsic);
    Mat distCoeffs = Mat(1,5,CV_32FC1,distortion);*/

	Size image_size = src.size();
	Mat R = Mat::eye(3, 3, CV_32F);
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	cout << "init already\n";
	initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
	cout << "before remap\n";
	rst = src.clone();
	remap(src, rst, mapx, mapy, INTER_LINEAR);

	/*fr1.release();
	fr2.release();*/
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "calibration");
	CamCalibration c;
	Mat img = imread("/home/wade/catkin_ws/src/calibration/src/3.jpg");
	Mat rst;
	cout << "read already!\n";
	image_repair(img, rst);
	cout << "repaired already\n";
	imshow("init", img);
	imshow("repair", rst);
	waitKey(0);
	ros::spin();
	return 0;
}