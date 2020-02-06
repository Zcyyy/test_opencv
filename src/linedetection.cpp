// Incremental.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "opencv2/imgproc/types_c.h"
#include <humanoid_league_msgs/LineInformationInImage.h>
//#include <opencv2/imgproc/types_c.h>
using namespace cv;
using namespace std;
//set parameter 
#define MAX_SEGMENTS 2000
#define VALID_SEGMENTS 200
#define widthMin  1
#define widthMax  10
#define connect_th 5
#define max_gap 10
#define min_length 10
int valid_segments;

int lineState(uint8_t label)
{
	enum { STATE_NONE, STATE_FIELD, STATE_LINE };
	static int state = STATE_NONE;
	static int width = 0;
	switch (state) {
	case STATE_NONE:
		if (label == 128)
			state = STATE_FIELD;
		break;
	case STATE_FIELD:
		if (label == 255) {
			state = STATE_LINE;
			width = 1;
		}
		else if (label != 128) {
			state = STATE_NONE;
		}
		break;
	case STATE_LINE:
		if (label == 128) {
			state = STATE_FIELD;
			return width;
		}
		else if (label != 255) {
			state = STATE_NONE;
		}
		else {
			width++;
		}
	}
	return 0;
}

struct SegmentStats {
	int state; //0 for inactive, 1 for active, 2 for ended
	int gap; //gap handling
	int count; //the horizontal length of the line
	int x0, y0; //start point
	int x1, y1; //end point
	double xMean;
	double yMean;
	double grad;//gradient, dy/dx
	double invgrad; //inv gradient, dx/dy
	int x, y, xy, xx, yy; //for gradient stats
	int updated;
	int length;
	int max_width;
};

struct USEFUL_SegmentStats {
	int x0, y0; //start point
	int x1, y1; //end point
};

static struct SegmentStats segments[MAX_SEGMENTS];
static struct USEFUL_SegmentStats useful_segments[VALID_SEGMENTS];
static int num_segments;

void segment_init() {
	for (int i = 0; i < MAX_SEGMENTS; i++) {
		segments[i].state = 0;
		segments[i].gap = 0;
		segments[i].count = 0;
		segments[i].xy = 0;
		segments[i].xx = 0;
		segments[i].x = 0;
		segments[i].y = 0;
		segments[i].updated = 0;
		segments[i].grad = 0;
		segments[i].max_width = 0;
	}
	num_segments = 0;
}

void useful_segment_init()
{
	for (int i = 0; i < VALID_SEGMENTS; i++) {
		useful_segments[i].x0 = 0;
		useful_segments[i].x1 = 0;
		useful_segments[i].y0 = 0;
		useful_segments[i].y1 = 0;
	}
}

void segment_refresh() {
	//end active segments if they were not updated for one line scan
	for (int i = 0; i < num_segments; i++) {
		if ((segments[i].state == 1) && (segments[i].updated == 0)) {
			if (segments[i].gap > 0)
				segments[i].gap--;
			else
				segments[i].state = 2;
		}
		segments[i].updated = 0;
	}
}


void segment_terminate() {
	//end all segments
	for (int i = 0; i < num_segments; i++) segments[i].state = 2;
}



void updateStat(struct SegmentStats *statPtr, int i, int j) {

	struct SegmentStats &stat = *statPtr;
	stat.xMean = (stat.count*stat.xMean + i) / (stat.count + 1);
	stat.yMean = (stat.count*stat.yMean + j) / (stat.count + 1);
	stat.x = stat.x + i;
	stat.y = stat.y + j;
	stat.xy = stat.xy + i * j;
	stat.xx = stat.xx + i * i;
	stat.yy = stat.yy + j * j;
	stat.gap++;
	if (stat.gap > max_gap + 1) stat.gap = max_gap + 1;
}



void initStat(struct SegmentStats *statPtr, int i, int j) {

	struct SegmentStats &stat = *statPtr;
	stat.state = 1;
	stat.count = 1;
	stat.gap = 1;
	stat.grad = 0;
	stat.x0 = i;
	stat.y0 = j;
	stat.x = i;
	stat.y = j;
	stat.xx = i * i;
	stat.xy = i * j;
	stat.yy = j * j;
	stat.xMean = i;
	stat.yMean = j;
	stat.updated = 1;
}





//We always add pixel from left to right

void addHorizontalPixel(int i, int j, int width) {
	//Find best matching active segment
	int seg_updated = 0;
	//printf("Checking pixel %d,%d\n",i,j);
	for (int k = 0; k < num_segments; k++) {
		if (segments[k].state == 1) {
			double yProj = segments[k].yMean + segments[k].grad*(i - segments[k].xMean);
			double yErr = j - yProj; if (yErr < 0) yErr = -yErr;
			//printf("Checking segment %d\n",k);

			//printf("xmean %.1f, ymean %.1f, grad %.2f, yErr %.2f\n", 

			//segments[k].xMean, segments[k].yMean, segments[k].grad, yErr);

			if (yErr < connect_th) {
				updateStat(&segments[k], i, j);
				segments[k].count++;
				segments[k].grad = (double)
					(segments[k].xy - segments[k].x*segments[k].y / segments[k].count)
					/ (segments[k].xx - segments[k].x*segments[k].x / segments[k].count);
				if ((segments[k].grad > 1.0) || (segments[k].grad < -1.0)) {
					segments[k].state = 2; //kill anything that exceeds 45 degree
					//segments[k].count = 0;
				}
				segments[k].updated = 1;
				segments[k].x1 = i;
				segments[k].y1 = j;
				if (width > segments[k].max_width) {
					segments[k].max_width = width;
				}
				seg_updated = seg_updated + 1;
			}
		}
	}

	if ((seg_updated == 0) && (num_segments < MAX_SEGMENTS)) {
		//printf("New segment %d at %d,%d\n",num_segments,i,j);
		initStat(&segments[num_segments], i, j);
		num_segments++;
	}
}

//We always add pixel from top to bottom

void addVerticalPixel(int i, int j, int width) {
	//Find best matching active segment
	int seg_updated = 0;
	for (int k = 0; k < num_segments; k++) {
		if (segments[k].state == 1) {
			double xProj = segments[k].xMean + segments[k].invgrad*(j - segments[k].yMean);
			double xErr = i - xProj; if (xErr < 0) xErr = -xErr;
			if (xErr < connect_th) {
				updateStat(&segments[k], i, j);
				segments[k].count++;
				segments[k].invgrad = (double)
					(segments[k].xy - segments[k].x*segments[k].y / segments[k].count)
					/ (segments[k].yy - segments[k].y*segments[k].y / segments[k].count);
				if ((segments[k].invgrad > 1.0) || (segments[k].invgrad < -1.0)) {
					segments[k].state = 2; //kill anything that exceeds 45 degree
					//segments[k].count = 0;
				}
				segments[k].updated = 1;
				segments[k].x1 = i;
				segments[k].y1 = j;
				seg_updated = seg_updated + 1;
				if (width > segments[k].max_width) {
					segments[k].max_width = width;
				}
			}
		}
	}
	if ((seg_updated == 0) && (num_segments < MAX_SEGMENTS)) {
		//printf("New segment %dstart:%d,%d\n",num_segments,i,j);
		initStat(&segments[num_segments], i, j);
		num_segments++;
	}
}


//New function, find multiple connected line segments

//Instead of one best long line



int lua_field_lines(Mat img)
{
	int nr = img.rows;
	int nc = img.cols;
	segment_init();
	// Scan for vertical line pixels:
	for (int j = 0; j < nr; j++) {
		//	uint8_t *im_col = im_ptr + ni * j;
		for (int i = 0; i < nc; i++) {
			//			uint8_t label = *im_col++;
			uint8_t label = img.at<uint8_t>(j, i);
			int width = lineState(label);
			if ((width >= widthMin) && (width <= widthMax)) {
				int iline = i - (width + 1) / 2;
				addVerticalPixel(iline, j, width);
			}
		}
		segment_refresh();
	}
	segment_terminate();
	// Scan for horizontal field line pixels:
	for (int i = 0; i < nc; i++) {
		//		uint8_t *im_row = im_ptr + i;
		for (int j = 0; j < nr; j++) {
			//			uint8_t label = *im_row;
			//			im_row += ni;
			uint8_t label = img.at<uint8_t>(j, i);
			int width = lineState(label);
			if ((width >= widthMin) && (width <= widthMax)) {
				int jline = j - (width + 1) / 2;
				addHorizontalPixel(i, jline, width);
			}
		}
		segment_refresh();
	}


	for (int k = 0; k < num_segments; k++) {
		int dx = segments[k].x1 - segments[k].x0;
		int dy = segments[k].y1 - segments[k].y0;
		segments[k].length = sqrt(dx*dx + dy * dy);
		if (segments[k].count > min_length) {
			useful_segments[valid_segments].x0 = segments[k].x0;
			useful_segments[valid_segments].x1 = segments[k].x1;
			useful_segments[valid_segments].y0 = segments[k].y0;
			useful_segments[valid_segments].y1 = segments[k].y1;
			valid_segments++;
			//std::cout <<"count"<< segments[k].count;
		}
	}
	return 0;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh ;
	image_transport::ImageTransport it(nh);
    Mat src = imread("~/Documents/newbots/src/bitbots_meta/bitbots_vision/test_opencv/SimImg/frame0000.jpg", cv::IMREAD_UNCHANGED);
	//imshow("测试窗口", src);
	Mat dst, gray_src;
	cvtColor(src, src, CV_BGR2GRAY);
	cvtColor(src, dst, CV_GRAY2BGR);
	Canny(src, gray_src, 150, 200);
	//cv::Smooth(gray_src, gray_src);
	/*imshow("image", gray_src);
	clock_t start_time, end_time;
	start_time = clock();*/
	//split channel
	
	Mat Bluechannel,Greenchannel,Redchannel;
	std::vector <Mat> channels;
	split(src, channels);
	Bluechannel = channels.at(0);
	Greenchannel = channels.at(1);
	Redchannel = channels.at(2);
	cv::Mat fig;
	fig = 0.05*Redchannel + 0.05*Greenchannel + 0.9*Bluechannel;
	//cv::imshow("After processing", fig);
	int rows = fig.rows;
	int cols = fig.cols;
	cv::Mat pro_fig = Mat::zeros(Size(rows, cols), CV_8UC1);
	//vector<Vec4f> plines;
	//HoughLinesP(gray_src, plines, 1, CV_PI / 180, 10, 0.0, 10);
	for (int y = 0; y < fig.rows; y++)
	{
		for (int x = 0; x < fig.cols; x++)
		{
			uint8_t data = fig.at<uint8_t>(y, x);
			//std::cout<<data;
			if (data < 60)
				fig.at<uint8_t>(y, x) = 0;
			else if ((data>=60)&&(data<190))
				fig.at<uint8_t>(y, x) = 128;
			else
				fig.at<uint8_t>(y, x) = 255;
		}
	}
	//cv::imshow("Finally",fig);
	/*ofstream outfile("test.txt", std::ios::app);
	for (int i = 0; i < fig.rows; i++)

	{

		for (int j = 0; j < fig.cols; j++)

		{
			uint8_t k = fig.at<uint8_t>(i, j);
			outfile << (int)k << "\t";

		}

		outfile << std::endl;

	}
	outfile.close();*/
	lua_field_lines(fig);
	//end_time = clock();//计时结束
	//std::cout << "The run time is: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;
	/*Scalar color = Scalar(0, 0, 255);
	Mat img = Mat::ones(Size(2000, 800), CV_8UC1);
	img = img * 255;
	//for (size_t i = 0; i < valid_segments; i++)
	for (size_t i = 0; i < valid_segments; i++)
	{
		//line(dst, Point(hline[0], hline[1]), Point(hline[2], hline[3]), color, 3, LINE_AA); 
		cv::line(img, cv::Point(useful_segments[i].x0, useful_segments[i].y0), cv::Point(useful_segments[i].x1, useful_segments[i].y1), color, 1, LINE_AA);

	}*/
	//cv::imshow("image2", img);
	//system("pause");
	//cv::waitKey(0);
    ros::Publisher chatter_pub = nh.advertise<humanoid_league_msgs::LineInformationInImage>("line_pub", 1000);
    ros::Rate loop_rate(10);
    int count=0;
    while (ros::ok())
    {
        humanoid_league_msgs::LineInformationInImage msg;
       // ROS_INFO("%d",valid_segments);
        msg.segments.resize(valid_segments);
        for (int i=0;i<valid_segments;i++)
        {
            msg.segments[i].start.x=useful_segments[i].x0;
            msg.segments[i].start.y=useful_segments[i].y0;
            msg.segments[i].end.x=useful_segments[i].x1;
            msg.segments[i].end.y=useful_segments[i].y1;
            msg.segments[i].confidence=i+1;
        }
        /*msg.header.stamp=ros::Time::now();
        ROS_INFO("header ok");
        msg.segments.resize(1);
        msg.segments[0].start.x=1.0;
        ROS_INFO("HELLO");
        msg.segments[0].start.y=1.0;
        msg.segments[0].end.x=2.0;
        msg.segments[0].end.y=2.0;*/
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
	return 0;
}

