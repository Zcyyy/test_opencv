#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <humanoid_league_msgs/LineInformationInImage.h>
#include <humanoid_league_msgs/LineIntersectionInImage.h>
#include <humanoid_league_msgs/LineSegmentInImage.h>
#include <humanoid_league_msgs/LineCircleInImage.h>
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

class linedetection
{
	public:
	   	linedetection()
		{
			pub_ = n_.advertise<humanoid_league_msgs::LineInformationInImage>("line_pubbbbbbbbbbbbbb", 1000);
			sub_ = n_.subscribe("/camera/image_raw",1,&linedetection::imageCallback,this);
		}

		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			humanoid_league_msgs::LineSegmentInImage segment;
			humanoid_league_msgs::LineCircleInImage circlee;
			humanoid_league_msgs::LineIntersectionInImage intersection;
			std::vector<humanoid_league_msgs::LineSegmentInImage> lsegment;
		    std::vector<humanoid_league_msgs::LineCircleInImage> lcircle;
			std::vector<humanoid_league_msgs::LineIntersectionInImage> lintersection;

			cv_bridge::CvImagePtr cv_ptr;
			try
			{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
			}
			catch( cv_bridge::Exception& e)
			{
				ROS_ERROR( "cv_bridge exception: %s", e.what() );
				return;
			}
			humanoid_league_msgs::LineInformationInImage line;	
			Mat src;//,dst,gray_src;
			src = cv_ptr->image;
			imshow("111",src);
			waitKey(1000);

			Mat Bchannel(src.size(),CV_8UC1),Gchannel,Rchannel;//blue,green,red
			std::vector<Mat> channels;
			split(src, channels);
			Bchannel = channels.at(0);//.at(0)
			Gchannel = channels.at(1);
			Rchannel = channels.at(2);

			Mat fig;
			fig	= 0.9*Rchannel + 0.05*Gchannel + 0.05*Bchannel;
//			imshow("fig",fig);
//			imshow("a",Bchannel);
//			imshow("b",Gchannel);
//			imshow("c",Rchannel);
//			waitKey(1000);
			int rows = fig.rows;
			int cols = fig.cols;
			cv::Mat pro_fig = Mat::zeros(Size(rows, cols), CV_8UC1);
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
			imshow("fig",fig);
			waitKey(1000);
			lua_field_lines(fig);
			printf("after_line\n");
			ros::Rate loop_rate(10);
			int count = 0;
			//while (ros::ok())
		    //{
		        // ROS_INFO("%d",valid_segments);
				printf("ready to publish\n");				
		        for (int i=0;i<valid_segments;i++)
		        {
		            segment.start.x=useful_segments[i].x0;
		            segment.start.y=useful_segments[i].y0;
		            segment.end.x=useful_segments[i].x1;
		            segment.end.y=useful_segments[i].y1;
		            segment.confidence=1;
					lsegment.push_back(segment);
		        }
				circlee.left.x = 0.0;
				circlee.left.y = 0.0;
				circlee.right.x = 0.0;
				circlee.right.y = 0.0;
				circlee.middle.x = 0.0;
				circlee.middle.y = 0.0;
				circlee.confidence = 0.0;
				line.intersections = lintersection;
				line.segments = lsegment;
				line.circles = lcircle;
			//}
			pub_.publish(line);
		}	
	private:
		ros::NodeHandle n_;
		ros::Publisher pub_;
		ros::Subscriber sub_;
		
		int lineState(uint8_t label);

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
		void segment_init(); 
		void useful_segment_init();
	
		void segment_refresh(); 

		void segment_terminate();
		void updateStat(struct SegmentStats *statPtr, int i, int j);
		void initStat(struct SegmentStats *statPtr, int i, int j);
		//We always add pixel from left to right
		void addHorizontalPixel(int i, int j, int width); 
		//We always add pixel from top to bottom
		void addVerticalPixel(int i, int j, int width);
		//New function, find multiple connected line segments
		//Instead of one best long line
		int lua_field_lines(Mat img);
		struct SegmentStats segments[MAX_SEGMENTS];
		struct USEFUL_SegmentStats useful_segments[VALID_SEGMENTS];
		int num_segments;
};

int linedetection::lineState(uint8_t label)
		{
			printf("lineState\n");
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

void linedetection::segment_init() {
			printf("segment_init\n");
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

void linedetection::useful_segment_init()
	{
		printf("useful_segment_init\n");
		for (int i = 0; i < VALID_SEGMENTS; i++) {
			useful_segments[i].x0 = 0;
			useful_segments[i].x1 = 0;
			useful_segments[i].y0 = 0;
			useful_segments[i].y1 = 0;
		}
	}

void linedetection::segment_refresh() {
			//end active segments if they were not updated for one line scan
			printf("segment_refresh\n");
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

void linedetection::segment_terminate() {
			//end all segments
			printf("segment_terminate\n");
			for (int i = 0; i < num_segments; i++) segments[i].state = 2;
		}

void linedetection::updateStat(struct SegmentStats *statPtr, int i, int j) {
			printf("updateStat\n");
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

void linedetection::initStat(struct SegmentStats *statPtr, int i, int j) {
			printf("initStat\n");
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

void linedetection::addHorizontalPixel(int i, int j, int width) {
			//Find best matching active segment
			printf("addHorizontalPixel\n");
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

void linedetection::addVerticalPixel(int i, int j, int width) {
			//Find best matching active segment
			printf("addVerticalPixel\n");
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

int linedetection::lua_field_lines(Mat img)
		{
			printf("lua_field_lines\n");
			int nr = img.rows;
			int nc = img.cols;
			segment_init();
			//Scan for vertical line pixels:
			for (int j = 0; j < nr; j++) {
				//uint8_t *im_col = im_ptr + ni * j;
				for (int i = 0; i < nc; i++) {
					//uint8_t label = *im_col++;
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
			//Scan for horizontal field line pixels:
			for (int i = 0; i < nc; i++) {
				//uint8_t *im_row = im_ptr + i;
				for (int j = 0; j < nr; j++) {
					//uint8_t label = *im_row;
					//im_row += ni;
					uint8_t label = img.at<uint8_t>(j, i);
					int width = lineState(label);
					if ((width >= widthMin) && (width <= widthMax)) {
						int jline = j - (width + 1) / 2;
						addHorizontalPixel(i, jline, width);
					}
				}
				segment_refresh();
			}
			useful_segment_init();
			for(int k = 0; k < num_segments; k++) {
				int dx = segments[k].x1 - segments[k].x0;
				int dy = segments[k].y1 - segments[k].y0;
				segments[k].length = sqrt(dx*dx + dy * dy);
				if (segments[k].count > min_length) {
					useful_segments[valid_segments].x0 = segments[k].x0;
					useful_segments[valid_segments].x1 = segments[k].x1;
					useful_segments[valid_segments].y0 = segments[k].y0;
					useful_segments[valid_segments].y1 = segments[k].y1;
					valid_segments++;
					std::cout  << segments[k].count << std::endl;
				}
			}
			printf("exit lua_line\n");
			return 0;
		}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"pub_sub");
	linedetection line;
	ros::spin();
	return 0;
}
