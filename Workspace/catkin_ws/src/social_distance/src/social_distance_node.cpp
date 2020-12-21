#include <iostream>
//ros header
#include <ros/ros.h>
//pcl 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
//opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//bbx msg
//#include "/robot/ws_robot/devel/include/darknet_ros_msgs/BoundingBoxes.h"
//#include "/robot/ws_robot/devel/include/darknet_ros_msgs/BoundingBoxe.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
//multi msg process header
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//std headers
#include <limits>
#include <algorithm>
#include <vector>
#include <cmath>
#include <string>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

static const std::string SDWINDOW = "Social Distance";

class socialDistance
{
public:
	socialDistance();
    
	void socialDistanceCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& img_msg);
    void bbxCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbx_msg);
	~socialDistance();
private:    
	//define nodehandle and subscriber
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::Image> img_sub_;
	ros::Subscriber bbx_sub_;
    //synchronize rgb image and depth
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync;
    
    //parameters
    double minimum_probability_ = 0.5f;
    double minimum_thereshhold_ = 0.2f;
	float scale_ = 0.001f;

	//Intrinsic matrix parameters
	double fx_ = 612.9436645507812;
	double cx_ = 331.5249328613281;
	double fy_ = 612.93798828125;
	double cy_ = 244.83120727539062;
	
	std::vector<darknet_ros_msgs::BoundingBox> bboxes_;

};

socialDistance::socialDistance():
depth_sub_(nh_, "/d435_front/aligned_depth_to_color/image_raw", 10),
img_sub_(nh_, "/d435_front/color/image_raw", 10),
sync(sync_pol(10), depth_sub_, img_sub_)
{
    nh_.param("minimum_probability", minimum_probability_, minimum_probability_);
    nh_.param("minimum_thereshhold", minimum_thereshhold_, minimum_thereshhold_);
    cv::namedWindow(SDWINDOW);
    sync.registerCallback(boost::bind(&socialDistance::socialDistanceCb,this,_1,_2));
	bbx_sub_ = nh_.subscribe("darknet_ros/bounding_boxes", 10, &socialDistance::bbxCb, this);
	cout << "Social Distance Initialization!" << endl;
};

socialDistance::~socialDistance()
{
    cv::destroyWindow(SDWINDOW);
	cout << "Exit" << endl;
};

void socialDistance::socialDistanceCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& img_msg)
{   
    //Image 
    cv_bridge::CvImagePtr cv_ptr, depth_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
	  depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	cv::Mat depth = depth_ptr->image;
	cv::Mat color = cv_ptr->image;

    std::vector<std::vector<double>> pt_drawer;

    for (auto bbx: bboxes_)
    {
		//bounding boxes filtering
        if((bbx.probability < minimum_probability_) || (bbx.Class != "person"))
        {
            continue;
        }
        
        float c_x, c_y, c_z;
        c_x = (bbx.xmax + bbx.xmin)/2;
        c_y = (bbx.ymax + bbx.ymin)/2;
		c_z = scale_ * depth.at<u_int16_t>(c_y,c_x);
		
		std::vector<float> z;
        float maxz, minz;
        //cv::rectangle(color, cv::Point(bbx.xmin, bbx.ymin), cv::Point(bbx.xmax, bbx.ymax), cv::Scalar(0,0,0));
        for (int i = bbx.xmin; i < bbx.xmax; i++)
            for (int j = bbx.ymin; j < bbx.ymax; j++)
            {
                float distance = scale_ * depth.at<u_int16_t>(j,i);
                if (std::isnan(distance))
                    continue;
                if (fabs(distance - c_z) < minimum_thereshhold_)
					z.push_back(distance);
            }
		if (z.size() < 2)
			continue;
        maxz = *max_element(z.begin(), z.end());
		minz = *min_element(z.begin(), z.end());
		cout << bbx.Class << ": " << "x: " << ((maxz+minz)/2) * (c_x-cx_)/fx_ << " y: " << ((maxz+minz)/2) * (c_y-cy_)/fy_ <<" z: " << (maxz+minz)/2 <<  endl;
        pt_drawer.push_back({((maxz+minz)/2) * (c_x-cx_)/fx_, ((maxz+minz)/2) * (c_y-cy_)/fy_, (maxz + minz)/2, bbx.xmax, bbx.xmin, bbx.ymax, bbx.ymin});
        
    }
    std::sort(pt_drawer.begin(), pt_drawer.end());

	ROS_INFO("Frame Counting");

    if (pt_drawer.size()>1)
	{
    //draw circles
		for(int i = 0; i < pt_drawer.size()-1; i++)
		{
		    //cv::line(cv_ptr->image, cv::Point(pt_drawer[i][0], pt_drawer[i][1]), cv::Point(pt_drawer[i+1][0], pt_drawer[i+1][1]), cv::Scalar(0,0,255), 1);
			cv::Scalar g = cv::Scalar(0,0,255);
			cv::Scalar r = cv::Scalar(0,255,0);
		    double sd = sqrt(pow(pt_drawer[i][0]-pt_drawer[i+1][0], 2) + pow(pt_drawer[i][1]-pt_drawer[i+1][1], 2) + pow(pt_drawer[i][2]-pt_drawer[i+1][2], 2));
			cout << "Distance: " << sd << endl;
			if (sd < 1.0f){
				cv::ellipse(color, cv::Point((pt_drawer[i][3]+pt_drawer[i][4])/2, pt_drawer[i][5]), cv::Size((pt_drawer[i][5]-pt_drawer[i][6])/2, (pt_drawer[i][3]-pt_drawer[i][4])/8),0,0,360,g,2);
				cv::ellipse(color, cv::Point((pt_drawer[i+1][3]+pt_drawer[i+1][4])/2, pt_drawer[i+1][5]), cv::Size((pt_drawer[i+1][5]-pt_drawer[i+1][6])/2, (pt_drawer[i+1][3]-pt_drawer[i+1][4])/8),0,0,360,g,2);}
			else {
				cv::ellipse(color, cv::Point((pt_drawer[i][3]+pt_drawer[i][4])/2, pt_drawer[i][5]), cv::Size((pt_drawer[i][5]-pt_drawer[i][6])/2, (pt_drawer[i][3]-pt_drawer[i][4])/8),0,0,360,r,2);
				cv::ellipse(color, cv::Point((pt_drawer[i+1][3]+pt_drawer[i+1][4])/2, pt_drawer[i+1][5]), cv::Size((pt_drawer[i+1][5]-pt_drawer[i+1][6])/2, (pt_drawer[i+1][3]-pt_drawer[i+1][4])/8),0,0,360,r,2);}
		}
	}                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	
	cv::imshow(SDWINDOW, color);
	cv::waitKey(10);
};

void socialDistance::bbxCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbx_msg)
{
	bboxes_ = bbx_msg->bounding_boxes;
	//ROS_INFO("BoundingBoxes Callback is Working!");
};

int main(int argc, char** argv)
{
   ros::init(argc, argv,"social_distance");
   cout << "Hello Automan!" << endl;
   socialDistance sd;
   ros::Rate rate(30);
   while( ros::ok() )
   {
       ros::spinOnce();
		rate.sleep();
       //cout<< "it is ok" << endl;
   }
    
   return 1;
};

