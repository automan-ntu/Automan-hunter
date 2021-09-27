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
#include <opencv2/calib3d.hpp>

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

static const std::string SDWINDOW = "Disparity";

class denseScene
{
public:
	denseScene();
    
	void disparityCb(const sensor_msgs::ImageConstPtr& img_1_msg, const sensor_msgs::ImageConstPtr& img_2_msg);
	~denseScene();
private:    
	//define nodehandle and subscriber
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> img_1_sub_;
    message_filters::Subscriber<sensor_msgs::Image> img_2_sub_;
	ros::Subscriber bbx_sub_;
    //synchronize rgb image and depth
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync;

	//Intrinsic matrix parameters
	double fx_ = 612.9436645507812;
	double cx_ = 331.5249328613281;
	double fy_ = 612.93798828125;
	double cy_ = 244.83120727539062;

	//sgbm parameters
	int minDisparity_ = 0;
	int numDisparities_ = 128;
	int SADWindowSize_ = 5;
	int P1_ = 600;
	int P2_ = 2400;
	int disp12MaxDiff_ = 10;
	int preFilterCap_ = 16;
	int uniquenessRatio_ = 2;
	int speckleWindowSize_ = 20;
	int speckleRange_ = 30;

};

denseScene::denseScene():
img_1_sub_(nh_, "/d435_front/infra1/image_rect_raw", 10),
img_2_sub_(nh_, "/d435_front/infra2/image_rect_raw", 10),
sync(sync_pol(10), img_1_sub_, img_2_sub_)
{
    nh_.param("minDisparity", minDisparity_, minDisparity_);
    nh_.param("numDisparities", numDisparities_, numDisparities_);
	nh_.param("SADWindowSize", SADWindowSize_, SADWindowSize_);
	nh_.param("P1", P1_, P1_);
	nh_.param("P2", P2_, P2_);
	nh_.param("disp12MaxDiff", disp12MaxDiff_, disp12MaxDiff_);
	nh_.param("preFilterCap", preFilterCap_, preFilterCap_);
	nh_.param("uniquenessRatio", uniquenessRatio_, uniquenessRatio_);
	nh_.param("speckleWindowSize", speckleWindowSize_, speckleWindowSize_);
	nh_.param("speckleRange", speckleRange_, speckleRange_);

    cv::namedWindow(SDWINDOW);
    sync.registerCallback(boost::bind(&denseScene::disparityCb,this,_1,_2));
	cout << "Social Distance Initialization!" << endl;
};

denseScene::~denseScene()
{
    cv::destroyWindow(SDWINDOW);
	cout << "Exit" << endl;
};

void denseScene::disparityCb(const sensor_msgs::ImageConstPtr& img_1_msg, const sensor_msgs::ImageConstPtr& img_2_msg)
{   
    //Image 
    cv_bridge::CvImagePtr cv_1_ptr, cv_2_ptr;
    try
    {
      cv_1_ptr = cv_bridge::toCvCopy(img_1_msg, sensor_msgs::image_encodings::MONO8);
	  cv_2_ptr = cv_bridge::toCvCopy(img_2_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	cv::Mat img_1 = cv_1_ptr->image;
	cv::Mat img_2 = cv_2_ptr->image;
	cv::Mat disparity,disparityNorm;
	cv::GaussianBlur(img_1, img_1, cv::Size(3,3), 0, 0);
	cv::GaussianBlur(img_2, img_2, cv::Size(3,3), 0, 0);
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisparity_,    //int minDisparity
                                    numDisparities_,     //int numDisparities
                                    SADWindowSize_,      //int SADWindowSize
                                    P1_,    //int P1 = 0
                                    P2_,   //int P2 = 0
                                    disp12MaxDiff_,     //int disp12MaxDiff = 0
                                    preFilterCap_,     //int preFilterCap = 0
                                    uniquenessRatio_,      //int uniquenessRatio = 0
                                    speckleWindowSize_,    //int speckleWindowSize = 0
                                    speckleRange_,     //int speckleRange = 0
                                    true);  //bool fullDP = false();

	sgbm->compute(img_1, img_2, disparity);
	cv::normalize(disparity, disparityNorm, 0, 255, CV_MINMAX, CV_8U);
	disparityNorm.convertTo(disparityNorm, CV_32F, 1.0/16.0);
	cv::imshow(SDWINDOW, disparityNorm);
	cv::waitKey(10);
};


int main(int argc, char** argv)
{
   ros::init(argc, argv,"social_distance");
   cout << "Hello Automan!" << endl;
   denseScene ds;
   ros::Rate rate(30);
   while( ros::ok() )
   {
       ros::spinOnce();
		rate.sleep();
       //cout<< "it is ok" << endl;
   }
    
   return 0;
};

