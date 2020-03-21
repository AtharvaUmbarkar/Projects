 #include <ros/ros.h>
 #include <image_transport/image_transport.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <geometry_msgs/Point.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <nav_msgs/Odometry.h>
 #include <geometry_msgs/TwistStamped.h>
 
 static const std::string OPENCV_WINDOW = "Image window";
 cv::Mat src,dst;
 float za=0,ya=0,ox,oy,oz,fy,fz;
 float area=1800,a=0,flag2=0;
 
 class ImageConverter
 {
     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_;
     image_transport::Publisher image_pub_;
   
   public:
     ImageConverter()
       : it_(nh_)
     {
       // Subscrive to input video feed and publish output video feed
       image_sub_ = it_.subscribe("/iris/usb_cam/image_raw", 1,&ImageConverter::imageCb, this);
       image_pub_ = it_.advertise("/image_converter/output_video", 1);
       cv::namedWindow(OPENCV_WINDOW);
     }
   
      ~ImageConverter()
     {
       cv::destroyWindow(OPENCV_WINDOW);
     }
   
     void imageCb(const sensor_msgs::ImageConstPtr& msg)
     {
       cv_bridge::CvImagePtr cv_ptr;
       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         src=cv_ptr->image;
         return;
       }

       src=cv_ptr->image;
      
       cv::Mat src_cnr,src_gray,src_canny,src_hsv,src_thresh;

       cv::cvtColor(src,src_hsv,CV_BGR2HSV);
    
       cv::inRange(src,cv::Scalar(30,0,60),cv::Scalar(60,50,255),src_thresh);

       cv::Canny( src_thresh, src_canny, 0, 255, 3 );
    
       std::vector<std::vector<cv::Point> > contours;

       cv::findContours(src_thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
       for(int i=0; i<contours.size(); ++i){
           cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);
   
           cv::Point2f rect_points[4]; 
           rotatedRect.points( rect_points );
           area=rotatedRect.size.height*rotatedRect.size.width;
           
           
           cv::Point2f center = rotatedRect.center;
           ya=center.x; za=center.y;
           cv::circle(src, center, 5, cv::Scalar(255,255,255));
            
            
           
           
       }

      
       cv::imshow(OPENCV_WINDOW,src);
       cv::imshow("out",src_thresh);
       cv::waitKey(3);
       cv::waitKey(3);

       sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
       
       image_pub_.publish(thresh_msg);
     }
   };

   int count = 0;
   float rt = 0;
   
   void odom_cb(nav_msgs::Odometry o){
     ox = o.pose.pose.position.x;
     oy = o.pose.pose.position.y;
     oz = o.pose.pose.position.z;
   }

   int main(int argc, char** argv)
   {
     ros::init(argc, argv, "image_converter");
     ros::NodeHandle nh;
     ImageConverter ic;
     ros::Publisher center_pub = nh.advertise<geometry_msgs::Point>("center1",10);
     ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("custom",10);
     ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",10);
     ros::Subscriber odom_sub = nh.subscribe("/mavros/global_position/local",10,odom_cb);
     ros::Rate looprate(20);
     geometry_msgs::TwistStamped v;
     v.twist.linear.x=-0.3;

     while(ros::ok()){
     geometry_msgs::Point p; p.x=ya; p.y=1; p.z=area;
     center_pub.publish(p);
    //  vel_pub.publish(v);
     if(count%60==0) rt+=1;
     geometry_msgs::PoseStamped t;
     
     if(area <= (9000) && flag2 == 0)
     {
       t.pose.position.z = oz+((120-za)*0.0447916);
       t.pose.position.y = oy+((ya-160)*0.0453125);
       t.pose.position.x = -rt;
       t.pose.orientation.z=160;
      //  t.pose.orientation.y=0;
       if(ya > 140 && ya < 180) fy=oy;
       if(za > 100 && za < 140) fz=oz;
       pose_pub.publish(t);
     }
     else
     {
       t.pose.position.z = fz;
       t.pose.position.y = fy;
       t.pose.position.x = -rt;
       t.pose.orientation.z=160;
      //  t.pose.orientation.y=0;
       pose_pub.publish(t);
       flag2 = 1;
     }
     
     ros::spinOnce();
     looprate.sleep();
     count++;
     }
     return 0;}

// NOTE:
// for image scaling: in x = 3mm on camera -> 0.75 units in gazebo;
//                           1mm on camera -> 0.25 units
//                           size of camera fov -> 58mm                  
//                           fov of camera = 14.5 units
//                           lambda(x) = 14.5/320 = 0.0453125 units
//                    in y = 3mm on camera -> 0.75 units in gazebo;
//                           1mm on camera -> 0.25 units
//                           size of camera fov -> 43mm                  
//                           fov of camera = 10.75 units
//                           lambda(y) = 10.75/240 = 0.0447916 units
//Area of frame: (1.5/0.0453125)*(1.5/0.0447916) = 1108 pixels















// #include<stdio.h>
// #include<ros/ros.h>
// #include<sensor_msgs/Image.h>
// #include<cv_bridge/cv_bridge.h>
// #include "opencv2/imgproc.hpp"
// #include<opencv2/opencv.hpp>
// #include<sensor_msgs/image_encodings.h>
// #include<geometry_msgs/PoseStamped.h>
// #include <image_transport/image_transport.h>

// int h,w,y=0,z=0;
// cv::Mat Im;
// cv::Mat src;//=cv::Mat::zeros(cv::Size(240,320),CV_8UC3);

// void callback1(sensor_msgs::Image img){

//     h=img.height ; w=img.width ;

//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//       cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception &e)
//     {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//     }
//     // imageID = img.header.seq;
//     src = cv_ptr->image;
//     return;
// }

// void ProcessImg(){
//     // cv::Mat img2 ;//=cv::Mat::zeros(src.size(),CV_32FC1);
//     cv::cvtColor(src,Im,CV_BGR2GRAY);


//     // cv::Mat dst =cv::Mat::zeros(src.size(),CV_32FC1);
//     // cornerHarris(img2,Im,2,3,0.04);
//     return;
// }

// int main(int argc,char** argv){
//     ros::init(argc,argv,"trajectory_node");
//     ros::NodeHandle nh1;
//     ros::Subscriber sub1=nh1.subscribe("/iris/usb_cam/image_raw",1,callback1);
//     ros::Publisher pub1=nh1.advertise<sensor_msgs::Image>("OutputImg",1);
//     ros::Publisher pose_pub=nh1.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
//     ros::Rate looprate(1000);

//     while(ros::ok()){
//         ros::spinOnce();
//         cv::GaussianBlur(src, src, cv::Size(3, 3), 0, 0);
//         // cv::cvtColor(src,Im,CV_BGR2GRAY);
//         // ProcessImg();
//         // a.data=src;
        // sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
//         pub1.publish(thresh_msg);
//         geometry_msgs::PoseStamped vel;
//         vel.pose.position.y=y; vel.pose.position.z=z;
//         pose_pub.publish(vel);
//         looprate.sleep();
//     }

//     return 0;
// }
