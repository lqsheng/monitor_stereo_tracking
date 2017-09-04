#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/UInt16MultiArray.h>

using namespace std;
using namespace cv;

Rect select_rect;
bool select_flag=false;
Point origin;
Mat frame_l;
Mat frame_r;

bool cam_left_roi_selected=false;
bool cam_right_roi_selected=false;

/************************************************************************************************************************/
/**** 如果采用这个onMouse()函数的话，则可以画出鼠标拖动矩形框的4种情形 ****/
/************************************************************************************************************************/
void onMouse(int event,int x,int y,int,void*)
{
 //Point origin;//不能在这个地方进行定义，因为这是基于消息响应的函数，执行完后origin就释放了，所以达不到效果。
 if(select_flag)
 {
     select_rect.x=MIN(origin.x,x);//不一定要等鼠标弹起才计算矩形框，而应该在鼠标按下开始到弹起这段时间实时计算所选矩形框
     select_rect.y=MIN(origin.y,y);
     select_rect.width=abs(x-origin.x);//算矩形宽度和高度
     select_rect.height=abs(y-origin.y);
     //select_rect&=Rect(0,0,frame.cols,frame.rows);//保证所选矩形框在视频显示区域之内
 }
 if(event==CV_EVENT_LBUTTONDOWN)
 {
     select_flag=true;//鼠标按下的标志赋真值
     origin=Point(x,y);//保存下来单击是捕捉到的点
     select_rect=Rect(x,y,0,0);//这里一定要初始化，宽和高为(0,0)是因为在opencv中Rect矩形框类内的点是包含左上角那个点的，但是不含右下角那个点
 }
 else if(event==CV_EVENT_LBUTTONUP)
 {
     select_flag=false;
 }
}

void roi_left_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    if(!cam_left_roi_selected)
    {
        frame_l=cv_bridge::toCvShare(msg, "bgr8")->image;
        rectangle(frame_l,select_rect,Scalar(0,0,255),3,8,0);//能够实时显示在画矩形窗口时的痕迹
        cv::imshow("cam_left", frame_l);
    }
    else
        cv::imshow("cam_left", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void roi_right_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    if(!cam_right_roi_selected)
    {
        frame_r=cv_bridge::toCvShare(msg, "bgr8")->image;
        rectangle(frame_r,select_rect,Scalar(0,0,255),3,8,0);//能够实时显示在画矩形窗口时的痕迹
        cv::imshow("cam_right", frame_r);
    }
    else
        cv::imshow("cam_right", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "monitor_stereo_camera_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_left_cam = it.subscribe("stereo_dsst_tracking/cam_left", 1, roi_left_Callback);
    image_transport::Subscriber sub_right_cam = it.subscribe("stereo_dsst_tracking/cam_right", 1, roi_right_Callback);


    ros::Publisher pub_left_roi= nh.advertise<std_msgs::UInt16MultiArray>("stereo_dsst_tracking/roi_left", 10);
    ros::Publisher pub_right_roi= nh.advertise<std_msgs::UInt16MultiArray>("stereo_dsst_tracking/roi_right", 10);

    ros::Rate rate(20.0);
    //建立窗口
    namedWindow("cam_left",1);//显示视频原图像的窗口
    //捕捉鼠标
    setMouseCallback("cam_left",onMouse,0);

     while(ros::ok())
     {
         //键盘响应
         char c=(char)waitKey(20);
         if(' '==c)//ESC键
         {
             cam_left_roi_selected=true;
             std_msgs::UInt16MultiArray msg_left_roi;
             msg_left_roi.layout.dim.push_back(std_msgs::MultiArrayDimension());  
             msg_left_roi.layout.dim[0].size = 4;  
             msg_left_roi.layout.dim[0].stride = 1;  
             msg_left_roi.layout.dim[0].label = "left";    
             msg_left_roi.data.resize(6);   
             msg_left_roi.data[0]=select_rect.x;
             msg_left_roi.data[1]=select_rect.y;
             msg_left_roi.data[2]=select_rect.width;
             msg_left_roi.data[3]=select_rect.height;
             pub_left_roi.publish(msg_left_roi);
             break;
         }
         ros::spinOnce();
     }

     //建立窗口
     namedWindow("cam_right",1);//显示视频原图像的窗口
     //捕捉鼠标
      setMouseCallback("cam_right",onMouse,0);

      while(ros::ok())
      {
          //键盘响应
          char c=(char)waitKey(20);
          if(' '==c)//ESC键
            {
                cam_left_roi_selected=true;
                std_msgs::UInt16MultiArray msg_right_roi;
                msg_right_roi.layout.dim.push_back(std_msgs::MultiArrayDimension());  
                msg_right_roi.layout.dim[0].size = 4;  
                msg_right_roi.layout.dim[0].stride = 1;  
                msg_right_roi.layout.dim[0].label = "right";    
                msg_right_roi.data.resize(6);   
                msg_right_roi.data[0]=select_rect.x;
                msg_right_roi.data[1]=select_rect.y;
                msg_right_roi.data[2]=select_rect.width;
                msg_right_roi.data[3]=select_rect.height;
                pub_right_roi.publish(msg_right_roi);
                break;
            }
            ros::spinOnce();
      }



    while(ros::ok()){
        // imshow("cam_left",frame_l);
        // imshow("cam_right",frame_r);
        if(waitKey(30)==27)break;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
