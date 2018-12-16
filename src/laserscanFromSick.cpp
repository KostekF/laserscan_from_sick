#include "laserscanFromSick.h"

     void LaserscanFromSick::createLaserScan(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_cloud)
     {

     }
     
     
    float LaserscanFromSick::findSmallestValueInCol(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_cloud,int numOfCol)
    {
        float smallestValue=range_max_;//assign value of  max range)
        for(int i=0;i<ptr_cloud->height;++i) //iterate over rows
        {
            float P_z=ptr_cloud->points.at(i*ptr_cloud->width +numOfCol).z;
           /*
            if(P_z!=0)
            {ROS_INFO("P.z= %f",P_z);
             ROS_INFO("ptr_z= %f",ptr_cloud->points.at(i*ptr_cloud->width +numOfCol).z);
            }*/
             if(P_z>height_min_ && P_z<height_max_) //check if point is in our height boundaries
            {
                float P_x=ptr_cloud->points.at(i*ptr_cloud->width +numOfCol).x;
                float P_y=ptr_cloud->points.at(i*ptr_cloud->width +numOfCol).y;

                float possibleSmallest=hypot(P_x,P_y); //sqrt(P.x^2+P.y^2)
                // float possibleSmallest=sqrt(pow(P_x,2)+pow(P_y,2));  //hypot better?
                if(possibleSmallest>range_min_ && possibleSmallest<range_max_)//check if point is in our range boundaries
                {
                    if(possibleSmallest<smallestValue)//check if new col value is smallest value
                    {
                        smallestValue=possibleSmallest; //assign new smallest value
                    }
                }
            }
        }
        if(smallestValue!=range_max_)
        {
            return smallestValue;
        }
        else return std::numeric_limits<float>::quiet_NaN();
        
    }
    
  void LaserscanFromSick::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
  {
   
    try
    {
    
        //convert PointCloud2 to pcl pointcloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*ptr_cloud);
    
    
    

 
    
    scan_msg_->ranges.assign(ptr_cloud->width, std::numeric_limits<float>::quiet_NaN()); //create table of NaNs
    
    //assign values of closest obstacles to table
    for(int i=0;i<ptr_cloud->width;++i) //iterating over rows
    {
        scan_msg_->ranges[i]= findSmallestValueInCol(ptr_cloud,i);
    }
    
    //Create Header and publish scan
    std_msgs::Header header; // empty header
    header.stamp = ros::Time::now(); // time
    header.frame_id="laser";
    scan_msg_->header=header;
    scan_pub_.publish(scan_msg_);
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to laser message: "
                        << e.what());
    }
 
  }

 

    LaserscanFromSick::LaserscanFromSick() :scan_msg_(new sensor_msgs::LaserScan())
  {
    sub_ = nh_.subscribe ("cloud_in", 30,
                          &LaserscanFromSick::cloud_cb, this);
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan> ("scan", 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName ("cloud_in");
    std::string r_it = nh_.resolveName ("scan");
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing laserscan on topic " << r_it );
    
    
 
    private_nh_.param<float>("range_min", range_min_, 0.5);
    private_nh_.param<float>("range_max", range_max_,10);
    private_nh_.param<float>("angle_min", angle_min_, -1.0471975512);////60degree in radians because sick mrs 6000 has 120degree horizontal FOV
    private_nh_.param<float>("angle_max", angle_max_, 1.0471975512);
    private_nh_.param<float>("angle_increment", angle_increment_, 0.0022689280276); ////0.13 degree in radians->from sick site
    private_nh_.param<float>("scan_time", scan_time_, 1.0/30.0);
    private_nh_.param<float>("height_min", height_min_, 0.05);
    private_nh_.param<float>("height_max", height_max_, 1);

    
    
    
    // initialize scan_msg_
    scan_msg_->angle_min=angle_min_; 
    scan_msg_->angle_max=angle_max_;
    scan_msg_->angle_increment=angle_increment_;
    scan_msg_->time_increment = 0.0;
    scan_msg_->scan_time = scan_time_;
    scan_msg_->range_min = range_min_; //[m] change it later maybe?
    scan_msg_->range_max = range_max_;//[m] change it later maybe?
    
    
    
  }
