/// \file costmap.cpp 
/// \brief  implementation file for costmap.hpp

#include<jackal_slam/costmap.hpp>
#include<string>



/// \brief translation for cost char map 
std::array<unsigned char, 256> init_translation_table()
{
    std::array<unsigned char, 256>cost_translation_table;

    // mapping data [0-100] to [0-255]
    for (size_t i=0; i<256; ++i)
    {
        cost_translation_table[i] = static_cast<unsigned char>(1 + (251*(i-1))/97);
    }

    // special values:
    cost_translation_table[0] = 0;                                     // no obstacle
    cost_translation_table[99] = 253;                                  // inscribed obstacle
    cost_translation_table[100] = 254;                                 // leathal obstacle
    cost_translation_table[static_cast<unsigned char>(-1)] = 255;      // unknown

    return cost_translation_table;
}

// get cost_translation_table
std::array<unsigned char,256> init_translation_table();
static const std::array<unsigned char,256> cost_translation_table = init_translation_table();


Costmap::Costmap(ros::NodeHandle &param_nh, ros::NodeHandle &subscribe_nh,
                const tf::TransformListener *tf_listener):tf_(tf_listener)
{
    std::string costmap_topic;
    
    // get parameters
    param_nh.param("costmap_topic", costmap_topic,std::string("map"));
    param_nh.param("base_frame",base_frame_,std::string("base_link"));
    param_nh.param("transform_tolerance",transform_tol_,1.0);


    costmap_sub_ = subscribe_nh.subscribe<nav_msgs::OccupancyGrid>(costmap_topic,10,
                                [this](const nav_msgs::OccupancyGrid::ConstPtr &msg){updateFullMap(msg);});

    // check the if map is received
    ROS_INFO("waiting for costmap, topic: %s",costmap_topic.c_str());
    
    // get costmap msg from topic 
    auto costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(costmap_topic,subscribe_nh);

    // update map
    updateFullMap(costmap_msg);



    // resolve tf_prefix for base_frame
    std::string tf_prefix = tf::getPrefixParam(param_nh);
    base_frame_ = tf::resolve(tf_prefix, base_frame_);

    ros::Time last_error = ros::Time::now();
    std::string tf_error;
    
    while (ros::ok() && !tf_->waitForTransform(global_frame_,base_frame_,ros::Time(),
                                               ros::Duration(0.1),ros::Duration(0.01),&tf_error))
    {
        ros::spinOnce();
        if(last_error + ros::Duration(5.0) < ros::Time::now())
        {
            ROS_WARN("Timed out waiting for transform from %s to %s",base_frame_.c_str(),global_frame_.c_str());
            last_error = ros::Time::now();
        }
        tf_error.clear();
    }

}



costmap_2d::Costmap2D* Costmap::getCostmap()
{
    return &costmap_;
}


const costmap_2d::Costmap2D* Costmap::getCostmap() const
{
    return &costmap_;
}



const std::string& Costmap::getGlobalFrameID() const
{
    return global_frame_;
}


const std::string& Costmap::getBaseFrameID() const
{
    return base_frame_;
}


void Costmap::updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    // get parameters
    global_frame_= msg->header.frame_id;
    unsigned int map_x = msg->info.width;
    unsigned int map_y= msg->info.height;
    double resolution = msg->info.resolution;
    double x_origin = msg->info.origin.position.x;
    double y_origin = msg->info.origin.position.y;

    ROS_DEBUG("Receving the new map, resizing to : %d, %d",map_x, map_y);
    costmap_.resizeMap(map_x,map_y,resolution,x_origin,y_origin);

    // lock the existing map data
    auto *mutex = costmap_.getMutex();
    std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*mutex);

    // fill map with data
    unsigned char *costmap_data = costmap_.getCharMap();
    size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
    ROS_DEBUG("full map update: %lu values", costmap_size);
    
    for(size_t i=0; i<costmap_size && i<msg->data.size();++i)
    {
        unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
        costmap_data[i] = cost_translation_table[cell_cost];
    }
    ROS_DEBUG("map update, written %lu values", costmap_size);

}





geometry_msgs::Pose Costmap::getRobotPose() const
{
    // initialize tf pose 
    tf::Stamped<tf::Pose> global_pose;
    global_pose.setIdentity();

    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    
    robot_pose.frame_id_= base_frame_;
    robot_pose.stamp_ = ros::Time();
    
    //save time for checking tf delay
    ros::Time cur_time = ros::Time::now();
    
    // get the global pose of the robot
    try
    {
        tf_->transformPose(global_frame_,robot_pose,global_pose);
    }
    catch(tf::LookupException &ex)
    {
        ROS_ERROR_THROTTLE(1.0,"No transform available error looking up robot pose: %s\n",ex.what());
        return {};
    }
    catch(tf::ConnectivityException &ex)
    {
        ROS_ERROR_THROTTLE(1.0,"Connectivity error looking up robot pose: %s\n",ex.what());
        return {};
    }
    catch(tf::ExtrapolationException &ex)
    {
        ROS_ERROR_THROTTLE(1.0,"Extrapolation error looking up robot pose: %s\n",ex.what());
        return {};
    }

    // check global pose timeout
    if (cur_time.toSec()-global_pose.stamp_.toSec() > transform_tol_)
    {
        ROS_ERROR_THROTTLE(1.0,"Costmap transform timeout");
        return{};
    }

    // convert to pose stamp
    geometry_msgs::PoseStamped msg;
    tf::poseStampedTFToMsg(global_pose,msg);
    return msg.pose;

}








/// end file