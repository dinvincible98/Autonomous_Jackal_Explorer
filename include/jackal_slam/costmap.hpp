/// \file costmap.hpp 
/// \brief header file for costmap

#ifndef JACKAL_SLAM_COSTMAP_INCLUDE_GUARD_HPP
#define JACKAL_SLAM_COSTMAP_INCLUDE_GUARD_HPP

#include<ros/ros.h>
#include<ros/console.h>
#include<costmap_2d/costmap_2d.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/OccupancyGrid.h>
#include<map_msgs/OccupancyGridUpdate.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<iostream>


/// \brief class to handle costmap 
class Costmap
{

public:
    /// \brief constructor for costmap
    /// \param param_nh - parameter nodehandle
    /// \param susbcriber_nh - subscriber nodehandle
    /// \param tf_listener - transform listener
    Costmap(ros::NodeHandle & param_nh, ros::NodeHandle &subscribe_nh,
                            const tf::TransformListener *tf_listener);

    /// \brief get RobotPose
    geometry_msgs::Pose getRobotPose() const;


    /// \brief get costmap
    costmap_2d::Costmap2D* getCostmap();

    
    /// \brief get costmap
    const costmap_2d::Costmap2D* getCostmap() const;

    
    /// \brief get global frame id
    const std::string& getGlobalFrameID() const;


    /// \brief get base frame id
    const std::string& getBaseFrameID() const;



private:

    /// \brief full map callback function, update occupancy grid
    void updateFullMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    
    /// \brief Costmap2D variables
    costmap_2d::Costmap2D costmap_;

    /// \brief tf listener
    const tf::TransformListener *const tf_;

    /// \brief global frame
    std::string global_frame_;

    /// \brief base frame
    std::string base_frame_;
    
    /// \brief transform tolerance;
    double transform_tol_;

    /// \brief costmap subscriber
    ros::Subscriber costmap_sub_;

};




#endif