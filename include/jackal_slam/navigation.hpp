/// \file nav.hpp
/// \brief this is a header file for jackal navigation

#ifndef JACKAL_SLAM_NAVIGATION_INCLUDE_GURAD_HPP
#define JACKAL_SLAM_NAVIGATION_INCLUDE_GUARD_HPP

#include<ros/ros.h>
#include<ros/console.h>
#include<geometry_msgs/PoseStamped.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include<visualization_msgs/MarkerArray.h>
#include<jackal_slam/costmap.hpp>
#include<jackal_slam/frontier_search.hpp>
#include<string>
#include<vector>


class Navigation
{

public: 
    
    /// \brief default constructor
    Navigation();
    
    /// \brief destructor
    ~Navigation();

    /// \brief stop the motion
    void stop();



private:

    // parameters
    double planner_freq_;

    double timeout;     

    double potential_scale_;

    double gain_scale_;
    
    ros::Duration progress_timeout_;

    bool visualize_;

    double min_frontier_size;

    ros::NodeHandle private_nh_;

    ros::NodeHandle relative_nh_;

    ros::Publisher marker_array_pub;

    ros::Publisher marker_pub;

    tf::TransformListener tf_listener_;

    Costmap costmap_client_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

    FrontierSearch search_;

    ros::Timer explore_timer_;
                                                                                                    
    ros::Timer oneshot_;

    std::vector<geometry_msgs::Point> frontier_blacklist_;

    geometry_msgs::Point prev_goal_;
    
    double prev_dist_; 

    ros::Time last_progress_;

    size_t last_markers_count;

    /// \brief plan the frontier point to move to
    void makePlan();

    /// \brief function to visualize the frontier
    /// \param frontiers - frontiers stored in vector 
    void visualizeFrontiers(const std::vector<Frontier> &frontiers);

    /// \brief check if robot reached the goal
    void reachedGoal(const actionlib::SimpleClientGoalState &status, 
                     const move_base_msgs::MoveBaseResultConstPtr &result,
                     const geometry_msgs::Point &frontier_goal);

    /// \brief function to check if goal in blacklist
    /// \param goal - goal pose
    /// \return true/false
    bool goalOnBlacklist(const geometry_msgs::Point &goal);


};



#endif


/// end file