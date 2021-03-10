/// \file  nav.cpp 
/// \brief this is an implementation file for header file nav.hpp


#include<ros/ros.h>
#include<ros/console.h>
#include<jackal_slam/navigation.hpp>


/// \brief check if two Points have almost identical pose 
inline static bool operator==(const geometry_msgs::Point& a,
                              const geometry_msgs::Point& b)
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

Navigation::Navigation():private_nh_("~"), tf_listener_(ros::Duration(10.0)), 
           costmap_client_(private_nh_,relative_nh_,&tf_listener_),
           move_base_client_("move_base"),prev_dist_(0), last_markers_count(0)
{
    // parameters
    private_nh_.param("planner_frequency", planner_freq_, 1.0);
    private_nh_.param("progress_timeout", timeout, 10.0);
    private_nh_.param("visualize", visualize_,true);
    private_nh_.param("potential_scale", potential_scale_,3.0);
    private_nh_.param("gain_scale" ,gain_scale_,1.0);
    private_nh_.param("min_frontier_size",min_frontier_size,1.0);

    progress_timeout_ = ros::Duration(timeout);
    search_ = FrontierSearch(costmap_client_.getCostmap(),
                            potential_scale_,gain_scale_,min_frontier_size);
                                                                      
    if(visualize_)
    {
        marker_array_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers",10);

        marker_pub = private_nh_.advertise<visualization_msgs::Marker>("target",10);

    }

    ROS_INFO("Waiting to connect to move base server");
    move_base_client_.waitForServer();
    ROS_INFO("Connected to move_base server");

    explore_timer_ = relative_nh_.createTimer(ros::Duration(1.0/planner_freq_),
                    [this](const ros::TimerEvent&) {makePlan();});


}


Navigation::~Navigation()
{
    stop();
}

void Navigation::visualizeFrontiers(const std::vector<Frontier> &frontiers)
{
    // set color
    std_msgs:: ColorRGBA red;
    red.r = 1.0;
    red.g = 0;
    red.b = 0;
    red.a = 1.0; 

    std_msgs:: ColorRGBA blue;
    blue.r = 0;
    blue.g = 0.0;
    blue.b = 1.0;
    blue.a = 1.0; 

    ROS_DEBUG("visualize %lu frontiers", frontiers.size());
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker> &markers = markers_msg.markers; 
    visualization_msgs::Marker m;

    m.header.frame_id = costmap_client_.getGlobalFrameID();
    m.header.stamp = ros::Time::now();
    m.ns = "frontiers";
    m.scale.x = 1.0;
    m.scale.y = 1.0;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 255;
    m.color.a = 255;

    m.lifetime = ros::Duration(0);
    m.frame_locked = true;

    m.action = visualization_msgs::Marker::ADD;

    size_t id = 0;

    for (auto &frontier : frontiers)
    {
        m.type = visualization_msgs::Marker::POINTS;
        m.id = int(id);
        m.pose.position = {};
        m.scale.x = 0.1;                                                         
        m.scale.y = 0.1;
        m.points = frontier.points;
        
        // red for blacklsit frontier
        if (goalOnBlacklist(frontier.centroid))
        {
            m.color = red;

        }
        // blue for reacheable frontier
        else
        {
            m.color = blue; 
        }
        markers.push_back(m);
        ++id;
    }
    size_t cur_marker_count = markers.size();

    // delete previous unused marker
    m.action = visualization_msgs::Marker::DELETE;
    for (;id<last_markers_count;++id)
    {
        m.id = int(id);
        markers.push_back(m);
    }

    last_markers_count = cur_marker_count;
    marker_array_pub.publish(markers_msg);


}

void Navigation::makePlan()
{
    // find frontiers
    auto pose = costmap_client_.getRobotPose();
    // get frontiers sorted based on cost
    auto frontiers = search_.searchFrom(pose.position);
    ROS_DEBUG("found %lu frontiers",frontiers.size());

    for (size_t i=0;i<frontiers.size();++i)
    {
        ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
    }
    if(frontiers.empty())
    {
        stop();
        return;
    }
    // publish frontiers as visualization marker
    if(visualize_)
    {
        visualizeFrontiers(frontiers);
    }

    // find non blacklisted frontier
    auto frontier = std::find_if_not(frontiers.begin(),frontiers.end(),
                    [this](const Frontier &frontier){return goalOnBlacklist(frontier.centroid);});

    if (frontier == frontiers.end())
    {
        stop();
        return;
    }

    // choose the centroid of frontier as target point
    geometry_msgs::Point target_pose = frontier->centroid;

    // timeout if no progress
    bool same_goal = prev_goal_ == target_pose;

    
    prev_goal_ = target_pose ;
    
    // there are some progress
    if(!same_goal || prev_dist_>frontier->min_dist)
    {
        last_progress_ = ros::Time::now();
        prev_dist_ = frontier->min_dist;
    }


    // blacklist if no progress for a long time

    if(ros::Time::now()-last_progress_ > progress_timeout_)
    {
        frontier_blacklist_.push_back(target_pose);
        ROS_INFO("This is an unreachable point");

        // re-select point to move to 
        makePlan();
        return;
    }


    if(same_goal)
    {
        return;
    }

    // send goal to move base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position = target_pose;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
    goal.target_pose.header.stamp = ros::Time::now();

    // initialize arror marker for target pose
    visualization_msgs::Marker marker;
    marker.header.frame_id = costmap_client_.getGlobalFrameID();
    marker.header.stamp = ros::Time::now();
    marker.ns = "target_pose";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = target_pose;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_pub.publish(marker);
    
    ROS_INFO("Move to goal, x: %f, y: %f", target_pose.x, target_pose.y);
    
    move_base_client_.sendGoal(goal, [this,target_pose](const actionlib::SimpleClientGoalState &status, 
                              const move_base_msgs::MoveBaseResultConstPtr &result){reachedGoal(status,result,target_pose);});
    
    ros::Duration(1.0).sleep();
    
}




bool Navigation::goalOnBlacklist(const geometry_msgs::Point &goal)
{
    constexpr static size_t tolerance = 5;
    costmap_2d::Costmap2D *costmap2d = costmap_client_.getCostmap();

    // check if goal is on blacklist
    for (auto &frontier_goal : frontier_blacklist_)
    {
        double x_diff = std::fabs(goal.x - frontier_goal.x);
        double y_diff = std::fabs(goal.y - frontier_goal.y);
        
        if (x_diff<tolerance*costmap2d->getResolution() && y_diff<tolerance*costmap2d->getResolution())
        {
            return true;
        }
    }
    return false;

}



void Navigation::reachedGoal(const actionlib::SimpleClientGoalState &status,
                      const move_base_msgs::MoveBaseResultConstPtr &,
                      const geometry_msgs::Point &frontier_goal)
{
    ROS_INFO("reached goal with status: %s", status.toString().c_str());
    // failed to reach goal
    if(status == actionlib::SimpleClientGoalState::ABORTED)
    {
        frontier_blacklist_.push_back(frontier_goal);
        ROS_DEBUG("adding current goal to blacklist");

    }

    // find new goal immediately
    oneshot_ = relative_nh_.createTimer(ros::Duration(1.0), [this](const ros::TimerEvent&){makePlan();},true);


}


void Navigation::stop()
{
    move_base_client_.cancelAllGoals();
    explore_timer_.stop();
    ROS_INFO("Finished mapping!");

}



int main(int argc, char** argv)
{
    ros::init(argc,argv, "jackal_slam");
    Navigation navigation;
    ros::spin();

    return 0;

}




/// end file 