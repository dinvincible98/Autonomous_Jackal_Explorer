/// \file FrontierSearch.hpp
/// \brief This is the header file for FrontierSearch.cpp

#ifndef JACKAL_SLAM_FRONTIER_SEARCH_INCLUDE_GUARD_HPP
#define JACKAL_SLAM_FRONTIER_SEARCH_INCLUDE_GUARD_HPP

#include<ros/ros.h>
#include<costmap_2d/costmap_2d.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/PointStamped.h>
#include<vector>


/// \brief struct stores frontier info
/// \param size - frontier size
/// \param min_dist - minmum distance of frontier
/// \param cost - cost of frontier
/// \param initial - initial point of frontier
/// \param centroid - centroid of the frontier
/// \param middle - middle of the frontier
/// \param points - vector of frontier points  
struct Frontier
{
    unsigned int size;
    double min_dist;
    double cost;
    geometry_msgs::Point initial;
    geometry_msgs::Point centroid;
    geometry_msgs::Point middle;
    std::vector<geometry_msgs::Point> points;
};


/// \brief class for frontier search
class FrontierSearch
{

public:

    FrontierSearch(){}

    /// \brief constructor for search
    /// \param costmap - costmap
    /// \param potential scale 
    FrontierSearch(costmap_2d::Costmap2D *costmap, double potential_scale, double gain_scale, double min_frontier_size);

    /// \brief search from the initial position
    /// \param pose - position of the point
    /// \return frontier list 
    std::vector<Frontier> searchFrom(geometry_msgs::Point pose);



private:

    /// \brief starts from the inital cell, builds the frontier from adjacent cells
    /// \param init_cell - starting cell
    /// \param ref - reference cell
    /// \param frontier_flag - flag to track the visited cell  
    /// \return frontier
    Frontier buildNewFrontier(unsigned int init_cell, unsigned int ref, std::vector<bool> &frontier_flag);


    /// \brief evaluates if cell is a valid candidate for a new frontier
    /// \param idx - cell index
    /// \param frontier_flag - check if the cell is unvisited
    /// \return true/false
    bool isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag);

    /// \brief computes the frontier cost
    /// \param frontier - strcut frontier
    /// \return cost to move to frontier
    double frontierCost(const Frontier &frontier);
    
    /// \brief determine 4 connected neighbourhood of an input cell
    /// \param idx - cell index
    /// \param costmap - costmap
    /// \return 4-connected neighbour cell 
    std::vector<unsigned int> nhood4(unsigned int idx, const costmap_2d::Costmap2D &costmap);

    /// \brief determine 8 connected neighbourhood of an input cell
    /// \param idx - cell index
    /// \param costmap - costmap
    /// \return 8-connected neighbour cell  
    std::vector<unsigned int> nhood8(unsigned int idx, const costmap_2d::Costmap2D &costmap);

    /// \brief check if there is a nearest cell given a initial start point
    /// \param result - map x pose
    /// \param start - map y pose
    /// \param val - costmap cell value
    /// \param costmap - costmap
    /// \return true/false
    bool nearestCell(unsigned int &result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D &costmap);

    costmap_2d::Costmap2D *costmap_;

    /// \brief (char)map
    unsigned char *map_;

    /// \brief x size of the map 
    unsigned int size_x_;

    /// \brief y size of the map
    unsigned int size_y_;

    
    double potential_scale_, gain_scale_;

    double min_frontier_size_;


};




#endif
/// end file