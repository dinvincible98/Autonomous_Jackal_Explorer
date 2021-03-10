/// \file  FrontierSearch.cpp
/// \brief this is an implementation file for FrontierSearch.hpp

#include<jackal_slam/frontier_search.hpp>
#include<costmap_2d/costmap_2d.h>
#include<costmap_2d/cost_values.h>
#include<geometry_msgs/Point.h>
#include<mutex>


using costmap_2d::FREE_SPACE;
using costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D *costmap, double potential_scale, double gain_scale,double min_frontier_size)
                              :costmap_(costmap),potential_scale_(potential_scale),gain_scale_(gain_scale),min_frontier_size_(min_frontier_size)        
{

}

std::vector<Frontier>FrontierSearch::searchFrom(geometry_msgs::Point pose)
{
    std::vector<Frontier> frontier_list;

    // check if the robot is inside the costmap
    unsigned int mx, my;
    if(!costmap_->worldToMap(pose.x,pose.y,mx,my))
    {
        ROS_ERROR("Robot is out of the costmap bounds, cannot search for the frontier");
        return frontier_list;
    }

    // lock the map while searching the new frontier
    std::lock_guard<costmap_2d::Costmap2D::mutex_t>lock(*(costmap_->getMutex()));

    map_ = costmap_->getCharMap();
    size_x_ = costmap_->getSizeInCellsX();
    size_y_ = costmap_->getSizeInCellsY();

    // initiate flags to track the visited cell and  frontier cell
    std::vector<bool> frontier_flag(size_x_ * size_y_,false);
    std::vector<bool> visited_flag(size_x_ * size_y_,false);

    // bfs search
    std::queue<unsigned int> bfs;
    
    // find closest clear cell to start search
    unsigned int clear, pos = costmap_->getIndex(mx,my);
    if (nearestCell(clear,pos,FREE_SPACE,*costmap_))
    {
        bfs.push(clear);
    }
    else
    {
        bfs.push(pos);
        ROS_WARN("Could not find nearby cell to start search");
    }
    visited_flag[bfs.front()] = true;

    while(!bfs.empty())
    {
        unsigned int idx = bfs.front();
        bfs.pop();

        // iterate over 4 connected neighborhood
        for (unsigned int nbr : nhood4(idx, *costmap_))
        {
            // add free, unvisited cell in a desceding search
            // initiate on non-free cell
            if(map_[nbr] <= map_[idx] && !visited_flag[nbr])
            {
                visited_flag[nbr] = true;
                bfs.push(nbr);
            }
            // check if cell is new frontier cell(unvisited, no information, free) 
            else if (isNewFrontierCell(nbr,frontier_flag))
            {
                frontier_flag[nbr] = true;
                Frontier new_frontier;
                new_frontier = buildNewFrontier(nbr,pos,frontier_flag);

                // checlk the frontier size
                if (new_frontier.size * costmap_->getResolution() >= min_frontier_size_)
                {
                    frontier_list.push_back(new_frontier);
                }
            }
        }
    }

    for (auto &frontier : frontier_list)
    {
        frontier.cost = frontierCost(frontier);
    }
    
    // sort the frontier based on cost
    std::sort(frontier_list.begin(),frontier_list.end(),[](const Frontier &f1, const Frontier &f2)
             {return f1.cost<f2.cost;});

    return frontier_list; 


}

Frontier FrontierSearch::buildNewFrontier(unsigned int init_cell, unsigned int ref, std::vector<bool> &frontier_flag)
{
    // initiated frontier
    Frontier res;
    res.centroid.x = 0;
    res.centroid.y = 0;
    res.size = 1;
    res.min_dist = std::numeric_limits<double>::infinity();

    // record initial contact point for frontier
    unsigned int ix, iy;
    costmap_->indexToCells(init_cell,ix,iy);
    costmap_->mapToWorld(ix,iy,res.initial.x,res.initial.y); 

    // push initial cell to the queue;
    std::queue<unsigned int> bfs;
    bfs.push(init_cell);

    // reference position in world frame
    unsigned int rx, ry;
    double ref_x_, ref_y_;
    costmap_->indexToCells(ref,rx,ry);
    costmap_->mapToWorld(rx,ry,ref_x_,ref_y_);

    while (!bfs.empty())
    {
        unsigned int idx = bfs.front();
        bfs.pop();

        // try to add cell in 8-connected neighborhood frontier
        for (unsigned int nbr : nhood8(idx, *costmap_))
        {
            // check if neighboor is a potential fronteir cell
            if (isNewFrontierCell(nbr,frontier_flag))
            {
                // mark nbr point as frontier point
                frontier_flag[nbr] = true;
                
                // get world coordinate for nbr 
                unsigned int mx, my;
                double wx,wy;
                costmap_->indexToCells(nbr,mx,my);
                costmap_->mapToWorld(mx,my,wx,wy);

                geometry_msgs::Point pt;
                pt.x = wx;
                pt.y = wy;
                // add point to point_list
                res.points.push_back(pt);
                
                // update frontier size
                res.size++;

                // update centroid of the frontier
                res.centroid.x += wx;
                res.centroid.y += wy;

                // determine frontier's distance from robot(close first)
                double distance;
                distance = std::sqrt(std::pow((double(ref_x_)-double(wx)), 2.0) + 
                                     std::pow((double(ref_y_)-double(wy)), 2.0));
                
                // update frontier info
                if (distance < res.min_dist)
                {
                    res.min_dist = distance;
                    res.middle.x = wx;
                    res.middle.y = wy;
                }

                // add to queue fpr bfs search
                bfs.push(nbr);

            }
        }
    }

    // average frontier centroid
    res.centroid.x /= res.size;
    res.centroid.y /= res.size;
    return res;
}   

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool> &frontier_flag)
{
    // check the cell is unknown and not marked as frontier
    if(map_[idx] != NO_INFORMATION || frontier_flag[idx])
    {
        return false;
    }

    // frontier cell should have at least 4 connected neighborhood
    for (unsigned int nbr : nhood4(idx, *costmap_))
    {
        if(map_[nbr] == FREE_SPACE)
        {
            return true;
        }
    }
    return false; 

}




std::vector<unsigned int>FrontierSearch::nhood4(unsigned int idx, const costmap_2d::Costmap2D &costmap)
{
    // get 4 connected neighborhood idx and check the edge of the map
    std::vector<unsigned int> res;

    unsigned int size_x_ = costmap.getSizeInCellsX();
    unsigned int size_y_ = costmap.getSizeInCellsY();

    if (idx > size_x_*size_y_-1)
    {
        ROS_WARN("Off map!");
        return res;
    }

    if (idx%size_x_ > 0)
    {
        res.push_back(idx-1);
    }
    if (idx%size_x_ < size_x_-1)
    {
        res.push_back(idx+1);
    }
    if (idx >= size_x_)
    {
        res.push_back(idx-size_x_);
    }
    if (idx < size_x_*size_y_-1)
    {
        res.push_back(idx+size_x_);
    }
    return res;
}


std::vector<unsigned int> FrontierSearch::nhood8(unsigned int idx, const costmap_2d::Costmap2D &costmap)
{
    // initialize with 4 connected nbr
    std::vector<unsigned int> res = nhood4(idx,costmap);

    unsigned int size_x_ = costmap.getSizeInCellsX();
    unsigned int size_y_ = costmap.getSizeInCellsY();
    
    // search for rest connected nbr
    if (idx > size_x_*size_y_-1)
    {
        ROS_WARN("Off map!");
        return res;
    }

    if (idx%size_x_>0 && idx>=size_x_)
    {
        res.push_back(idx-1-size_x_);
    }
    if (idx%size_x_>0 && idx<size_x_*(size_y_-1))
    {
        res.push_back(idx-1+size_x_);
    }
    if (idx%size_x_<size_x_-1 && idx>=size_x_)
    {
        res.push_back(idx+1-size_x_);
    }
    if (idx%size_x_<size_x_-1 && idx<size_x_*(size_y_-1))
    {
        res.push_back(idx+1+size_x_);
    }
    return res;
}

bool FrontierSearch::nearestCell(unsigned int &result, unsigned int start, unsigned char val, const costmap_2d::Costmap2D &costmap)
{
    // get parameters value
    const unsigned char *map = costmap.getCharMap();
    const unsigned int size_x_ = costmap.getSizeInCellsX();
    const unsigned int size_y_ = costmap.getSizeInCellsY();

    // check if the start position is inside the costmap
    if (start >= size_x_*size_y_)
    {
        return false;
    }
    
    // implement bfs
    std::queue<unsigned int> bfs;

    // flag to track if the cell is visited
    std::vector<bool> visited_flag(size_x_*size_y_,false);
    
    // push initial cell
    bfs.push(start);
    visited_flag[start] = true;

    // search for neighboor cells
    while(!bfs.empty())
    {
        unsigned int idx = bfs.front();
        bfs.pop();

        // return if correct value is found
        if(map[idx] == val)
        {
            result = idx;
            return true;
        }
        
        // search for neighboorhood cell
        std::vector<unsigned int> nh = nhood8(idx,costmap); 
        for(unsigned int i=0; i<nh.size(); ++i)
        {
            if(!visited_flag[i])
            {
                bfs.push(nh[i]);
                visited_flag[nh[i]]= true;
            }
        }
    }
    return false;

}

double FrontierSearch::frontierCost(const Frontier &frontier)
{
    return (potential_scale_ * frontier.min_dist * costmap_->getResolution()) -
           (gain_scale_ * frontier.size * costmap_->getResolution());
    // return frontier.min_dist*costmap_->getResolution();
}
























///end file