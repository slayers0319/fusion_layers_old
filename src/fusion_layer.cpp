#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <fusion_layers/fusion_layer.h>
#include <pluginlib/class_list_macros.h>

#define DEBUG 0

PLUGINLIB_EXPORT_CLASS(fusion_layer_namespace::FusionLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace fusion_layer_namespace{

FusionLayer::FusionLayer() {}

void split(char *src,const char *separator, std::vector<std::string> &dest) {
/*
    src 源字串的首地址(buf的地址) 
    separator 指定的分割字元
    dest 接收子字串的陣列
*/
#if DEBUG
    ROS_INFO("split string");
#endif
    char *pNext;
    if (src == NULL || strlen(src) == 0){ //如果傳入的地址為空或長度為0，直接終止 c
        ROS_INFO("string==NULL");
        return;
    }
    if (separator == NULL || strlen(separator) == 0){ //如未指定分割的字串，直接終止 
        ROS_INFO("separator==NULL");
        return;
    }
    pNext = (char *)strtok(src,separator); //必須使用(char *)進行強制型別轉換(雖然不寫有的編譯器中不會出現指標錯誤)
    while(pNext != NULL) {
        dest.push_back(pNext);
        pNext = (char *)strtok(NULL,separator);  //必須使用(char *)進行強制型別轉換
    }
} 

void dataSplit(const std_msgs::String::ConstPtr& msg){ 
    std::lock_guard<std::mutex> l(_data_mutex);

#if DEBUG
    ROS_INFO("callback start ------");
    ROS_INFO("get msg %s", msg->data.c_str());
#endif

    //input string
    char *buf = const_cast<char*>(msg->data.c_str());

    //clear split_result
    std::vector<std::string> ().swap(split_result);
    
    //there is not input
    if(buf==NULL || strlen(buf)==0 ){
#if DEBUG
        ROS_INFO("msg==NULL");
#endif
        return;
    }else if(msg->data.c_str()==NONE){
#if DEBUG
    ROS_INFO("none");
#endif
        return;
    }else if(msg->data.c_str()==CLEAR){
        //clear point vector
        std::vector<PointDouble> ().swap(related_points);
        std::vector<PointDouble> ().swap(absolute_points);
        ROS_INFO("clear fusion layers");
        return;
    }

    //split data
    split(buf,",",split_result);

#if DEBUG
    ROS_INFO("split_result.size() = %d", (int)split_result.size());
#endif

    if(split_result.size()%3 || split_result.empty() || split_result.size()<=3){
#if DEBUG
        ROS_INFO("not a point");
#endif
        return;
    }else{
        //clear vector
        std::vector<PointDouble> ().swap(related_points);
        std::vector<PointDouble> ().swap(absolute_points);

        //split_result=["P1", "x1", "y1", "P2", "x2", "y2"]  data format
        ROS_INFO("(%s,%s) (%s,%s)",split_result[1].c_str(),split_result[2].c_str(),split_result[4].c_str(),split_result[5].c_str());
        for(int i = 1;i < split_result.size(); i+=3) {
            PointDouble pt;
            
            double sx = std::strtod(split_result[i].c_str(),NULL); //convert string to double
            pt.x= sx;

            double sy = std::strtod(split_result[i+1].c_str(),NULL); //convert string to double
            pt.y= sy;
            related_points.push_back(pt);
        }
    }

    //calculate absolute coordinate
    for(int i=0;i<related_points.size();++i){
        double r =sqrt(pow(related_points[i].x,2)+pow(related_points[i].y,2));
        double theata = atan2(related_points[i].y,related_points[i].x) - (pi/2);

        if((theata+pi)<=0){
            theata = theata + 2*pi;
        }
        PointDouble pt;
        pt.x = robot_px + cos(theata+robot_pyaw)*r;
        pt.y = robot_py + sin(theata+robot_pyaw)*r;
        absolute_points.push_back(pt);
    }

#if DEBUG
    ROS_INFO("before thickened absolute_points.size() =  %d", (int)absolute_points.size());
#endif    

    if(absolute_points.size()==2){
        //加粗
        thickened();
    }

#if DEBUG
    ROS_INFO("before computeMapBounds absolute_points.size() =  %d", (int)absolute_points.size());
#endif 

    //reflesh map bounds
    computeMapBounds();

}

void thickened(){
    // calculate the normal vector for AB
    PointDouble point_N;
    point_N.x = absolute_points[1].y - absolute_points[0].y;
    point_N.y = absolute_points[0].x - absolute_points[1].x;

    // get the absolute value of N to normalize and get
    // it to the length of the costmap resolution
    double abs_N = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
    point_N.x = point_N.x / abs_N * _costmap_resolution;
    point_N.y = point_N.y / abs_N * _costmap_resolution;

    // calculate the new points to get a polygon which can be filled
    PointDouble point;
    point.x = absolute_points[0].x + point_N.x;
    point.y = absolute_points[0].y + point_N.y;
    absolute_points.push_back(point);

    point.x = absolute_points[1].x + point_N.x;
    point.y = absolute_points[1].y + point_N.y;
    absolute_points.push_back(point);
}

void computeMapBounds(){
    //reset bounds
    _min_x = _min_y = _max_x = _max_y = 0;

    for (int i = 0; i < absolute_points.size(); ++i){
        double px = absolute_points[i].x;
        double py = absolute_points[i].y;
        _min_x = std::min(px, _min_x);
        _min_y = std::min(py, _min_y);
        _max_x = std::max(px, _max_x);
        _max_y = std::max(py, _max_y);
    }

}

void FusionLayer::onInitialize(){ 
#if DEBUG
    ROS_INFO("fusion_layers in DEBUG mode!!");
#endif

    ros::NodeHandle ped_nh("fusion_data");
    pedestrian_sub_ = ped_nh.subscribe<std_msgs::String>("/chatter7" ,1000, dataSplit);
    ROS_INFO("fusion_data nodehandle");
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&FusionLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    costmap_2d::Costmap2D *costmap = layered_costmap_->getCostmap();
    _costmap_resolution = costmap->getResolution();

    _min_x = _min_y = _max_x = _max_y = 0;
}

void FusionLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
}

void FusionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y){

    if (!enabled_)
        return;

    //std::lock_guard<std::mutex> l(_data_mutex);

    robot_px = robot_x;
    robot_py = robot_y;
    robot_pyaw = robot_yaw;

    if (absolute_points.empty()){
        return;
    }
    *min_x = std::min(*min_x, _min_x);
    *min_y = std::min(*min_y, _min_y);
    *max_x = std::max(*max_x, _max_x);
    *max_y = std::max(*max_y, _max_y);
}

void FusionLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
    if (!enabled_)
        return;
    
    //std::lock_guard<std::mutex> l(_data_mutex);

    setPolygonCost(master_grid, absolute_points, LETHAL_OBSTACLE, min_i, min_j, max_i, max_j);
}

void FusionLayer::setPolygonCost(costmap_2d::Costmap2D &master_grid, const std::vector<PointDouble>& polygon, unsigned char cost,
                                             int min_i, int min_j, int max_i, int max_j){

    std::vector<PointInt> map_polygon;
    for (unsigned int i = 0; i < polygon.size(); ++i)    {
        PointInt loc;
        master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x, loc.y);
        map_polygon.push_back(loc);
    }

    std::vector<PointInt> polygon_cells;

    // get the cells that fill the polygon
    rasterizePolygon(map_polygon, polygon_cells);

    // set the cost of those cells
    for (unsigned int i = 0; i < polygon_cells.size(); ++i){
        int mx = polygon_cells[i].x;
        int my = polygon_cells[i].y;
        // check if point is outside bounds
        if (mx < min_i || mx >= max_i){
            continue;
        }
        if (my < min_j || my >= max_j){
            continue;
        }
        master_grid.setCost(mx, my, cost);
    }
}

void FusionLayer::rasterizePolygon(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells){
    // this implementation is a slighly modified version of Costmap2D::convexFillCells(...)

    //we need a minimum polygon of a traingle
    if(polygon.size() < 3)
        return;

    //first get the cells that make up the outline of the polygon
    polygonOutlineCells(polygon, polygon_cells);

    //quick bubble sort to sort points by x
    PointInt swap;
    unsigned int i = 0;
    while(i < polygon_cells.size() - 1)    {
        if(polygon_cells[i].x > polygon_cells[i + 1].x){
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i + 1];
            polygon_cells[i + 1] = swap;

            if(i > 0){
                --i;
            }
        }else{
            ++i;
        }
    }

    i = 0;
    PointInt min_pt;
    PointInt max_pt;
    int min_x = polygon_cells[0].x;
    int max_x = polygon_cells[(int)polygon_cells.size() -1].x;

    //walk through each column and mark cells inside the polygon
    for(int x = min_x; x <= max_x; ++x){
        if(i >= (int)polygon_cells.size() - 1){
            break;
        }

        if(polygon_cells[i].y < polygon_cells[i + 1].y)        {
            min_pt = polygon_cells[i];
            max_pt = polygon_cells[i + 1];
        }else{
            min_pt = polygon_cells[i + 1];
            max_pt = polygon_cells[i];
        }

        i += 2;
        while(i < polygon_cells.size() && polygon_cells[i].x == x){
            if(polygon_cells[i].y < min_pt.y){
                min_pt = polygon_cells[i];
            }else if(polygon_cells[i].y > max_pt.y){
                max_pt = polygon_cells[i];
            }
            ++i;
        }

        PointInt pt;
        //loop though cells in the column
        for(int y = min_pt.y; y < max_pt.y; ++y){
            pt.x = x;
            pt.y = y;
            polygon_cells.push_back(pt);
        }
    }
}


void FusionLayer::polygonOutlineCells(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells){
    for (unsigned int i = 0; i < polygon.size() - 1; ++i){
        raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
    }
    if (!polygon.empty()){
        unsigned int last_index = polygon.size() - 1;
        // we also need to close the polygon by going from the last point to the first
        raytrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
    }
}

void FusionLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells){
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    PointInt pt;
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;
        
    for (; n > 0; --n){
        cells.push_back(pt);

        if (error > 0){
            pt.x += x_inc;
            error -= dy;
        }else{
            pt.y += y_inc;
            error += dx;
        }
    }
}
} // end namespace
