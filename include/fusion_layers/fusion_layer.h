#ifndef FUSION_LAYER_H_
#define FUSION_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <string>



namespace fusion_layer_namespace
{
void split(char *src,const char *separator, std::vector<std::string> &dest);
void dataSplit(const std_msgs::String::ConstPtr& msg);
void thickened();
void computeMapBounds();

struct PointDouble{
    double x;
    double y;
};

struct PointInt{
    int x;
    int y;
};

double robot_px;
double robot_py;
double robot_pyaw;

std::vector<PointDouble> related_points; //related points
std::vector<PointDouble> absolute_points; //absolute points
std::vector<std::string> split_result;

std::mutex _data_mutex;

//compute bounds
double _min_x, _min_y, _max_x, _max_y; 
double _costmap_resolution=0.05; 

double pi = 3.14159265359;

std::string CLEAR = "clear";
std::string NONE = "none";

class FusionLayer : public costmap_2d::Layer
{
public:
    FusionLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

    void setPolygonCost(costmap_2d::Costmap2D &master_grid, const std::vector<PointDouble>& polygon, unsigned char cost,
                                                int min_i, int min_j, int max_i, int max_j);
    void rasterizePolygon(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells);
    void polygonOutlineCells(const std::vector<PointInt>& polygon, std::vector<PointInt>& polygon_cells);
    void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells);

    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_; 
    ros::Subscriber pedestrian_sub_;
    
};
}
#endif
