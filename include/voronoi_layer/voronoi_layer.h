#ifndef _VORONOI_LAYER_H_
#define _VORONOI_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include "voronoi_layer/dynamicvoronoi.h"

namespace costmap_2d
{
class VoronoiLayer : public Layer
{
public:
  VoronoiLayer();
  virtual ~VoronoiLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void publishVoronoiGrid(costmap_2d::Costmap2D& master_grid);

  void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
  ros::Publisher voronoi_grid_pub_;

  DynamicVoronoi voronoi_;
  bool initialized_;
};

}  // namespace costmap_2d

#endif