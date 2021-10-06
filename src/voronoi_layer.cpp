#include <pluginlib/class_list_macros.h>
#include <chrono>
#include "voronoi_layer/voronoi_layer.h"

PLUGINLIB_EXPORT_CLASS(costmap_2d::VoronoiLayer, costmap_2d::Layer)

namespace costmap_2d
{
VoronoiLayer::VoronoiLayer() : initialized_(false)
{
}

VoronoiLayer::~VoronoiLayer()
{
}

void VoronoiLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  voronoi_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_grid", 1);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb =
      boost::bind(&VoronoiLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void VoronoiLayer::reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level)
{
  enabled_ = config.enabled;
}

void VoronoiLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
  if (!enabled_)
    return;
}

void VoronoiLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  int size_x = master_grid.getSizeInCellsX();
  int size_y = master_grid.getSizeInCellsY();
  if (!initialized_)
  {
    voronoi_.initializeEmpty(size_x, size_y);
    initialized_ = true;
  }

  std::vector<IntPoint> new_free_cells, new_occupied_cells;
  for (int j = 0; j < size_y; j++)
  {
    for (int i = 0; i < size_x; i++)
    {
      if (voronoi_.isOccupied(i, j) && master_grid.getCost(i, j) == FREE_SPACE)
        new_free_cells.push_back(IntPoint(i, j));

      if (!voronoi_.isOccupied(i, j) && master_grid.getCost(i, j) == LETHAL_OBSTACLE)
        new_occupied_cells.push_back(IntPoint(i, j));
    }
  }

  for (int i = 0; i < (int)new_free_cells.size(); i++)
    voronoi_.clearCell(new_free_cells[i].x, new_free_cells[i].y);

  for (int i = 0; i < (int)new_occupied_cells.size(); i++)
    voronoi_.occupyCell(new_occupied_cells[i].x, new_occupied_cells[i].y);

  // start timing
  const auto start_t = std::chrono::system_clock::now();

  voronoi_.update();
  voronoi_.prune();

  // end timing
  const auto end_t = std::chrono::system_clock::now();
  std::chrono::duration<double> timediff = end_t - start_t;
  ROS_INFO("Runtime=%.3fms.", timediff.count() * 1e3);

  publishVoronoiGrid(master_grid);
}

void VoronoiLayer::publishVoronoiGrid(costmap_2d::Costmap2D& master_grid)
{
  int nx = master_grid.getSizeInCellsX(), ny = master_grid.getSizeInCellsY();

  double resolution = master_grid.getResolution();
  nav_msgs::OccupancyGrid grid;
  // Publish Whole Grid
  grid.header.frame_id = "map";
  grid.header.stamp = ros::Time::now();
  grid.info.resolution = resolution;

  grid.info.width = nx;
  grid.info.height = ny;

  grid.info.origin.position.x = master_grid.getOriginX();
  grid.info.origin.position.y = master_grid.getOriginY();
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(nx * ny);

  for (unsigned int x = 0; x < nx; x++)
  {
    for (unsigned int y = 0; y < ny; y++)
    {
      if (voronoi_.isVoronoi(x, y))
        grid.data[x + y * nx] = 128;
      else
        grid.data[x + y * nx] = 0;
    }
  }
  voronoi_grid_pub_.publish(grid);
}

}  // namespace costmap_2d