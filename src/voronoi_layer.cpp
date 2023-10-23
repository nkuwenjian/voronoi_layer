/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "voronoi_layer/voronoi_layer.h"

#include <chrono>  // NOLINT

#include "glog/logging.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(costmap_2d::VoronoiLayer, costmap_2d::Layer)

namespace costmap_2d {

void VoronoiLayer::onInitialize() {
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  voronoi_grid_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("voronoi_grid", 1);

  dsrv_ = std::make_unique<
      dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType
      cb = boost::bind(&VoronoiLayer::ReconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void VoronoiLayer::ReconfigureCB(const costmap_2d::GenericPluginConfig& config,
                                 uint32_t level) {
  enabled_ = config.enabled;
}

void VoronoiLayer::updateBounds(double robot_x, double robot_y,
                                double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y) {}

bool VoronoiLayer::OutlineMap(const costmap_2d::Costmap2D& master_grid,
                              uint8_t value) {
  uint8_t* char_map = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  if (char_map == nullptr) {
    LOG(ERROR) << "char_map == nullptr";
    return false;
  }

  uint8_t* pc = char_map;
  for (unsigned int i = 0U; i < size_x; ++i) {
    *pc++ = value;
  }
  pc = char_map + (size_y - 1U) * size_x;
  for (unsigned int i = 0U; i < size_x; ++i) {
    *pc++ = value;
  }
  pc = char_map;
  for (unsigned int i = 0U; i < size_y; ++i, pc += size_x) {
    *pc = value;
  }
  pc = char_map + size_x - 1U;
  for (unsigned int i = 0U; i < size_y; ++i, pc += size_x) {
    *pc = value;
  }
  return true;
}

void VoronoiLayer::UpdateDynamicVoronoi(
    const costmap_2d::Costmap2D& master_grid) {
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  if (last_size_x_ != size_x || last_size_y_ != size_y) {
    voronoi_.initializeEmpty(size_x, size_y);

    last_size_x_ = size_x;
    last_size_y_ = size_y;
  }

  std::vector<IntPoint> new_free_cells;
  std::vector<IntPoint> new_occupied_cells;
  for (unsigned int j = 0; j < size_y; ++j) {
    for (unsigned int i = 0; i < size_x; ++i) {
      if (voronoi_.isOccupied(i, j) &&
          master_grid.getCost(i, j) == costmap_2d::FREE_SPACE) {
        new_free_cells.push_back(IntPoint(i, j));
      }

      if (!voronoi_.isOccupied(i, j) &&
          master_grid.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE) {
        new_occupied_cells.push_back(IntPoint(i, j));
      }
    }
  }

  for (std::size_t i = 0U; i < new_free_cells.size(); ++i) {
    voronoi_.clearCell(new_free_cells[i].x, new_free_cells[i].y);
  }

  for (std::size_t i = 0U; i < new_occupied_cells.size(); ++i) {
    voronoi_.occupyCell(new_occupied_cells[i].x, new_occupied_cells[i].y);
  }
}

void VoronoiLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                               int min_j, int max_i, int max_j) {
  if (!enabled_) {
    LOG(ERROR) << "VoronoiLayer is disable.";
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!OutlineMap(master_grid, costmap_2d::LETHAL_OBSTACLE)) {
    LOG(ERROR) << "Failed to outline map.";
    return;
  }

  UpdateDynamicVoronoi(master_grid);

  // Start timing.
  const auto start_timestamp = std::chrono::system_clock::now();

  voronoi_.update();
  voronoi_.prune();

  // End timing.
  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  VLOG(4) << std::fixed << "Runtime=" << diff.count() * 1e3 << "ms.";

  PublishVoronoiGrid(master_grid);
}

void VoronoiLayer::PublishVoronoiGrid(
    const costmap_2d::Costmap2D& master_grid) {
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();
  double resolution = master_grid.getResolution();
  double origin_x = master_grid.getOriginX();
  double origin_y = master_grid.getOriginY();

  // Publish whole grid.
  nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.header.stamp = ros::Time::now();
  grid.info.resolution = resolution;

  grid.info.width = size_x;
  grid.info.height = size_y;

  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(size_x * size_y);

  for (unsigned int x = 0; x < size_x; ++x) {
    for (unsigned int y = 0; y < size_y; ++y) {
      if (voronoi_.isVoronoi(x, y)) {
        grid.data[x + y * size_x] = 128;
      } else {
        grid.data[x + y * size_x] = 0;
      }
    }
  }
  voronoi_grid_pub_.publish(grid);
}

}  // namespace costmap_2d
