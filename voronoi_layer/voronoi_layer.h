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

#pragma once

#include <memory>
#include <mutex>  // NOLINT

#include "costmap_2d/GenericPluginConfig.h"
#include "costmap_2d/cost_values.h"
#include "costmap_2d/layer.h"
#include "costmap_2d/layered_costmap.h"
#include "dynamic_reconfigure/server.h"
#include "dynamicvoronoi/dynamicvoronoi.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

namespace costmap_2d {

class VoronoiLayer : public Layer {
 public:
  VoronoiLayer() = default;
  ~VoronoiLayer() override = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                   int max_i, int max_j) override;

  const DynamicVoronoi& voronoi() const { return voronoi_; }
  std::mutex& mutex() { return mutex_; }

 private:
  static bool OutlineMap(const costmap_2d::Costmap2D& master_grid,
                         uint8_t value);
  void UpdateDynamicVoronoi(const costmap_2d::Costmap2D& master_grid);
  void ReconfigureCB(const costmap_2d::GenericPluginConfig& config,
                     uint32_t level);
  std::unique_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>>
      dsrv_ = nullptr;

  DynamicVoronoi voronoi_;
  unsigned int last_size_x_ = 0;
  unsigned int last_size_y_ = 0;
  std::mutex mutex_;
};

}  // namespace costmap_2d
