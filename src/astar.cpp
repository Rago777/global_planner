/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;

        int x = i % nx_, y = i / nx_;
        
        // 8-neighbor expansion
        add(costs, potential, potential[i], i + 1, end_x, end_y, 1.0);
        add(costs, potential, potential[i], i - 1, end_x, end_y, 1.0);
        add(costs, potential, potential[i], i + nx_, end_x, end_y, 1.0);
        add(costs, potential, potential[i], i - nx_, end_x, end_y, 1.0);
        add(costs, potential, potential[i], i + nx_ + 1, end_x, end_y, std::sqrt(2.0));
        add(costs, potential, potential[i], i + nx_ - 1, end_x, end_y, std::sqrt(2.0));
        add(costs, potential, potential[i], i - nx_ + 1, end_x, end_y, std::sqrt(2.0));
        add(costs, potential, potential[i], i - nx_ - 1, end_x, end_y, std::sqrt(2.0));

        cycle++;
    }

    return false;
}

void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y, float movement_cost) {
    if (next_i < 0 || next_i >= ns_)
        return;

    if (potential[next_i] < POT_HIGH)
        return;

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;
    int dx = abs(end_x - x), dy = abs(end_y - y);
    float octile_distance = std::max(dx, dy) + (std::sqrt(2.0) - 1) * std::min(dx, dy);

    // Compute dynamic heuristic weight
    float heuristic_weight = computeDynamicWeight(costs, next_i, end_x, end_y);
    std::cout << "Node: (" << x << ", " << y << ") robs: " << heuristic_weight << std::endl;

    queue_.push_back(Index(next_i, potential[next_i] + heuristic_weight * octile_distance * neutral_cost_));
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

float AStarExpansion::computeDynamicWeight(unsigned char* costs, int current_i, int end_x, int end_y) {
    int radius = 3; // Increase neighborhood radius for better smoothing
    int obstacle_count = 0;
    int total_count = 0;
    int x = current_i % nx_, y = current_i / nx_;
    int dx_dir = (end_x > x) ? 1 : -1;
    int dy_dir = (end_y > y) ? 1 : -1;

    for (int dx = 0; dx <= radius; dx++) {
        for (int dy = 0; dy <= radius; dy++) {
            int nx = x + dx * dx_dir, ny = y + dy * dy_dir;
            if (nx >= 0 && nx < nx_ && ny >= 0 && ny < ny_) {
                int neighbor_i = ny * nx_ + nx;
                total_count++;
                if (costs[neighbor_i] > 200) { // Adjust threshold to avoid false positives
                    obstacle_count++;
                }
            }
        }
    }
    
    float robs = (total_count > 0) ? (float)obstacle_count / total_count : 0;
    float sigma_obs = 0.3; // Reduce sigma_obs to prevent excessive weight reduction
    float omega = 0.75 + 0.5 * (1 - robs); // Adjust range from 0.75 to 1.25 for aggressive search
    return std::max(0.75f, std::min(1.25f, omega)); // Ensure omega remains in [0.75,1.25]
}

} //end namespace global_planner
