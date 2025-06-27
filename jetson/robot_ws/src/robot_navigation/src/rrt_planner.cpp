/*
 * @copyright Copyright (C) 2017, Jessica Howard
 * @author Jessica Howard
 * @file rrt_planning/src/rrt_planning.cc
 *
 * @brief TODO
 * @detail TODO
 *
 *
 *
 * @license 3-Clause BSD License
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 #include <random>
 #include <tf/transform_datatypes.h>
 #include <geometry_msgs/PoseStamped.h>      // para geometry_msgs::PoseStamped
 #include <pluginlib/class_list_macros.h>
 #include "turtlebot3_2dnav/PlanningMetrics.h"
 #include "/home/vinicius/catkin_ws/src/turtlebot3_2dnav/include/rrt_planner/rrt_planner.h"
 #include "/home/vinicius/catkin_ws/src/turtlebot3_2dnav/include/rrt_planner/vertex.h"
 
 using geometry_msgs::PoseStamped;
 
 // Register as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(rrt_planning::RRTPlanner, nav_core::BaseGlobalPlanner)
 
namespace rrt_planning {
   RRTPlanner::RRTPlanner()
     : costmap_ros_(nullptr), initialized_(false) { }
 
   RRTPlanner::RRTPlanner(std::string name,
                          costmap_2d::Costmap2DROS* costmap_ros)
     : costmap_ros_(costmap_ros) {
     initialize(name, costmap_ros);
   }
 
   void RRTPlanner::initialize(std::string name,
                               costmap_2d::Costmap2DROS* costmap_ros) {
     if (!initialized_) {
       // Initialize map
       costmap_ros_ = costmap_ros;
       costmap_ = costmap_ros->getCostmap();
 
       // Initialize node handle
       ros::NodeHandle node("~/rrt_planning");
       node_handle_ = node;
       world_model_ = new base_local_planner::CostmapModel(*costmap_);
 
      // Supondo que node_handle_ seja o NodeHandle privado (por exemplo, "~rrt_planning")
      if (!node_handle_.getParam("step_size", step_size_))
        step_size_ = 0.2;
      if (!node_handle_.getParam("delta", delta_))
        delta_ = 0.3;
      if (!node_handle_.getParam("goal_radius", goal_radius_))
        goal_radius_ = 0.2;
      if (!node_handle_.getParam("max_iterations", max_iterations_))
        max_iterations_ = 200000;

       ROS_INFO("Step size: %.2f, goal radius: %.2f, delta: %.2f, max "


        
                "iterations: %d", step_size_, goal_radius_, delta_,
                max_iterations_);
       current_iterations_ = 0;
 
       // Get obstacles in the costmap
       map_width_cells_ = costmap_-> getSizeInCellsX();
       map_height_cells_ = costmap_-> getSizeInCellsY();
 
       for (unsigned int iy = 0; iy < map_height_cells_; iy++) {
         for (unsigned int ix = 0; ix < map_width_cells_; ix++) {
           unsigned char cost = static_cast<int>(costmap_->getCost(ix, iy));
           if (cost >= 115)
             obstacle_map_.push_back(false);
           else
             obstacle_map_.push_back(true);
         }
       }
 
       // Display info message
       ROS_INFO("RRT planner initialized successfully.");
       metrics_pub_ = node_handle_.advertise<turtlebot3_2dnav::PlanningMetrics>("planning_metrics", 1);
       initialized_ = true;
     } else {
       ROS_WARN("RRT planner has already been initialized.");
     }
   }
 
   bool RRTPlanner::makePlan(const PoseStamped& start, const PoseStamped& goal, std::vector<PoseStamped>& plan)
  { 
    // 1) mede o tempo
    ros::Time t0 = ros::Time::now();
    int goal_index = FindPath(start, goal);
    double planning_time = (ros::Time::now() - t0).toSec();

    // 2) reconstrói o plan
    plan = BuildPlan(goal_index, start, goal);
    if (plan.size() <= 1)
      return false;

    // 3) calcula comprimento
    double length = 0.0;
    for (size_t i = 1; i < plan.size(); ++i) {
      double dx = plan[i].pose.position.x - plan[i-1].pose.position.x;
      double dy = plan[i].pose.position.y - plan[i-1].pose.position.y;
      length += std::hypot(dx, dy);
    }

    // 4) conta colisões previstas
    uint32_t collisions = 0;
    for (size_t i = 1; i < plan.size(); ++i) {
      auto& A = plan[i-1];
      auto& B = plan[i];
      double dx = B.pose.position.x - A.pose.position.x;
      double dy = B.pose.position.y - A.pose.position.y;
      double dist = std::hypot(dx, dy);
      double theta = std::atan2(dy, dx);
      for (double d = 0.0; d < dist; d += delta_) {
        double x = A.pose.position.x + d * std::cos(theta);
        double y = A.pose.position.y + d * std::sin(theta);
        unsigned int mx, my;
        if (costmap_->worldToMap(x,y,mx,my)) {
        unsigned char c = costmap_->getCost(mx,my);
          if (c >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            ++collisions;
            break;
          }
        }
      }
    }

    // 5) mede suavidade (variação angular)
    double smoothness = 0.0;
    auto angle = [&](int i){
    auto& A = plan[i-2];
    auto& B = plan[i-1];
    auto& C = plan[i];
    double ux = B.pose.position.x - A.pose.position.x;
    double uy = B.pose.position.y - A.pose.position.y;
    double vx = C.pose.position.x - B.pose.position.x;
    double vy = C.pose.position.y - B.pose.position.y;
    double nu = std::hypot(ux,uy), nv = std::hypot(vx,vy);
    if (nu<1e-6||nv<1e-6) return 0.0;
      double cosang = (ux*vx + uy*vy)/(nu*nv);
      cosang = std::min(1.0, std::max(-1.0, cosang));
      return std::acos(cosang);
    };
    for (size_t i = 2; i < plan.size(); ++i) {
      smoothness += std::abs(angle(i));
    }

    // 6) monta e publica a mensagem
    turtlebot3_2dnav::PlanningMetrics m;
    m.planning_time   = planning_time;
    m.path_length     = length;
    m.collision_count = collisions;
    m.smoothness      = smoothness;
    metrics_pub_.publish(m);

    return true;
  }

 
   std::pair<float, float> RRTPlanner::GetRandomPoint() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> prob(0, 1);
    std::pair<float, float> random_point;
    
    // Defina uma probabilidade para usar o goal como amostra
    float goal_bias = 0.1; // 10% de chance
    if(prob(gen) < goal_bias) {
        random_point.first = x_goal_;
        random_point.second = y_goal_;
    } else {
        float origin_x = costmap_->getOriginX();
        float origin_y = costmap_->getOriginY();
        std::uniform_real_distribution<> x_dist(origin_x, origin_x + costmap_->getSizeInMetersX());
        std::uniform_real_distribution<> y_dist(origin_y, origin_y + costmap_->getSizeInMetersY());
        
        random_point.first = x_dist(gen);
        random_point.second = y_dist(gen);
    }
    return random_point;
  }
 
  int RRTPlanner::FindPath(const geometry_msgs::PoseStamped& start,
                         const geometry_msgs::PoseStamped& goal)
  {
    /********************  reinicialização do estado  ********************/
    vertex_list_.clear();
    current_iterations_ = 0;

    /* 1. adiciona o nó-raiz (start) — índice 0, sem pai (-1) */
    vertex_list_.emplace_back(start.pose.position.x,
                              start.pose.position.y,
                              0,          /* index   */
                              -1);        /* parent  */

    /* 2. grava as coordenadas do goal para o bias de amostragem         */
    x_goal_ = goal.pose.position.x;
    y_goal_ = goal.pose.position.y;

    /*********************************************************************/
    bool done       = false;
    int  goal_index = -1;

    /* 3. laço principal: expande até achar o alvo ou atingir o limite   */
    while (!done && current_iterations_ < max_iterations_)
    {
      /* ponto aleatório (com viés para o objetivo)                      */
      std::pair<float,float> rand_pt = GetRandomPoint();

      /* vértice mais próximo já existente                               */
      int closest = GetClosestVertex(rand_pt);
      if (closest < 0)                  // salvaguarda extra
        continue;                       // (não deve acontecer)

      /* tenta estender da árvore até rand_pt                            */
      if (MoveTowardsPoint(closest, rand_pt))
      {
        ++current_iterations_;

        /* novo vértice acabou de entrar na lista                         */
        int new_vertex = vertex_list_.back().get_index();
        done = ReachedGoal(new_vertex);

        if (done)
        {
          ROS_INFO("Goal alcançado! índice do vértice final: %d", new_vertex);
          goal_index = new_vertex;
        }
      }
    }

    if (!done)
      ROS_INFO("Limite de iterações atingido: plano não encontrado.");

    return goal_index;   // −1 caso não exista caminho
  }

 
   int RRTPlanner::GetClosestVertex(std::pair<float, float> random_point) {
     int closest = -1;
 
     // closest_distance will keep track of the closest distance we find
     float closest_distance = std::numeric_limits<float>::infinity();
 
     // current_distance will keep track of the distance of the current
     float current_distance = std::numeric_limits<float>::infinity();
 
     // iterate through the vertex list to find the closest
     for (rrt_planning::Vertex v : vertex_list_) {
       current_distance = GetDistance(v.get_location(), random_point);
 
       // If the current distance is closer than what was previously
       // saved, update
       if (current_distance < closest_distance) {
         ROS_DEBUG("Closest distance: %.5f, vertex: %d.",
                   current_distance, v.get_index());
         closest = v.get_index();
         closest_distance = current_distance;
       }
     }
     return closest;
   }
 
   float RRTPlanner::GetDistance(std::pair<float, float> start_point,
                                  std::pair<float, float> end_point) {
     // coords for our first point
     float x1 = start_point.first;
     float y1 = start_point.second;
 
     // coords for our second point
     float x2 = end_point.first;
     float y2 = end_point.second;
 
     // euclidean distance
     float distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
 
     ROS_DEBUG("Distance: %.5f", distance);
     return distance;
   }
 
   bool RRTPlanner::MoveTowardsPoint(int closest_vertex,
                                     std::pair<float, float> random_point) {
     ROS_DEBUG("In MoveTowardsPoint");
     float x_closest = vertex_list_.at(closest_vertex).get_location().first;
     float y_closest = vertex_list_.at(closest_vertex).get_location().second;
     float x_random = random_point.first;
     float y_random = random_point.second;
 
     // get the angle between the random point and our closest point (in rads)
     float theta = atan2(y_random - y_closest, x_random - x_closest);
 
     // proposed new point step_size_ from our closest vertex towards
     // the random point
     float new_x = x_closest + step_size_ * cos(theta);
     float new_y = y_closest + step_size_ * sin(theta);
 
     std::pair<float, float> proposed_point(new_x, new_y);
     std::pair<float, float> closest_point(x_closest, y_closest);
 
     // Check if the path between closest_vertex and the new point
     // is safe
     if (IsSafe(closest_point, proposed_point)) {
       // If safe, add new Vertex to the back of vertex_list_
       rrt_planning::Vertex new_vertex(new_x, new_y, vertex_list_.size(),
                                        closest_vertex);
       ROS_DEBUG("Added new vertex at: %.5f, %.5f, index: %d",
                new_x, new_y, new_vertex.get_index());
       addVertex(new_vertex);
 
       // Return true, that we moved towards the proposed point
       return true;
     }
     // Return false, move not made
     return false;
   }
 
   bool RRTPlanner::IsSafe(std::pair<float, float> start_point,
    std::pair<float, float> end_point) {
      unsigned int map_x, map_y;

      // Verifica se o ponto final é válido
      if (!costmap_->worldToMap(end_point.first, end_point.second, map_x, map_y)) {
        ROS_WARN("worldToMap falhou para o ponto final: %.2f, %.2f", end_point.first, end_point.second);
        return false;
      }

      // Correção na indexação: utiliza map_width_cells_ no cálculo
      if (!obstacle_map_.at(map_y * map_width_cells_ + map_x))
        return false;

      // Checa o caminho em intervalos de delta para colisões
      float theta = atan2(end_point.second - start_point.second,
      end_point.first - start_point.first);
      float current_x = start_point.first;
      float current_y = start_point.second;

      while (GetDistance(std::pair<float, float>(current_x, current_y), end_point) > delta_) {
      current_x += delta_ * cos(theta);
      current_y += delta_ * sin(theta);

      // Converte coordenadas do mundo para as do mapa
      if (!costmap_->worldToMap(current_x, current_y, map_x, map_y)) {
        ROS_WARN("worldToMap falhou durante a checagem do caminho: %.2f, %.2f", current_x, current_y);
        return false;
      }

      if (!obstacle_map_.at(map_y * map_width_cells_ + map_x))
        return false;
      }
        return true;
    }

 
   bool RRTPlanner::ReachedGoal(int new_vertex) {
     ROS_DEBUG("In ReachedGoal, vertex index: %d.", new_vertex);
 
     // save our goal and current location as pairs
     std::pair<float, float> goal(x_goal_, y_goal_);
     std::pair<float, float> current_location;
     current_location.first =
       vertex_list_.at(new_vertex).get_location().first;
     current_location.second =
       vertex_list_.at(new_vertex).get_location().second;
 
     ROS_DEBUG("cx: %.5f, cy: %.5f, gx: %.5f, gy: %.5f",
               current_location.first,
               current_location.second,
               goal.first,
               goal.second);
 
     // Check distance between current point and goal, if distance is less
     // than goal_radius_ return true, otherwise return false
     float distance = GetDistance(current_location, goal);
     ROS_DEBUG("Distance to goal: %.5f", distance);
 
     if (distance <= goal_radius_)
       return true;
     else
       return false;
   }
 
   std::vector<geometry_msgs::PoseStamped>
     RRTPlanner::BuildPlan(int goal_index,
                            const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal) {
       ROS_INFO("Building the plan.");
 
       // reset our current iterations
       current_iterations_ = 0;
 
       // The plan we'll be adding to and returning
       std::vector<geometry_msgs::PoseStamped> plan;
 
       // no plan found
       if (goal_index == -1)
         return plan;
 
       // The list of vertex indices we pass through to get to the goal
       std::deque<int> index_path;
       int current_index = goal_index;
       while (current_index > 0) {
         index_path.push_front(current_index);
         current_index = vertex_list_.at(current_index).get_parent();
       }
       index_path.push_front(0);
 
       // build the plan back up in PoseStamped messages
       for (int i : index_path) {
         if (i == 0) {
           plan.push_back(start);
         } else {
           geometry_msgs::PoseStamped pos;
 
           pos.pose.position.x = vertex_list_.at(i).get_location().first;
           pos.pose.position.y = vertex_list_.at(i).get_location().second;
           pos.pose.position.z = 0.0;
 
           pos.pose.orientation = tf::createQuaternionMsgFromYaw(0);
           plan.push_back(pos);
         }
       }
       plan.push_back(goal);
       unsigned int map_x, map_y;
       for (geometry_msgs::PoseStamped p : plan) {
         costmap_->worldToMap(p.pose.position.x, p.pose.position.y,
                              map_x, map_y);
         ROS_INFO("x: %.2f (%d), y: %.2f (%d)", p.pose.position.x,
                                                map_x,
                                                p.pose.position.y,
                                                map_y);
       }
       return plan;
   }
 };  // namespace rrt_planning