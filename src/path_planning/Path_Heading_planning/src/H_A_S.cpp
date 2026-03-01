#include "H_A_S.h"

using namespace std;
  H_A_Star::H_A_Star(std::shared_ptr<ElectricVehicleDynamicsModel::VehicleParams> vehicle_param,
                          std::shared_ptr<Params> solver_param,
                          std::shared_ptr<Cost_weight> cost_param)
      : vehicle_params_(vehicle_param ? vehicle_param : std::make_shared<ElectricVehicleDynamicsModel::VehicleParams>()),
        params_(solver_param ? solver_param : std::make_shared<Params>()),
        cost_weights_(cost_param ? cost_param : std::make_shared<Cost_weight>()) {


  }

  bool H_A_Star::init_plan(WorldMap& map)
  {
    open_set_.clear();
    close_set_.clear();
    open_pq_ = decltype(open_pq_)();
    final_node_ = nullptr;

    //对起点与终点进行碰撞检测
    if (!ValidityCheck(start_node_)) {
      std::cout << "起点处有障碍物";
      return false;
    }
    if (!ValidityCheck(end_node_)) {
      std::cout << "终点处有障碍物";
      return false;
    }
    //RS曲线生成器
    reedsShepp_=std::shared_ptr<RSCurve>(new RSCurve());
    //dijkstra求解器

    //初始化搜索位姿
    Pos3d start_pose=map.get_start_pos(),end_pose=map.get_goal_pos();
    //初始化栅格地图
    ObstacleGridMap* grid_map=new ObstacleGridMap(map,params_->dijkstra_grid_resolution,params_->robot_radius);
    //设置混合A*的终点当dijkstra的搜索起点，dijkstra的终点设置在地图外（保证得到终点到任意点的最短距离）
    grid_map->setstr(end_pose.x,end_pose.y);
    grid_map->setgoal(-1,-1);
    grid_map->plotWorldMap();
    plt::show();
    Dijk_solver_=std::shared_ptr<Dijkstra>(new Dijkstra(grid_map,false,false,false));
    Dijk_solver_->planning();

    //障碍物数据类型转换
    obstacle_import(map);

    //私有变量初始化
    XYbounds_ =map.getBounds();
      //reset先对旧对象引用计数减1（如果引用计数为0就delete），再new新对象，将新创建的Node3d对象的所有权转移给 start_node_智能指针，
    //引用计数减1，原来的指针由其他智能指针继续管理（也就是这个变量名现在管理新的指针，原来指针由其他引用的变量名管理）
    start_node_.reset(new Node3d({start_pose.x}, {start_pose.y}, {start_pose.phi},
                                XYbounds_,
                                params_->dijkstra_grid_resolution));
    end_node_.reset(new Node3d({end_pose.x}, {end_pose.y}, {end_pose.phi},
                              XYbounds_,
                              params_->dijkstra_grid_resolution));

    init_check_=false;
    return true;
  }

  void H_A_Star::obstacle_import(WorldMap& map)
  {
    std::vector<WorldMap::Obstacle> obs=map.get_Obstacle();
    //初始化障碍物容器
    std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec;
    for (const auto& obstacle : obs) {
        obstacles_linesegments_vec.emplace_back(math::computeRectangleEdges(obstacle.x,obstacle.y,obstacle.width,obstacle.height,obstacle.rotation));
    }
    //数据转交，转移后 obstacles_linesegments_vec变为无效状态，不用拷贝，相当于指针转交，不用拷贝
    obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

  }

  std::vector<std::vector<double>> H_A_Star::planning(WorldMap& map)
  {
    //初始化搜索位姿
    Pos3d start_pose_=map.get_start_pos(),end_pose_=map.get_goal_pos();


    //初始化规划器
    if(!init_plan(map))
    {
      return {};
    }
    //std::cout<<"初始化完成"<<std::endl;

    //一个放节点，一个放代价，节点索引一致
    open_set_.emplace(start_node_->GetIndex(), start_node_);
    open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

    int num_=0;
    //开集有元素就不断搜索
    while (!open_pq_.empty()) {
      //拿出开集里代价最小的节点的索引
      const std::string current_id = open_pq_.top().first;
      open_pq_.pop();
      //取出节点
      std::shared_ptr<Node3d> current_node = open_set_[current_id];
      num_++;
      //检查是否可以用RS曲线直接从当前点到终点，如果可以，则无需探索，一次性完成
      if (AnalyticExpansion(current_node)) {
        break;
      }

      close_set_.emplace(current_node->GetIndex(), current_node);
      for (size_t i = 0; i <params_->next_node_num; ++i) {
        std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);

        if (next_node == nullptr) {
          continue;
        }

        if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
          continue;
        }

        if (!ValidityCheck(next_node)) {
          continue;
        }

        if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
          CalculateNodeCost(current_node, next_node);
          open_set_.emplace(next_node->GetIndex(), next_node);
          open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
        }

      }
    }
    std::cout<<"共搜索了"<<num_<<"个节点"<<std::endl;
    return Path_Backtracking();
  }

  std::vector<std::vector<double>> H_A_Star::Path_Backtracking()
  {
    std::shared_ptr<Node3d> current_node = final_node_;
    std::vector<double> hybrid_a_x;
    std::vector<double> hybrid_a_y;
    std::vector<double> hybrid_a_phi;
    while (current_node->GetPreNode() != nullptr) {
      std::vector<double> x = current_node->GetXs();
      std::vector<double> y = current_node->GetYs();
      std::vector<double> phi = current_node->GetPhis();
      if (x.empty() || y.empty() || phi.empty()) {
        std::cout << "result size check failed";
        return {};
      }
      if (x.size() != y.size() || x.size() != phi.size()) {
        std::cout << "states sizes are not equal";
        return {};
      }
      std::reverse(x.begin(), x.end());
      std::reverse(y.begin(), y.end());
      std::reverse(phi.begin(), phi.end());
      x.pop_back();
      y.pop_back();
      phi.pop_back();
      hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
      hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
      hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
      current_node = current_node->GetPreNode();
    }
    hybrid_a_x.push_back(current_node->GetX());
    hybrid_a_y.push_back(current_node->GetY());
    hybrid_a_phi.push_back(current_node->GetPhi());

    std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
    std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
    std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());

    std::vector<double> time;
    for(size_t i=0;i<hybrid_a_x.size()+1;++i)
    {
      time.emplace_back((double)i*params_->dt);
    }
    
    hybrid_a_x.push_back(end_node_->GetX());
    hybrid_a_y.push_back(end_node_->GetY());
    hybrid_a_phi.push_back(end_node_->GetPhi());

    return {time,hybrid_a_x,hybrid_a_y,hybrid_a_phi};
  }



  bool H_A_Star::ValidityCheck(std::shared_ptr<Node3d> node) {
  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }
  //如果接节点初始化的时候传入的是轨迹点，std::vector<double>，其size就是此轨迹点数量，节点是最后一个点
  //其他初始化size都是1
  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  //除了起点终点，都是用轨迹初始化，所以只有起点终点size是1；
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    //起点和终点进行碰撞检测
    check_start_index = 0;
  } else {
    //其余节点的第一个轨迹点都是上一个节点的最后一个轨迹点，因此跳过
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    //超出了边界
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      return false;
    }

    //规划点是车辆后轴重心点，因此需要向前移动到几何中心
    Box2d bounding_box = Node3d::GetBoundingBox(
        *vehicle_params_, traversed_x[i], traversed_y[i], traversed_phi[i]);

    //用矩形框和障碍物作碰撞检测
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const LineSegment2d& linesegment : obstacle_linesegments) {
        
        if (math::HasOverlap(bounding_box, linesegment)) {
          if(init_check_){plot_collision(bounding_box,linesegment);}
          return false;
        }
      }
    }
  }

  return true;
}

void H_A_Star::plot_collision(Box2d bounding_box,LineSegment2d linesegment)
{
  std::vector<double> obs_x={linesegment.start.x,linesegment.end.x};
  std::vector<double> obs_y={linesegment.start.y,linesegment.end.y};
  std::vector<LineSegment2d> box= math::computeRectangleEdges(bounding_box.center.x,bounding_box.center.y,bounding_box.length,bounding_box.width,bounding_box.heading);
  std::vector<double> box_x,box_y;
  for(auto line:box)
  {
    box_x.emplace_back(line.start.x);
    box_x.emplace_back(line.end.x);
    box_y.emplace_back(line.start.y);
    box_y.emplace_back(line.end.y);
  }
  plt::figure();
  plt::axis("equal");
  plt::plot(obs_x,obs_y, {{"color", "red"}}); 
  plt::plot(box_x,box_y, {{"color", "green"}}); 
  plt::show();
}

std::shared_ptr<Node3d> H_A_Star::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;
  if (next_node_index < static_cast<double>(params_->next_node_num) / 2) {
    steering =
        -vehicle_params_->max_steer +
        (2 * vehicle_params_->max_steer / (static_cast<double>(params_->next_node_num) / 2 - 1)) *
            static_cast<double>(next_node_index);
    traveled_distance = params_->step_size;
  } else {
    size_t index = next_node_index - params_->next_node_num / 2;
    steering =
        -vehicle_params_->max_steer +
        (2 * vehicle_params_->max_steer / (static_cast<double>(params_->next_node_num) / 2 - 1)) *
            static_cast<double>(index);
    traveled_distance = -params_->step_size;
  }

  //轨迹数据，记录current_node到next_node之间的轨迹点数据
  double arc = std::sqrt(2) * params_->grid_resolution;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;

  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();

  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);

  for (size_t i = 0; i < arc / params_->step_size; ++i) {
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = math::NormalizeAngle(last_phi +traveled_distance / (vehicle_params_->lf+vehicle_params_->lr) * std::tan(steering));
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }

  if (intermediate_x.back() > XYbounds_[1] ||
      intermediate_x.back() < XYbounds_[0] ||
      intermediate_y.back() > XYbounds_[3] ||
      intermediate_y.back() < XYbounds_[2]) {
    return nullptr;
  }
  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 params_->dijkstra_grid_resolution));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}

bool H_A_Star::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
  //使用RS曲线生成
  std::vector<Pos3d> curve_path=RS_generate_path(current_node->GetPose());

  //使用一般几何曲线计算
  // std::vector<Pos3d> curve_path = math::GetTrajFromCurvePathsConnect(
  //   current_node->GetPose(), end_node_->GetPose(), params_->min_radius,params_->grid_resolution);

  //判断RS曲线是否正确
  double dx=abs(curve_path.back().x-end_node_->GetPose().x),
  dy=abs(curve_path.back().y-end_node_->GetPose().y),
  dphi=abs(curve_path.back().phi-end_node_->GetPose().phi);
  if(dx>params_->step_size/2||dy>params_->step_size/2||dphi>0.1)
  {return false;}

  if (!IsPathVaild(curve_path)) {
    return false;
  }

  final_node_ = GenerateFinalNode(curve_path, current_node);
  return true;
}

std::vector<Pos3d> H_A_Star::RS_generate_path(Pos3d current_pos)
{
  vector<Pos3d> pos3d_vector;
  Pos3d end_pos=end_node_->GetPose();
  point_type start_pose(std::make_pair(current_pos.x,current_pos.y),current_pos.phi); // 起点 (0,0)，朝向 0 弧度
  point_type end_pose(std::make_pair(end_pos.x,end_pos.y), end_pos.phi); // 终点 (4,2)，朝向 π/2 弧度

  auto curve=reedsShepp_->GetBestRSCurve(params_->min_radius,start_pose,end_pose);
  auto path= reedsShepp_->GetRSPoint(curve,params_->step_size);

  for (size_t i = 0; i < path.size(); ++i) {
    Pos3d pos(path[i].first.first, path[i].first.second, path[i].second);
     pos3d_vector.push_back(pos);
  }
  return pos3d_vector;
}

bool H_A_Star::IsPathVaild(const std::vector<Pos3d>& curve_path) {
  if (curve_path.empty()) {
    return false;
  }

  if (obstacles_linesegments_vec_.empty()) {
    return true;
  }

  for (const auto& pose : curve_path) {
    if (pose.x > XYbounds_[1] || pose.x < XYbounds_[0] ||
        pose.y > XYbounds_[3] || pose.y < XYbounds_[2]) {
      return false;
    }
    Box2d bounding_box =
        Node3d::GetBoundingBox(*vehicle_params_, pose.x, pose.y, pose.phi);
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const LineSegment2d& linesegment : obstacle_linesegments) {
        if (math::HasOverlap(bounding_box, linesegment)) {
          return false;
        }
      }
    }
  }
  return true;
}

void H_A_Star::CalculateNodeCost(std::shared_ptr<Node3d> current_node,std::shared_ptr<Node3d> next_node) {

  next_node->SetTrajCost(current_node->GetTrajCost()+TrajCost(current_node, next_node));

  double optimal_path_cost = Heuristic_cost(next_node);
  next_node->SetHeuCost(optimal_path_cost);
}

double H_A_Star::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {

  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      params_->step_size * cost_weights_->traj_forward_penalty;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      params_->step_size * cost_weights_->traj_back_penalty;
  }

  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += cost_weights_->traj_gear_switch_penalty;
  }
  piecewise_cost += cost_weights_->traj_steer_penalty * std::abs(next_node->GetSteer());

  piecewise_cost += cost_weights_->traj_steer_change_penalty*
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  //障碍物距离代价
  piecewise_cost += cost_weights_->traj_obs_dist_penalty*calcu_obstacle_cost(next_node);

  return piecewise_cost;
}

double H_A_Star::calcu_obstacle_cost(std::shared_ptr<Node3d> next_node)
{
  double cost=0;
  vector<double> deta_path_x=next_node->GetXs(),
  deta_path_y=next_node->GetYs(),
  deta_path_phi=next_node->GetPhis();
  double deta_s=params_->step_size;

  for(size_t i=0;i<deta_path_x.size();i++)
  {
    Pos3d pos(deta_path_x[i],deta_path_y[i],deta_path_phi[i]);
    double di=distance_to_obs(pos);
    if(di>5){continue;}
    else{
      cost+=1/(di+0.00001);
    }
  }
  return cost*deta_s;
}

double H_A_Star::distance_to_obs(Pos3d current_pos)
{
      //规划点是车辆后轴重心点，因此需要向前移动到几何中心
    Box2d bounding_box = Node3d::GetBoundingBox(
        *vehicle_params_, current_pos.x, current_pos.y, current_pos.phi);
      double min_dist=std::numeric_limits<double>::max();
    //用矩形框和障碍物算距离
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const LineSegment2d& linesegment : obstacle_linesegments) {
          double dist=DistanceTo(bounding_box, linesegment);
          if(dist<min_dist)
          {min_dist=dist;}
      }
    }
    return min_dist;
}

double H_A_Star::Heuristic_cost(std::shared_ptr<Node3d> next_node) {
  //完整性约束，Dijkstra有障碍计算
  double holo_heuristic=Dijk_solver_->cost_to_point(next_node->GetX(),next_node->GetY());
  //非完整性约束，RS曲线长度
  double non_holo_heuristic=0;

  Pos3d current_pos=next_node->GetPose();
  non_holo_heuristic=Get_RS_Length(current_pos);

  double heuristic_cost=std::max(holo_heuristic,non_holo_heuristic);
  return cost_weights_->Heuristic_penalty*heuristic_cost;
}

double H_A_Star::Get_RS_Length(Pos3d current_pos)
{
  Pos3d end_pos=end_node_->GetPose();
  point_type start_pos(std::make_pair(current_pos.x,current_pos.y),current_pos.phi); // 起点 (0,0)，朝向 0 弧度
  point_type end_pose(std::make_pair(end_pos.x,end_pos.y), end_pos.phi); // 终点 (4,2)，朝向 π/2 弧度
  auto curve=reedsShepp_->GetBestRSCurve(params_->min_radius,start_pos,end_pose);
  return curve.second.second;
}

std::shared_ptr<Node3d>  H_A_Star::GenerateFinalNode(const std::vector<Pos3d>& curve_path,std::shared_ptr<Node3d> current_node) 
{
  std::vector<double> traversed_x;
  std::vector<double> traversed_y;
  std::vector<double> traversed_phi;
  for (const auto& pose : curve_path) {
    traversed_x.push_back(pose.x);
    traversed_y.push_back(pose.y);
    traversed_phi.push_back(pose.phi);
  }

  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(
      new Node3d(traversed_x, traversed_y, traversed_phi, XYbounds_,
                 params_->grid_resolution));
  end_node->SetPre(current_node);
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}