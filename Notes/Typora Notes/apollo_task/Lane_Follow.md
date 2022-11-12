# SCENARIO: LANE_FOLLOW

## 简介

1. **位置**

   apollo-6.0.0/modules/planning/tasks

   apollo-6.0.0/modules/planning/conf/scenario/lane_follow_config.pb.txt

2. **学习目的**

   1. 了解lane_follow场景的整个运行流程
   2. 学习如何将轨迹进行横纵向解耦
   3. 学习如何在sl图和st图中分别对path和speed进行决策
   4. 学习如何对对path和speed规划的实际问题进行数学建模，并将模型转换为对应求解器的接口进行求解
   5. 最终目的是了解Apollo的planning模块的核心----task的处理逻辑

3. **LANE_FOLLOW场景配置文件**

   后面将按照顺序对task进行逐一分析

```protobuf
scenario_type: LANE_FOLLOW
stage_type: LANE_FOLLOW_DEFAULT_STAGE
stage_config: {
  stage_type: LANE_FOLLOW_DEFAULT_STAGE
  enabled: true
  task_type: LANE_CHANGE_DECIDER
  task_type: PATH_REUSE_DECIDER
  task_type: PATH_LANE_BORROW_DECIDER
  task_type: PATH_BOUNDS_DECIDER
  task_type: PIECEWISE_JERK_PATH_OPTIMIZER
  task_type: PATH_ASSESSMENT_DECIDER
  task_type: PATH_DECIDER
  task_type: RULE_BASED_STOP_DECIDER
  task_type: ST_BOUNDS_DECIDER
  task_type: SPEED_BOUNDS_PRIORI_DECIDER
  task_type: SPEED_HEURISTIC_OPTIMIZER
  task_type: SPEED_DECIDER
  task_type: SPEED_BOUNDS_FINAL_DECIDER
  # task_type: PIECEWISE_JERK_SPEED_OPTIMIZER
  task_type: PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER
  task_type: RSS_DECIDER
  ...
  }
```

## LANE_CHANGE_DECIDER

## PATH_REUSE_DECIDER

## PATH_LANE_BORROW_DECIDER

## PATH_BOUNDS_DECIDER

## PIECEWISE_JERK_PATH_OPTIMIZER

## PATH_ASSESSMENT_DECIDER

## PATH_DECIDER

## RULE_BASED_STOP_DECIDER

## ST_BOUNDS_DECIDER

> `STBoundsDecider` 主要是对新加入的动态以及最近的一个静态且阻塞当前引导路径(`path_decider`规划的路径)的障碍物进行`st图`构建,对不影响纵向规划的障碍物设置`IGNORE`属性,并按照设定轨迹给出每一个障碍物`boundary`的最优决策(`overtake/yield`),最终决策出最优的`Drivable_st_boundary`;

### 1 文件目录

```bash
.
├── BUILD
├── st_bounds_decider.cc
├── st_bounds_decider.h
├── st_driving_limits.cc
├── st_driving_limits.h
├── st_guide_line.cc
├── st_guide_line.h
├── st_obstacles_processor.cc
└── st_obstacles_processor.h
```

### 2 st_obstacles_processor.cc

#### 2.1 初始化ST边界

```c++
void STObstaclesProcessor::Init(const double planning_distance,
                                const double planning_time,
                                const PathData& path_data,
                                PathDecision* const path_decision,
                                History* const history);
```

#### 2.2 将pathDecision的障碍物投影到该路径的ST图中

```c++
common::Status MapObstaclesToSTBoundaries(PathDecision* const path_decision){}
```

##### 1 将路径点分类处理

- 路径点的类型有以下几种

  ```c++
  //modules/planning/common/path/path_data.h
  enum class PathPointType {
      IN_LANE,	//在本车道上
      OUT_ON_FORWARD_LANE,	//在前进行驶的车道上
      OUT_ON_REVERSE_LANE,	//在反向行驶的车道上
      OFF_ROAD,	//在不规则道路上
      UNKNOWN,
    };
  ```

- 遍历路径点将位于 `OUT_ON_FORWARD_LANE` 和 `OUT_ON_REVERSE_LANE` 的路径点存放到`adc_low_road_right_segments_`中

  ```c++
  for (const auto& path_pt_info : path_data_.path_point_decision_guide()) {
      double path_pt_s = 0.0;
      PathData::PathPointType path_pt_type;
      std::tie(path_pt_s, path_pt_type, std::ignore) = path_pt_info;
      if (path_pt_type == PathData::PathPointType::OUT_ON_FORWARD_LANE ||
          path_pt_type == PathData::PathPointType::OUT_ON_REVERSE_LANE) {
        if (is_adc_low_road_right_beginning) {
          adc_low_road_right_segments_.emplace_back(path_pt_s, path_pt_s);
          is_adc_low_road_right_beginning = false;
        } else {
          adc_low_road_right_segments_.back().second = path_pt_s;
        }
      } else if (path_pt_type == PathData::PathPointType::IN_LANE) {
        if (!is_adc_low_road_right_beginning) {
          is_adc_low_road_right_beginning = true;
        }
      }
    }
  ```


<img src="/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture2.png" alt="Picture2" style="zoom:67%;" />

<img src="/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture1.png" alt="Picture1" style="zoom: 67%;" />

##### 2 将各类障碍物绘制到ST图中

障碍物的类型

```c++
//.cache/bazel/540135163923dd7d5820f3ee4b306b32/execroot/apollo/bazel-out/k8-opt/bin/modules/planning/proto/decision.pb.h
  enum ObjectTagCase {
    kIgnore = 1,
    kStop = 2,
    kFollow = 3,
    kYield = 4,
    kOvertake = 5,
    kNudge = 6,
    kAvoid = 7,
    kSidePass = 8,
    OBJECT_TAG_NOT_SET = 0,
  };
```

1. 首先判断每个障碍物是否要出现在`st_graph`中，依据是计算自车的`path路径`与障碍物感知框是否重叠，如果存在碰撞，将其绘制在ST图中，没有碰撞就忽略

2. 将障碍物的上下边界分别存入 `lower_points` 和 `upper_points` 中

   ```c++
   ComputeObstacleSTBoundary(*obs_ptr, &lower_points, &upper_points,
                                      &is_caution_obstacle, &obs_caution_end_t)){}
   ```

3. 将上下边界进行封装，并指明每个障碍物的ID

   ```c++
   auto boundary =
           STBoundary::CreateInstanceAccurate(lower_points, upper_points);
       boundary.set_id(obs_ptr->Id());
   ```

###### 2.1 处理速度相关的交通指示牌

- 如果遇到`KC`(请勿在此停车)，则将其存入 `candidate_clear_zones_`


```c++
    // Store all Keep-Clear zone together.
    // kc: 为请勿停留
    if (obs_item_ptr->Id().find("KC") != std::string::npos) {
      candidate_clear_zones_.push_back(
          make_tuple(obs_ptr->Id(), boundary, obs_ptr));
      continue;
    }
```

###### 2.2 处理静态障碍物

- 静态障碍物将只用4个位置来表述，同时设置 `is_caution_obstacle` 为`true`，在ST图中的时间跨度为整个规划周期(planning_time_default = 8s)

```c++
 if (GetOverlappingS(adc_path_points, obs_box, kADCSafetyLBuffer,
                        &overlapping_s)) {
      lower_points->emplace_back(overlapping_s.first, 0.0);
      lower_points->emplace_back(overlapping_s.first, planning_time_);
      upper_points->emplace_back(overlapping_s.second, 0.0);
      upper_points->emplace_back(overlapping_s.second, planning_time_);
    }
    *is_caution_obstacle = true; //设置为是静态障碍物
    *obs_caution_end_t = planning_time_; //设置时间跨度
```

![Picture3](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture3.png)

- 如果静态障碍物跟自车离得很近，需要记录到`closest_stop_obstacle`，后面还要将其跟`kc` 区域进行对比

  ```c++
  if (std::get<0>(closest_stop_obstacle) == "NULL" ||
            std::get<1>(closest_stop_obstacle).bottom_left_point().s() >
                boundary.bottom_left_point().s()) {
          // If this static obstacle is closer for ADC to stop, record it.c++
          closest_stop_obstacle =
              std::make_tuple(obs_ptr->Id(), boundary, obs_ptr);
        }
  ```

###### 2.3 处理动态障碍物

- 遍历所有出现的障碍物(由pathdecision做出的决策)，计算出每个障碍物与轨迹重叠的起点和终点，另外对于重叠区域的路径点，如果位于 `adc_low_road_right_segments_` 片段中，则设置 `is_caution_obstacle` 为`true`.


```c++
lower_points->emplace_back(overlapping_s.first,
                                   obs_traj_pt.relative_time());
        upper_points->emplace_back(overlapping_s.second,
                                   obs_traj_pt.relative_time());
 if (is_obs_first_traj_pt) {
          if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
              IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
            *is_caution_obstacle = true;
          }
        }
        if ((*is_caution_obstacle)) {
          if (IsSWithinADCLowRoadRightSegment(overlapping_s.first) ||
              IsSWithinADCLowRoadRightSegment(overlapping_s.second)) {
            *obs_caution_end_t = obs_traj_pt.relative_time();
          }
  }
```

<img src="/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture4.png" alt="Picture4" style="zoom: 67%;" />

<img src="/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture5.png" alt="Picture5" style="zoom: 50%;" />

![Picture6](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture6.png)

- 在遍历完所有障碍物之后，需要裁剪无用的边界点，以 `obs_caution_end_t` 为依据进行删除,因为在路径上存在需要小心的障碍物或者在另外车道的障碍物需要，需要将`obs_caution_end_t`的时间对应的边界从`st_boundary`中删除，车辆可能必须停车

  - 更新之后的边界存入`alternative_boundary`, 将可替代边界的障碍物存放到`obs_id_to_alternative_st_boundary_`

  ```c++
      // Update the trimmed obstacle into alternative st-bound storage
      // for later uses.
      while (lower_points.size() > 2 &&
             lower_points.back().t() > obs_caution_end_t) {
        lower_points.pop_back();
      }
      while (upper_points.size() > 2 &&
             upper_points.back().t() > obs_caution_end_t) {
        upper_points.pop_back();
      }
      auto alternative_boundary =
          STBoundary::CreateInstanceAccurate(lower_points, upper_points);
      alternative_boundary.set_id(obs_ptr->Id());
      obs_id_to_alternative_st_boundary_[obs_ptr->Id()] = alternative_boundary;
  ```

  <img src="/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture7.png" alt="Picture7" style="zoom: 50%;" />

- 遍历障碍物的过程中忽略自车后面的车，没有忽略的障碍物的边界存放至`obs_id_to_st_boundary_`,ID存放至`non_ignore_obstacles` 

###### 2.4 处理静态障碍物和KC区域

- 计算`candidate_clear_zones_ `与  `closest_stop_obstacle_`  哪个个区域离本车最近,因为停止线只考虑最近,并更新于`closest_stop_obstacle_`中(如果有另一个障碍物与 Keep-Clear 区域重叠，从而导致更近的停止线，那么该 Keep-Clear 区域也必须与最近的障碍物重叠)

  ```c++
      if (!closest_stop_obs_ptr->IsVirtual()) {
        for (const auto& clear_zone : candidate_clear_zones_) {
          const auto& clear_zone_boundary = std::get<1>(clear_zone);
          if (closest_stop_obs_boundary.min_s() >= clear_zone_boundary.min_s() &&
              closest_stop_obs_boundary.min_s() <= clear_zone_boundary.max_s()) {
            std::tie(closest_stop_obs_id, closest_stop_obs_boundary,
                     closest_stop_obs_ptr) = clear_zone;
            ADEBUG << "Clear zone " << closest_stop_obs_id << " is closer.";
            break;
          }
        }
      }
      obs_id_to_st_boundary_[closest_stop_obs_id] = closest_stop_obs_boundary;
      closest_stop_obs_ptr->set_path_st_boundary(closest_stop_obs_boundary);
      non_ignore_obstacles.insert(closest_stop_obs_id);
  ```

###### 2.5 处理未与自车轨迹相交的障碍物

遍历所有的障碍物，将未与自车规划轨迹碰撞的障碍物，设置为 `Ignore` 状态

```c++
  // Set IGNORE decision for those that are not in ST-graph:
  for (const auto* obs_item_ptr : path_decision->obstacles().Items()) {
    Obstacle* obs_ptr = path_decision->Find(obs_item_ptr->Id());
    if (non_ignore_obstacles.count(obs_ptr->Id()) == 0) {
      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      if (!obs_ptr->HasLongitudinalDecision()) {
        obs_ptr->AddLongitudinalDecision("st_obstacle_processor",
                                         ignore_decision);
      }
      if (!obs_ptr->HasLateralDecision()) {
        //设置Ignore状态
        obs_ptr->AddLateralDecision("st_obstacle_processor", ignore_decision);
      }
    }
  }
```

###### 2.6 处理在运动过程中新加入进ST图的障碍物

- 首先找到当前决策下的障碍物组合的 `s_min` 和 `s_max` 

  - 通过对以决策的障碍物上下边界循环检索

  ```c++
   // Based on existing decisions, get the s-boundary.
    double s_min = 0.0;
    // planning_distance指的是离散数据点的长度 back().s() - front().s();
    double s_max = planning_distance_;
    for (auto it : obs_id_to_decision_) {
      auto obs_id = it.first;
      auto obs_decision = it.second;
      auto obs_st_boundary = obs_id_to_st_boundary_[obs_id];
      double obs_s_min = 0.0;
      double obs_s_max = 0.0;
      obs_st_boundary.GetBoundarySRange(t, &obs_s_max, &obs_s_min);
      if (obs_decision.has_yield() || obs_decision.has_stop()) {
        s_max = std::fmin(s_max, obs_s_min);
      } else if (it.second.has_overtake()) {
        s_min = std::fmax(s_min, obs_s_max);
      }
    }
    if (s_min > s_max) {
      return false;
    }
  ```

![Picture8](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture8.png)

​																			(此处感觉有误)

- 如果`new_t_edge` 的边界大于`s_max`说明障碍物距离自车较远，决策为`yield`

- 如果`new_t_edge` 的边界小于`s_min`说明障碍物位于自车后方，决策为`overtaken`

  ```c++
    // For newly entering st_boundaries, determine possible new-boundaries.
    // For apparent ones, make decisions directly.
  	for (auto obs_t_edge : new_t_edges) {
      ADEBUG << "For obstacle id: " << std::get<4>(obs_t_edge)
             << ", its s-range = [" << std::get<2>(obs_t_edge) << ", "
             << std::get<3>(obs_t_edge) << "]";
      if (std::get<0>(obs_t_edge) == 1) {
        if (std::get<2>(obs_t_edge) >= s_max) {
          ADEBUG << "  Apparently, it should be yielded.";
          obs_id_to_decision_[std::get<4>(obs_t_edge)] =
              DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                        std::get<3>(obs_t_edge), s_max);
          obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
              STBoundary::BoundaryType::YIELD);
        } else if (std::get<3>(obs_t_edge) <= s_min) {
          ADEBUG << "  Apparently, it should be overtaken.";
          obs_id_to_decision_[std::get<4>(obs_t_edge)] =
              DetermineObstacleDecision(std::get<2>(obs_t_edge),
                                        std::get<3>(obs_t_edge), s_min);
          obs_id_to_st_boundary_[std::get<4>(obs_t_edge)].SetBoundaryType(
              STBoundary::BoundaryType::OVERTAKE);
        } else {
          ADEBUG << "  It should be further analyzed.";
          ambiguous_t_edges.push_back(obs_t_edge);
        }
      }
    }
  ```

- 如果`new_t_edge` 的边界位于`s_min`和`s_max`之间，列举所有已有的决策和已存在的界限进行下一步的分析

  ```c++
  // For ambiguous ones, enumerate all decisions and corresponding bounds.
  // 计算出 st_graph 的垂直间隙(SGaps)
    auto s_gaps = FindSGaps(ambiguous_t_edges, s_min, s_max);
    if (s_gaps.empty()) {
      return false;
    }
    for (auto s_gap : s_gaps) {
      available_s_bounds->push_back(s_gap);
      std::vector<std::pair<std::string, ObjectDecisionType>> obs_decisions;
      for (auto obs_t_edge : ambiguous_t_edges) {
        std::string obs_id = std::get<4>(obs_t_edge);
        double obs_s_min = std::get<2>(obs_t_edge);
        double obs_s_max = std::get<3>(obs_t_edge);
        obs_decisions.emplace_back(
            obs_id,
            DetermineObstacleDecision(obs_s_min, obs_s_max,
                                      (s_gap.first + s_gap.second) / 2.0));
      }
      available_obs_decisions->push_back(obs_decisions);
    }
  ```

  - 首先计算新加入的障碍物与根据现有决策得到的`s_min`、`s_max`在某一时间`t`下的垂直间隙`s_gaps` ，即对应可通行的上下边界

    ```c++
      auto s_gaps = FindSGaps(ambiguous_t_edges, s_min, s_max);
    ```
    
  - 根据新加入的障碍物,目前决策该障碍物的决策信号.需要进一步判断,根据障碍物信息`s_gap`，判断决策，如果`s_gap[s1,s2]`决策信号为`yeild`,如果是`s_gap[s3,s4]`决策信号为`over_take`,后面会根据这些情况进行遍历，选择最优的决策信号

    ```c++
    for (auto s_gap : s_gaps) {
        available_s_bounds->push_back(s_gap);
        std::vector<std::pair<std::string, ObjectDecisionType>> obs_decisions;
        for (auto obs_t_edge : ambiguous_t_edges) {
          std::string obs_id = std::get<4>(obs_t_edge);
          double obs_s_min = std::get<2>(obs_t_edge);
          double obs_s_max = std::get<3>(obs_t_edge);
          obs_decisions.emplace_back(
              obs_id,
              DetermineObstacleDecision(obs_s_min, obs_s_max,
                                        (s_gap.first + s_gap.second) / 2.0));
        }
        available_obs_decisions->push_back(obs_decisions);
      }
    ```

    ![Picture9](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture9.png)


###### 2.7 封装障碍物边界

遍历`obs_id_to_st_boundary_`中的障碍物边界生成`obs_t_edges`，其中`true`为预测障碍物与自车轨迹开始重叠，`false`为预测障碍物与自车轨迹重叠结束。之后再将`obs_t_edges`以起始障碍物出现的时间位置升序排序，若相同，则按是是否为`is_starting_t`降序排序

```c++
 // Preprocess the obstacles for sweep-line algorithms.
  // Fetch every obstacle's beginning end ending t-edges only.
  for (const auto& it : obs_id_to_st_boundary_) {
    obs_t_edges_.emplace_back(true, it.second.min_t(),
                              it.second.bottom_left_point().s(),
                              it.second.upper_left_point().s(), it.first);
    obs_t_edges_.emplace_back(false, it.second.max_t(),
                              it.second.bottom_right_point().s(),
                              it.second.upper_right_point().s(), it.first);
  }
  // Sort the edges.
	// ObsTEdge contains: (is_starting_t, t, s_min, s_max, obs_id).
  std::sort(obs_t_edges_.begin(), obs_t_edges_.end(),
            [](const ObsTEdge& lhs, const ObsTEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });
```

### 3 st_driving_limits.cc

计算由于车辆运动学约而导致的速度限制，将会得到一条 `s-t` 边界约束曲线

#### 3.1 计算减速曲线

$$
s(t)=\left\{
\begin{aligned}
s_{0} + (v_{lower}(t) \cdot\Delta t - \frac{1}{2}  a_{dec} \cdot \Delta t^2) &&, \Delta t \le t_{最大减速时间}\\
s_{0} + \frac{1}{2} v_{lower}(t) \cdot\Delta t  &&, \Delta t \ge t_{最大减速时间}\\
\end{aligned}
\right. \\\\

v_{lower}(t) = v_{0} -a_{dec} \cdot \Delta t
$$

 

```c++
  // Process lower bound: (constant deceleration)
  double dec_time = lower_v0_ / max_dec_;
  if (t - lower_t0_ < dec_time) {
    dynamic_limits.first =
        lower_s0_ + (lower_v0_ - max_dec_ * (t - lower_t0_) + lower_v0_) *
                        (t - lower_t0_) * 0.5;
  } else {
    dynamic_limits.first = lower_s0_ + (lower_v0_ * dec_time) * 0.5;
  }
```

#### 3.2 计算加速曲线

$$
s(t)=\left\{
\begin{aligned}
s_{0} + (v_{upper}(t) \cdot\Delta t + \frac{1}{2}  a_{acc} \cdot \Delta t^2) &&, \Delta t \le t_{最大加速时间}\\
s_{0} + \frac{1}{2} v_{upper}(t) \cdot\Delta t  &&, \Delta t \ge t_{最大加速时间}\\
\end{aligned}
\right. \\\\

v_{upper}(t) = v_{0}+a_{acc} \cdot \Delta t
$$

```c++
 // Process upper bound: (constant acceleration)
  double acc_time = (max_v_ - upper_v0_) / max_acc_;
  if (t - upper_t0_ < acc_time) {
    dynamic_limits.second =
        upper_s0_ + (upper_v0_ + max_acc_ * (t - upper_t0_) + upper_v0_) *
                        (t - upper_t0_) * 0.5;
  } else {
    dynamic_limits.second = upper_s0_ + (upper_v0_ + max_v_) * acc_time * 0.5 +
                            (t - upper_t0_ - acc_time) * max_v_;
  }
```

#### 3.3 速度边界

<img src="/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture10.png" alt="Picture10" style="zoom: 67%;" />

### 4 st_guide_line.cc

#### 4.1 计算匀速运动曲线

用匀速模型(下图绿色曲线)来模拟生成引导线，这条引导线用于`对新加入障碍物做出决策`

> 事实上用匀加速来模拟引导线会更好，下图红色曲线

```c++
double STGuideLine::GetGuideSFromT(double t) {
  common::SpeedPoint speed_point;
  if (t < guideline_speed_data_.TotalTime() &&
      guideline_speed_data_.EvaluateByTime(t, &speed_point)) {
    s0_ = speed_point.s();
    t0_ = t;
    return speed_point.s();
  }
  return s0_ + (t - t0_) * v0_;
}
```

![Screenshot from 2022-05-12 11-35-26](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Screenshot from 2022-05-12 11-35-26.png)

### 5 st_bounds_decider.cc

#### 5.1 移除无效决策

通过`st_driving_limits.cc`和 `st_guide_line.cc` 两个处理过程可以得到**自车的行驶速度边界范围** 和用来决策新障碍物的**速度引导线**，由此可以**移除无效的决策**

- `available_choices` 中存放 `st_obstacles_processor.cc`中**新加入**的且**无法做出决策**的障碍物

```c++
void STBoundsDecider::RemoveInvalidDecisions(
    std::pair<double, double> driving_limit,
    std::vector<std::pair<STBoundPoint, ObsDecSet>>* available_choices) {
  // Remove those choices that don't even fall within driving-limits.
  size_t i = 0;
  while (i < available_choices->size()) {
    double s_lower = 0.0;
    double s_upper = 0.0;
    std::tie(std::ignore, s_lower, s_upper) = available_choices->at(i).first;
    //driving_limit (减速曲线，加速曲线);
    if (s_lower > driving_limit.second || s_upper < driving_limit.first) {
      // Invalid bound, should be removed.
      if (i != available_choices->size() - 1) {
        swap(available_choices->at(i),
             available_choices->at(available_choices->size() - 1));
      }
      available_choices->pop_back();
    } else {
      // Valid bound, proceed to the next one.
      ++i;
    }
  }
}
```

图中`s_gap[s4,1]`,`s_gap[s3,1]`超出车辆物理限制，被认定为无效的决策及无效的边界，将从`available_choices` 移除

![Picture12](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture12.png)

#### 5.2 对可利用区域进行等级排序

1. 如果2个`s_gap`均大于`kSTPassableThreshold`，那么将可行区域大的`s_gap`排在前面

   ```c++
    // If not both are larger than passable-threshold, should select
         // the one with larger room.
         double A_room = std::fmin(driving_limit.second, A_s_upper) -
                         std::fmax(driving_limit.first, A_s_lower);
         double B_room = std::fmin(driving_limit.second, B_s_upper) -
                         std::fmax(driving_limit.first, B_s_lower);
         if (A_room < kSTPassableThreshold || B_room < kSTPassableThreshold) {
           if (A_room < B_room) {
             swap(available_choices->at(i + 1), available_choices->at(i));
             has_swaps = true;
             ADEBUG << "Swapping to favor larger room.";
           }
           continue;
         }
   ```

2. 继续检查`s_gap` 区间是否包含`s_guide_line`引导线，如果是，则将包含引导线的`s_gap`排在前面，遍历`s_gap`，以此进行检查和排序

   ```c++
       // Should select the one with overlap to guide-line
         bool A_contains_guideline =
             A_s_upper >= s_guide_line && A_s_lower <= s_guide_line;
         bool B_contains_guideline =
             B_s_upper >= s_guide_line && B_s_lower <= s_guide_line;
         if (A_contains_guideline != B_contains_guideline) {
           if (!A_contains_guideline) {
             swap(available_choices->at(i + 1), available_choices->at(i));
             has_swaps = true;
             ADEBUG << "Swapping to favor overlapping with guide-line.";
           }
           continue;
         }
   ```

#### 5.3 根据等级做出决策以及更新数据

1. 选择排序最靠前的`s_gap`,这个过程相当于对无法确定的新加入障碍物做出决策
2. 结合`s_gap`和物理性能限制,更新`s_lower`,`s_upper`更新最大，最小速度

![Picture13](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture13.png)

#### 5.4 将可行驶区域的数据传入ST图中

可行驶区域`RegularSTBound`的数据包括，`regular_st_bound`和 `regular_vt_bound`

```c++
st_graph_data->SetSTDrivableBoundary(regular_st_bound, regular_vt_bound);
```

![Picture14](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture14.png)

图中`s_t_boundary`为青绿色区域，`v_t_boundary`为速度限制区域(最大加速度，最大减速度)，

则在整个规划周期`s_boundary`由`（t,s_lower,s_upper）`三个数据组成的元组构成

**注意：**

​		获取`obs_id_to_st_boundary_`（注意`obs_id_to_st_boundary_`只是在轨迹上跟车辆可能有重合的障碍物，比如id=11,12,13,14的障碍物没有与路径重合，不在`obs_id_to_st_boundary_`中,但是id=10障碍物与路径重合,需要考虑），方便记录数据，ST边界和可视化debug

<img src="/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture15.png" alt="Picture15" style="zoom:80%;" />	

## SPEED_BOUNDS_PRIORI_DECIDER

> `SpeedBoundsPrioriDecider`主要是根据道路限速、曲率以及障碍物信息对`path`上的每个点进行限速，并针对`STBoundsDecider`生成的`ST_Boundary`进行更细致的调整

### 1 文件目录

```bash
.
├── BUILD
├── speed_bounds_decider.cc
├── speed_bounds_decider.h
├── speed_limit_decider.cc
├── speed_limit_decider.h
├── st_boundary_mapper.cc
├── st_boundary_mapper.h
└── st_boundary_mapper_test.cc
```

### 2 st_boundary_mapper.cc

> 将障碍物映射到ST图中，根据决策信息不同，st_boundary进行进一步的微调

#### 2.1 计算静态障碍物的STBoundary

静态障碍物没有预测轨迹，只需将障碍物的当前位置映射到 st_graph 并始终假设它是静态的

#### 2.2 计算动态障碍物的STBoundary

> 对于每个障碍物以及他的预测轨迹(5s内，每0.1s有一个预测轨迹点)。检查轨迹点和障碍物的预测轨迹是否相交，如果相交，就可以构造一个(累计距离s，相对时间t)的一个锚点

从上面代码我们可以很清晰的看到这个(累计距离s，相对时间t)标定框的计算，这个标定框可以解释为，无人车再该时间点，可以行驶到的坐标上下界。每个时刻障碍物的所在的区域标定框，无人车不能与该标定框有冲突。

```c++
//每半个车身长度的距离为一个采样点
const double step_length = vehicle_param_.front_edge_to_center();
// 在规划的路径下采样
for (double path_s = 0.0; path_s < path_len; path_s += step_length) {
  const auto curr_adc_path_point =
    discretized_path.Evaluate(path_s + discretized_path.front().s()); //计算采样点的累计步长
  if (CheckOverlap(curr_adc_path_point, obs_box, l_buffer)) {
    // Found overlap, start searching with higher resolution
    //下界初始距离
    const double backward_distance = -step_length; 	
    //上边界初始位置
    const double forward_distance = vehicle_param_.length() +		
      vehicle_param_.width() +
      obs_box.length() + obs_box.width();
    const double default_min_step = 0.1;  // in meters
    const double fine_tuning_step_length = std::fmin(
      default_min_step, discretized_path.Length() / default_num_point);

    bool find_low = false;
    bool find_high = false;
    double low_s = std::fmax(0.0, path_s + backward_distance);
    double high_s =
      std::fmin(discretized_path.Length(), path_s + forward_distance);

    // Keep shrinking by the resolution bidirectionally until finally
    // locating the tight upper and lower bounds.
    // 采用逐步靠近的方法，构造更紧凑的上下界low_s和high_s
    while (low_s < high_s) {
      ...
        if (!find_high) {
          const auto& point_high = discretized_path.Evaluate(
            high_s + discretized_path.front().s());
          if (!CheckOverlap(point_high, obs_box, l_buffer)) {
            high_s -= fine_tuning_step_length;
          } else {
            find_high = true;
          }
        }
    }
    if (find_high && find_low) {
      // 加入上下界信息，对应在参考线上的累计距离s，和相对时间t(障碍物轨迹相对时间)
      lower_points->emplace_back(
        low_s - speed_bounds_config_.point_extension(),
        trajectory_point_time);
      upper_points->emplace_back(
        high_s + speed_bounds_config_.point_extension(),
        trajectory_point_time);
    }
    break;
  }
}
```

### 3 speed_limit_decider.cc

> 为沿路径的每个点设置速度限制

#### 3.1 由reference_line_得到道路限速

```c++
// (1) speed limit from map
double speed_limit_from_reference_line =
  reference_line_.GetSpeedLimitFromS(reference_line_s);
```

#### 3.2 由最大向心加速度和道路曲率得到最大速度限制

$$
v_{max}=\sqrt \frac {a_{centicmax}}{K_{path}}
$$

```c++
// (2) speed limit from path curvature
//  -- 2.1: limit by centripetal force (acceleration)
const double speed_limit_from_centripetal_acc =
  std::sqrt(speed_bounds_config_.max_centric_acceleration_limit() /
            std::fmax(std::fabs(discretized_path.at(i).kappa()),
                      speed_bounds_config_.minimal_kappa()));
```

#### 3.3 由nudge obstacles触发的速度限制(可选)

> 障碍物分为横向障碍物和纵向障碍物。
>
> 1.横向障碍物将可能导致车辆的nudge行为
>
> 2.纵向障碍物可能导致车辆出现：stop，yield，follow，overtake行为。这几个行为的优先级从左到右依次递减

障碍物分两种情况考虑，但在速度限制数值上没有区别

1. **nudge**位于自车左边，并且无人车确实被障碍物阻挡，自车应当靠右行驶
2. **nudge**位于自车右边，并且无人车确实被障碍物阻挡，自车应当靠左行驶

```c++
// obstacle is on the right of ego vehicle (at path point i)
bool is_close_on_left =
  (nudge_decision.type() == ObjectNudge::LEFT_NUDGE) &&
  (frenet_point_l - vehicle_param_.right_edge_to_center() -
   collision_safety_range <
   ptr_obstacle->PerceptionSLBoundary().end_l());

// obstacle is on the left of ego vehicle (at path point i)
bool is_close_on_right =
  (nudge_decision.type() == ObjectNudge::RIGHT_NUDGE) &&
  (ptr_obstacle->PerceptionSLBoundary().start_l() -
   collision_safety_range <
   frenet_point_l + vehicle_param_.left_edge_to_center());
```

速度 =  当前道路限速  * 自定义比例

```c++
speed_limit_from_nearby_obstacles =
nudge_speed_ratio * speed_limit_from_reference_line;
```

![Picture16](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture16.png)

#### 3.4 分析得到当前时刻的速度限制

1. 默认自车的最低限速为**2.5 m/s**

   ```protobuf
   message SpeedBoundsDeciderConfig {
     optional double total_time = 1 [default = 7.0];
     optional double boundary_buffer = 2 [default = 0.1];
     optional double max_centric_acceleration_limit = 3 [default = 2.0];
     optional double minimal_kappa = 4 [default = 0.00001];
     optional double point_extension = 5 [default = 1.0];
     optional double lowest_speed = 6 [default = 2.5];
     optional double collision_safety_range = 7 [default = 1.0];
     optional double static_obs_nudge_speed_ratio = 8;
     optional double dynamic_obs_nudge_speed_ratio = 9;
   }
   ```

2. 取上述三种限速中的**最低限速**和**默认速度**作比较，取二者的较大值

   ```c++
   int curr_speed_limit =
     std::fmax(speed_bounds_config_.lowest_speed(),
               std::min({speed_limit_from_reference_line,
                         speed_limit_from_centripetal_acc,
                         speed_limit_from_nearby_obstacles}));
   ```

### 4 speed_bounds_decider.cc

> 重新计算每个障碍物的`ST_boundary`，此时所有障碍物还没`yeild、overtake`等决策信息，因此对为决策的障碍物进行重新一遍的`st_boundary`计算,但不影响`st_driver_boundary`（注意：`st_boundary` 不是 `st_driver_boundary`）

#### 4.1 处理逻辑

```c++
//重新计算每个障碍物的STBoundary
STBoundaryMapper boundary_mapper(
      speed_bounds_config_, reference_line, path_data,
      path_data.discretized_path().Length(), speed_bounds_config_.total_time(),
      injector_);
...
//沿path路径为每个点建立速度约束  
//针对每个s位置，设置不同速度限制，便于DP检查约束
SpeedLimitDecider speed_limit_decider(speed_bounds_config_, reference_line,
                                        path_data);
...
//获取path_length作为st_graph中的s轴                                    
const double path_data_length = path_data.discretized_path().Length();
...
//将规划时间作为 st_graph中的 t 轴
const double total_time_by_conf = speed_bounds_config_.total_time();
```

1. 不与车辆轨迹重合的障碍物，将决策设置为忽略，此阶段不会重新计算该障碍物`st_bounary`
2. 与车辆轨迹重合的障碍物，如果障碍物是明显的决策，比如**障碍物距离本车非常远**（决策为`yeild`），如下图红色，橄榄绿障碍物是明显的决策，不需计算就能判断决策信息，将此障碍物设置相应决策标签，并计算`st_bounary`。
3. 与车辆轨迹重合的障碍物，如果障碍物是不明显的决策，此时需要**DP**进一步计算，此时决策标签不设置，但计算`st_bounary`。例如下图黄色，蓝色，绿色障碍物是非明显决策障碍物。

![Picture17](/home/next/routing_planning/Notes/Typora Notes/apollo_task/jpg/Picture17.png)  

#### 4.2 **st_boundary** 和 **st_driver_boundary** 的区别

`ST_BOUNDS_DECIDER`模块障碍物的ST图为`st_driver_boundary` ,此模块计算耗时较高，最终生成的`st_drivable_boundary_`边界精度也较低。`FLAGS_use_st_drivable_boundary`不置位时,则无需使用该模块输出的`st_drivable_boundary`，可结合实际需求，来精简该模块的冗余计算。

```c++
if (!FLAGS_use_st_drivable_boundary) {
    path_decision->EraseStBoundaries();
  }
```

## SPEED_HEURISTIC_OPTIMIZER

## SPEED_DECIDER

## SPEED_BOUNDS_FINAL_DECIDER

## PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER

## RSS_DECIDER



