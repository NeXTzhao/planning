/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file reference_line.h
 **/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "../math/math_method/vec2d.h"
// #include "modules/common/proto/pnc_point.pb.h"
// #include "modules/map/pnc_map/path.h"
// #include "modules/map/proto/map.pb.h"
// #include "modules/map/proto/map_geometry.pb.h"
// #include "modules/planning/proto/sl_boundary.pb.h"
// #include "modules/planning/reference_line/reference_point.h"
// #include "modules/routing/proto/routing.pb.h"

namespace apollo {
namespace planning {

class ReferenceLine {
 public:
  ReferenceLine(){}
//TODO:记得修改
  double Length(){return 5;}

};

}  // namespace planning
}  // namespace apollo