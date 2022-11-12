/**
 * @file cpprobotics_types_double.h
 * @brief 
 * @author Wang Dezhao (1282507109@qq.com)
 * @version 1.0
 * @date 2022-05-02 11:41:53
 * 
 * @copyright Copyright (c) 2022 
 */

#ifndef _CPPROBOTICS_TYPES_H
#define _CPPROBOTICS_TYPES_H

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>

namespace cpprobotics{

using Vec_f=std::vector<double>;
using Poi_f=std::array<double, 2>;
using Vec_Poi=std::vector<Poi_f>;

};

#endif
