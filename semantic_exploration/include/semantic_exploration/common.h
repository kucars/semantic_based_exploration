
#ifndef SEMANTIC_EXPLORATION_COMMON_H
#define SEMANTIC_EXPLORATION_COMMON_H

#include <ros/ros.h>
#include <pcl/point_types.h>

// include input and output archivers for serialization
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include "utilities/time_profiler.h"


extern TimeProfiler timer;


#endif // NBV_EXPLORATION_COMMON_H
