#pragma once
#include <prx_models/Graph.h>
#include <prx_models/NodeEdge.h>

namespace motion_planning
{
namespace Types
{

enum NodeEdgeStatus
{
  IDLE = 0,
  ADDED = 1,
  MODIFIED = 2,
  REMOVED = 3
};
}  // namespace Types
}  // namespace motion_planning