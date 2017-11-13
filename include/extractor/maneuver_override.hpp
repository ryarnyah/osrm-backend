#ifndef MANUEVER_OVERRIDE_HPP
#define MANUEVER_OVERRIDE_HPP

#include "guidance/turn_instruction.hpp"
#include "util/typedefs.hpp"

namespace osrm
{
namespace extractor
{

struct InputManeuverOverride
{
    OSMWayID from;
    OSMWayID to;
    OSMNodeID via;
    std::string type;
    std::string direction;
};

struct ManeuverOverride
{
    NodeID from_node; // initially the internal OSM ID of the node before the turn, then later, the
                      // edge_based_node_id of the turn
    NodeID via_node_id; // node-based node ID
    NodeID to_node; // initially the internal OSM ID of the node before the turn, then later, the
                    // edge_based_node_id of the turn
    guidance::TurnType::Enum override_type;
    guidance::DirectionModifier::Enum direction;

    // check if all parts of the restriction reference an actual node
    bool Valid() const
    {
        return from_node != SPECIAL_NODEID && to_node != SPECIAL_NODEID &&
               via_node_id != SPECIAL_NODEID &&
               direction != guidance::DirectionModifier::MaxDirectionModifier &&
               override_type != guidance::TurnType::Invalid;
    };
};
}
}
#endif
/*
from=1
to=3
via=b

      101      a      102      b        103
---------------+---------------+-------------- (way 1)
      99        \      98       \       97
             51  \ 2          50 \ 3
                  \               \
*/
