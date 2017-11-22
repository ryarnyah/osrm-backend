#include "extractor/intersection/intersection_analysis.hpp"

#include "util/assert.hpp"
#include "util/bearing.hpp"
#include "util/coordinate_calculation.hpp"

#include "extractor/guidance/coordinate_extractor.hpp"

namespace osrm
{
namespace extractor
{
namespace intersection
{
namespace
{
double findAngleBisector(double alpha, double beta)
{
    alpha *= M_PI / 180.;
    beta *= M_PI / 180.;
    return 180. * std::atan2(std::sin(alpha) + std::sin(beta), std::cos(alpha) + std::cos(beta)) /
           M_PI;
}
}

IntersectionEdges getIncomingEdges(const util::NodeBasedDynamicGraph &graph,
                                   const NodeID intersection_node)
{
    IntersectionEdges result;

    for (const auto outgoing_edge : graph.GetAdjacentEdgeRange(intersection_node))
    {
        const auto from_node = graph.GetTarget(outgoing_edge);
        const auto incoming_edge = graph.FindEdge(from_node, intersection_node);

        if (!graph.GetEdgeData(incoming_edge).reversed)
        {
            result.push_back({from_node, incoming_edge});
        }
    }

    // Enforce ordering of incoming edges
    std::sort(result.begin(), result.end());
    return result;
}

IntersectionEdges getOutgoingEdges(const util::NodeBasedDynamicGraph &graph,
                                   const NodeID intersection_node)
{
    IntersectionEdges result;

    for (const auto outgoing_edge : graph.GetAdjacentEdgeRange(intersection_node))
    {
        // TODO: to use TurnAnalysis all outgoing edges are required, to be removed later
        // if (!graph.GetEdgeData(outgoing_edge).reversed)
        {
            result.push_back({intersection_node, outgoing_edge});
        }
    }

    // Enforce ordering of outgoing edges
    std::sort(result.begin(), result.end());
    return result;
}

std::vector<util::Coordinate>
getEdgeCoordinates(const extractor::CompressedEdgeContainer &compressed_geometries,
                   const std::vector<util::Coordinate> &node_coordinates,
                   const NodeID from_node,
                   const EdgeID edge,
                   const NodeID to_node)
{
    if (!compressed_geometries.HasEntryForID(edge))
        return {node_coordinates[from_node], node_coordinates[to_node]};

    BOOST_ASSERT(from_node < node_coordinates.size());
    BOOST_ASSERT(to_node < node_coordinates.size());

    // extracts the geometry in coordinates from the compressed edge container
    std::vector<util::Coordinate> result;
    const auto &geometry = compressed_geometries.GetBucketReference(edge);
    result.reserve(geometry.size() + 2);

    result.push_back(node_coordinates[from_node]);
    std::transform(geometry.begin(),
                   geometry.end(),
                   std::back_inserter(result),
                   [&node_coordinates](const auto &compressed_edge) {
                       return node_coordinates[compressed_edge.node_id];
                   });
    result.push_back(node_coordinates[to_node]);

    // filter duplicated coordinates
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

IntersectionEdgeGeometries
getIntersectionGeometries(const util::NodeBasedDynamicGraph &graph,
                          const extractor::CompressedEdgeContainer &compressed_geometries,
                          const std::vector<util::Coordinate> &node_coordinates,
                          const guidance::MergableRoadDetector &detector,
                          const NodeID intersection_node)
{
    IntersectionEdgeGeometries edge_geometries;

    // TODO: keep CoordinateExtractor to reproduce bearings, simplify later
    const guidance::CoordinateExtractor coordinate_extractor(
        graph, compressed_geometries, node_coordinates);

    std::uint8_t max_lanes_intersection = 0;
    for (auto eid : graph.GetAdjacentEdgeRange(intersection_node))
    {
        max_lanes_intersection =
            std::max(max_lanes_intersection,
                     graph.GetEdgeData(eid).flags.road_classification.GetNumberOfLanes());
    }

    // Collect outgoing edges
    for (const auto outgoing_edge : graph.GetAdjacentEdgeRange(intersection_node))
    {
        const auto remote_node = graph.GetTarget(outgoing_edge);

        const auto &geometry = getEdgeCoordinates(
            compressed_geometries, node_coordinates, intersection_node, outgoing_edge, remote_node);

        OSRM_ASSERT(geometry.size() >= 2, node_coordinates[intersection_node]);

        const auto initial_bearing =
            util::coordinate_calculation::bearing(geometry[0], geometry[1]);
        const auto perceived_bearing = util::coordinate_calculation::bearing(
            geometry[0],
            coordinate_extractor.ExtractRepresentativeCoordinate(intersection_node,
                                                                 outgoing_edge,
                                                                 false,
                                                                 remote_node,
                                                                 max_lanes_intersection,
                                                                 geometry));

        const auto edge_length = util::coordinate_calculation::getLength(
            geometry.begin(), geometry.end(), util::coordinate_calculation::haversineDistance);

        edge_geometries.push_back({outgoing_edge, initial_bearing, perceived_bearing, edge_length});

        // for (auto x : geometry)
        //     std::cout << x << ", ";
        // std::cout << "\n";
    }

    const auto edges_number = edge_geometries.size();

    if (edges_number >= 3)
    {
        // Order edges by bearings in clockwise order
        std::sort(
            edge_geometries.begin(), edge_geometries.end(), [](const auto &lhs, const auto &rhs) {
                return lhs.perceived_bearing < rhs.perceived_bearing;
            });

        // Get mergeable roads
        std::vector<bool> mergable_roads(edges_number);
        for (std::size_t curr = 0, next = 1; curr < edges_number;
             ++curr, next = (next + 1) % edges_number)
        {
            const auto &lhs = edge_geometries[curr];
            const auto &rhs = edge_geometries[next];
            mergable_roads[curr] =
                detector.CanMergeRoad(intersection_node,
                                      {lhs.edge, lhs.perceived_bearing, lhs.length},
                                      {rhs.edge, rhs.perceived_bearing, rhs.length});
            std::cout << curr << " " << lhs.edge << " " << rhs.edge << " " << lhs.perceived_bearing
                      << " " << rhs.perceived_bearing << " " << mergable_roads[curr] << "\n";
        }

        for (std::size_t curr = 0, next = 1, prev = edges_number - 1; curr < edges_number;
             ++curr, next = (next + 1) % edges_number, prev = (prev + 1) % edges_number)
        {
            if (!mergable_roads[prev] && mergable_roads[curr] && !mergable_roads[next])
            { // Merge bearings of roads curr & next
                // TODO: Here is used an angle bisector, but not a reversed closed turn, to be
                // checked
                const auto angle = findAngleBisector(edge_geometries[curr].perceived_bearing,
                                                     edge_geometries[next].perceived_bearing);
                edge_geometries[curr].perceived_bearing = angle;
                edge_geometries[next].perceived_bearing = angle;
            }
        }
    }

    // Add incoming edges with reversed bearings
    edge_geometries.resize(2 * edges_number);
    for (std::size_t index = 0; index < edges_number; ++index)
    {
        const auto &geometry = edge_geometries[index];
        const auto remote_node = graph.GetTarget(geometry.edge);
        const auto incoming_edge = graph.FindEdge(remote_node, intersection_node);
        edge_geometries[edges_number + index] = {incoming_edge,
                                                 util::bearing::reverse(geometry.initial_bearing),
                                                 util::bearing::reverse(geometry.perceived_bearing),
                                                 geometry.length};
    }

    // for (auto x : result)
    //     std::cout << x.edge << "," << x.bearing << ";   ";
    // std::cout << "\n";

    // Enforce ordering of edges by IDs
    std::sort(edge_geometries.begin(), edge_geometries.end());

    return edge_geometries;
}

inline auto findEdge(const IntersectionEdgeGeometries &geometries, const EdgeID &edge)
{
    const auto it = std::lower_bound(
        geometries.begin(), geometries.end(), edge, [](const auto &geometry, const auto edge) {
            return geometry.edge < edge;
        });
    BOOST_ASSERT(it != geometries.end() && it->edge == edge);
    return it;
}

double findEdgeBearing(const IntersectionEdgeGeometries &geometries, const EdgeID &edge)
{
    return findEdge(geometries, edge)->perceived_bearing;
}

double findEdgeLength(const IntersectionEdgeGeometries &geometries, const EdgeID &edge)
{
    return findEdge(geometries, edge)->length;
}

double computeTurnAngle(const IntersectionEdgeGeometries &geometries,
                        const IntersectionEdge &from,
                        const IntersectionEdge &to)
{
    return util::bearing::angleBetween(findEdgeBearing(geometries, from.edge),
                                       findEdgeBearing(geometries, to.edge));
}

template <typename RestrictionsRange>
bool isTurnRestricted(const RestrictionsRange &restrictions, const NodeID to)
{
    // Check turn restrictions to find a node that is the only allowed target when coming from a
    // node to an intersection
    //     d
    //     |
    // a - b - c  and `only_straight_on ab | bc would return `c` for `a,b`
    const auto is_only = std::find_if(restrictions.first,
                                      restrictions.second,
                                      [](const auto &pair) { return pair.second->is_only; });
    if (is_only != restrictions.second)
        return is_only->second->AsNodeRestriction().to != to;

    // Check if explicitly forbidden
    const auto no_turn =
        std::find_if(restrictions.first, restrictions.second, [&to](const auto &restriction) {
            return restriction.second->AsNodeRestriction().to == to;
        });

    return no_turn != restrictions.second;
}

bool isTurnAllowed(const util::NodeBasedDynamicGraph &graph,
                   const EdgeBasedNodeDataContainer &node_data_container,
                   const RestrictionMap &restriction_map,
                   const std::unordered_set<NodeID> &barrier_nodes,
                   const IntersectionEdgeGeometries &geometries,
                   const guidance::TurnLanesIndexedArray &turn_lanes_data,
                   const IntersectionEdge &from,
                   const IntersectionEdge &to)
{
    BOOST_ASSERT(graph.GetTarget(from.edge) == to.node);

    // TODO: to use TurnAnalysis all outgoing edges are required, to be removed later
    if (graph.GetEdgeData(from.edge).reversed || graph.GetEdgeData(to.edge).reversed)
        return false;

    const auto intersection_node = to.node;
    const auto destination_node = graph.GetTarget(to.edge);
    auto const &restrictions = restriction_map.Restrictions(from.node, intersection_node);

    // Check if turn is explicitly restricted by a turn restriction
    if (isTurnRestricted(restrictions, destination_node))
        return false;

    // Precompute reversed bearing of the `from` edge
    const auto from_edge_reversed_bearing =
        util::bearing::reverse(findEdgeBearing(geometries, from.edge));

    // Collect some information about the intersection
    // 1) number of allowed exits and adjacent bidirectional edges
    std::uint32_t allowed_exits = 0, bidirectional_edges = 0;
    // 2) edge IDs of roundabouts edges
    EdgeID roundabout_from = SPECIAL_EDGEID, roundabout_to = SPECIAL_EDGEID;
    double roundabout_from_angle = 0., roundabout_to_angle = 0.;

    for (const auto eid : graph.GetAdjacentEdgeRange(intersection_node))
    {
        const auto &edge_data = graph.GetEdgeData(eid);
        const auto &edge_class = edge_data.flags;
        const auto to_node = graph.GetTarget(eid);
        const auto reverse_edge = graph.FindEdge(to_node, intersection_node);
        BOOST_ASSERT(reverse_edge != SPECIAL_EDGEID);

        const auto is_exit_edge = !edge_data.reversed && !isTurnRestricted(restrictions, to_node);
        const auto is_bidirectional = !graph.GetEdgeData(reverse_edge).reversed;
        allowed_exits += is_exit_edge;
        bidirectional_edges += is_bidirectional;

        if (edge_class.roundabout || edge_class.circular)
        {
            if (edge_data.reversed)
            {
                // "Linked Roundabouts" is an example of tie between two linked roundabouts
                // A tie breaker for that maximizes ∠(roundabout_from_bearing, ¬from_edge_bearing)
                const auto angle = util::bearing::angleBetween(
                    findEdgeBearing(geometries, reverse_edge), from_edge_reversed_bearing);
                if (angle > roundabout_from_angle)
                {
                    roundabout_from = reverse_edge;
                    roundabout_from_angle = angle;
                }
            }
            else
            {
                // a tie breaker that maximizes ∠(¬from_edge_bearing, roundabout_to_bearing)
                const auto angle = util::bearing::angleBetween(from_edge_reversed_bearing,
                                                               findEdgeBearing(geometries, eid));
                if (angle > roundabout_to_angle)
                {
                    roundabout_to = eid;
                    roundabout_to_angle = angle;
                }
            }
        }
    }

    // 3) if the intersection has a barrier
    const bool is_barrier_node = barrier_nodes.find(intersection_node) != barrier_nodes.end();

    // Check a U-turn
    if (from.node == destination_node)
    {
        // Allow U-turns before barrier nodes
        if (is_barrier_node)
            return true;

        // Allow U-turns at dead-ends
        if (graph.GetAdjacentEdgeRange(intersection_node).size() == 1)
            return true;

        // Allow U-turns at dead-ends if there is at most one bidirectional road at the intersection
        // The condition allows a U-turns d→a→d and c→b→c ("Bike - Around the Block" test)
        //   a→b
        //   ↕ ↕
        //   d↔c
        if (allowed_exits == 1 || bidirectional_edges <= 1)
            return true;

        // Allow U-turn if the incoming edge has a U-turn lane
        const auto &incoming_edge_annotation_id = graph.GetEdgeData(from.edge).annotation_data;
        const auto lane_description_id = static_cast<std::size_t>(
            node_data_container.GetAnnotation(incoming_edge_annotation_id).lane_description_id);
        if (lane_description_id != INVALID_LANE_DESCRIPTIONID)
        {
            const auto &turn_lane_offsets = std::get<0>(turn_lanes_data);
            const auto &turn_lanes = std::get<1>(turn_lanes_data);
            BOOST_ASSERT(lane_description_id + 1 < turn_lane_offsets.size());

            if (std::any_of(turn_lanes.begin() + turn_lane_offsets[lane_description_id],
                            turn_lanes.begin() + turn_lane_offsets[lane_description_id + 1],
                            [](const auto &lane) { return lane & guidance::TurnLaneType::uturn; }))
                return true;
        }

        // Don't allow U-turns on usual intersections
        return false;
    }

    // Don't allow turns via barriers for not U-turn maneuvers
    if (is_barrier_node)
        return false;

    // Check for roundabouts exits in the opposite direction of roundabout flow
    if (roundabout_from != SPECIAL_EDGEID && roundabout_to != SPECIAL_EDGEID)
    {
        // Get bearings of edges
        const auto roundabout_from_bearing = findEdgeBearing(geometries, roundabout_from);
        const auto roundabout_to_bearing = findEdgeBearing(geometries, roundabout_to);
        const auto to_bearing = findEdgeBearing(geometries, to.edge);

        // Get angles from the roundabout edge to three other edges
        const auto roundabout_angle =
            util::bearing::angleBetween(roundabout_from_bearing, roundabout_to_bearing);
        const auto roundabout_from_angle =
            util::bearing::angleBetween(roundabout_from_bearing, from_edge_reversed_bearing);
        const auto roundabout_to_angle =
            util::bearing::angleBetween(roundabout_from_bearing, to_bearing);

        // Restrict turning over a roundabout if `roundabout_to_angle` is in
        // a sector between `roundabout_from_bearing` to `from_bearing`
        //
        //             150°                            150°
        //              v░░░░░░                ░░░░░░░░░v
        //             v░░░░░░░                ░░░░░░░░v
        // 270° <-ooo- v -ttt-> 90°       270° <-ttt- v -ooo-> 90°
        //             ^░░░░░░░                ░░░░░░░^
        //             r░░░░░░░                ░░░░░░░r
        //             r░░░░░░░                ░░░░░░░r
        if ((roundabout_from_angle < roundabout_angle &&
             roundabout_to_angle < roundabout_from_angle) ||
            (roundabout_from_angle > roundabout_angle &&
             roundabout_to_angle > roundabout_from_angle))
            return false;
    }

    return true;
}
}
}
}
