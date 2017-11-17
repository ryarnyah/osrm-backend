#ifndef ENGINE_API_ROUTE_HPP
#define ENGINE_API_ROUTE_HPP

#include "extractor/maneuver_override.hpp"
#include "engine/api/base_api.hpp"
#include "engine/api/json_factory.hpp"
#include "engine/api/route_parameters.hpp"

#include "engine/datafacade/datafacade_base.hpp"

#include "engine/guidance/assemble_geometry.hpp"
#include "engine/guidance/assemble_leg.hpp"
#include "engine/guidance/assemble_overview.hpp"
#include "engine/guidance/assemble_route.hpp"
#include "engine/guidance/assemble_steps.hpp"
#include "engine/guidance/collapse_turns.hpp"
#include "engine/guidance/lane_processing.hpp"
#include "engine/guidance/post_processing.hpp"
#include "engine/guidance/verbosity_reduction.hpp"

#include "engine/internal_route_result.hpp"

#include "util/coordinate.hpp"
#include "util/integer_range.hpp"
#include "util/json_util.hpp"

#include <iterator>
#include <vector>

namespace osrm
{
namespace engine
{
namespace api
{

class RouteAPI : public BaseAPI
{
  public:
    RouteAPI(const datafacade::BaseDataFacade &facade_, const RouteParameters &parameters_)
        : BaseAPI(facade_, parameters_), parameters(parameters_)
    {
    }

    void MakeResponse(const InternalManyRoutesResult &raw_routes,
                      util::json::Object &response) const
    {
        BOOST_ASSERT(!raw_routes.routes.empty());

        util::json::Array jsRoutes;

        for (const auto &route : raw_routes.routes)
        {
            if (!route.is_valid())
                continue;

            jsRoutes.values.push_back(MakeRoute(route.segment_end_coordinates,
                                                route.unpacked_path_segments,
                                                route.source_traversed_in_reverse,
                                                route.target_traversed_in_reverse));
        }

        response.values["waypoints"] =
            BaseAPI::MakeWaypoints(raw_routes.routes[0].segment_end_coordinates);
        response.values["routes"] = std::move(jsRoutes);
        response.values["code"] = "Ok";
    }

  protected:
    template <typename ForwardIter>
    util::json::Value MakeGeometry(ForwardIter begin, ForwardIter end) const
    {
        if (parameters.geometries == RouteParameters::GeometriesType::Polyline)
        {
            return json::makePolyline<100000>(begin, end);
        }

        if (parameters.geometries == RouteParameters::GeometriesType::Polyline6)
        {
            return json::makePolyline<1000000>(begin, end);
        }

        BOOST_ASSERT(parameters.geometries == RouteParameters::GeometriesType::GeoJSON);
        return json::makeGeoJSONGeometry(begin, end);
    }

    template <typename GetFn>
    util::json::Array GetAnnotations(const guidance::LegGeometry &leg, GetFn Get) const
    {
        util::json::Array annotations_store;
        annotations_store.values.reserve(leg.annotations.size());
        std::for_each(leg.annotations.begin(),
                      leg.annotations.end(),
                      [Get, &annotations_store](const auto &step) {
                          annotations_store.values.push_back(Get(step));
                      });
        return annotations_store;
    }

    util::json::Object MakeRoute(const std::vector<PhantomNodes> &segment_end_coordinates,
                                 const std::vector<std::vector<PathData>> &unpacked_path_segments,
                                 const std::vector<bool> &source_traversed_in_reverse,
                                 const std::vector<bool> &target_traversed_in_reverse) const
    {
        for (const auto &a : unpacked_path_segments)
        {
            std::cout << "Route: ";
            std::cout << (source_traversed_in_reverse[0]
                              ? segment_end_coordinates[0].source_phantom.reverse_segment_id.id
                              : segment_end_coordinates[0].source_phantom.forward_segment_id.id)
                      << " ";
            for (const auto &b : a)
            {
                std::cout << "(";
                std::cout << b.from_edge_based_node << " ";
                std::cout << static_cast<int>(b.turn_instruction.type) << " ";
                std::cout << static_cast<int>(b.turn_instruction.direction_modifier) << ") ";
            }
            std::cout << (target_traversed_in_reverse[0]
                              ? segment_end_coordinates[0].target_phantom.reverse_segment_id.id
                              : segment_end_coordinates[0].target_phantom.forward_segment_id.id)
                      << " ";

            std::cout << std::endl;
        }
        std::vector<guidance::RouteLeg> legs;
        std::vector<guidance::LegGeometry> leg_geometries;
        auto number_of_legs = segment_end_coordinates.size();
        legs.reserve(number_of_legs);
        leg_geometries.reserve(number_of_legs);

        for (auto idx : util::irange<std::size_t>(0UL, number_of_legs))
        {
            const auto &phantoms = segment_end_coordinates[idx];
            const auto &path_data = unpacked_path_segments[idx];

            const bool reversed_source = source_traversed_in_reverse[idx];
            const bool reversed_target = target_traversed_in_reverse[idx];

            auto leg_geometry = guidance::assembleGeometry(BaseAPI::facade,
                                                           path_data,
                                                           phantoms.source_phantom,
                                                           phantoms.target_phantom,
                                                           reversed_source,
                                                           reversed_target);
            auto leg = guidance::assembleLeg(facade,
                                             path_data,
                                             leg_geometry,
                                             phantoms.source_phantom,
                                             phantoms.target_phantom,
                                             reversed_target,
                                             parameters.steps);

            if (parameters.steps)
            {
                auto steps = guidance::assembleSteps(BaseAPI::facade,
                                                     path_data,
                                                     leg_geometry,
                                                     phantoms.source_phantom,
                                                     phantoms.target_phantom,
                                                     reversed_source,
                                                     reversed_target);

                /* Perform step-based post-processing.
                 *
                 * Using post-processing on basis of route-steps for a single leg at a time
                 * comes at the cost that we cannot count the correct exit for roundabouts.
                 * We can only emit the exit nr/intersections up to/starting at a part of the leg.
                 * If a roundabout is not terminated in a leg, we will end up with a
                 *enter-roundabout
                 * and exit-roundabout-nr where the exit nr is out of sync with the previous enter.
                 *
                 *         | S |
                 *         *   *
                 *  ----*        * ----
                 *                  T
                 *  ----*        * ----
                 *       V *   *
                 *         |   |
                 *         |   |
                 *
                 * Coming from S via V to T, we end up with the legs S->V and V->T. V-T will say to
                 *take
                 * the second exit, even though counting from S it would be the third.
                 * For S, we only emit `roundabout` without an exit number, showing that we enter a
                 *roundabout
                 * to find a via point.
                 * The same exit will be emitted, though, if we should start routing at S, making
                 * the overall response consistent.
                 *
                 * ⚠ CAUTION: order of post-processing steps is important
                 *    - handleRoundabouts must be called before collapseTurnInstructions that
                 *      expects post-processed roundabouts
                 */

                guidance::trimShortSegments(steps, leg_geometry);
                leg.steps = guidance::handleRoundabouts(std::move(steps));
                leg.steps = guidance::collapseTurnInstructions(std::move(leg.steps));
                leg.steps = guidance::anticipateLaneChange(std::move(leg.steps));
                leg.steps = guidance::buildIntersections(std::move(leg.steps));
                leg.steps = guidance::suppressShortNameSegments(std::move(leg.steps));
                leg.steps = guidance::assignRelativeLocations(std::move(leg.steps),
                                                              leg_geometry,
                                                              phantoms.source_phantom,
                                                              phantoms.target_phantom);
                leg_geometry = guidance::resyncGeometry(std::move(leg_geometry), leg.steps);

                // apply manual override relations
                bool done = false;
                for (auto current_step_it = leg.steps.begin(); current_step_it != leg.steps.end();
                     ++current_step_it)
                {
                    // TODO: figure out a way to check the
                    // `phantoms.source_phantom.forward/reverse_segment_id.id`
                    // before starting looking at the leg.steps entries.
                    //    first edge_based_node =
                    //      reversed_source ? phantoms.source_phantom.reverse_segment_id.id
                    //                      : phantoms.source_phantom.forward_segment_id.id);
                    const auto overrides =
                        BaseAPI::facade.GetOverridesThatStartAt(current_step_it->from_id);
                    if (overrides.empty())
                        continue;
                    for (const extractor::ManeuverOverride &maneuver_relation : overrides)
                    {
                        // check if the to member of the override relation is in the route
                        // Look ahead up to 5 steps
                        std::size_t MAX_MANEUVER_DISTANCE =
                            std::min(5l, std::distance(current_step_it, leg.steps.end()));
                        auto max_steps_fwd = current_step_it + MAX_MANEUVER_DISTANCE;
                        auto to_match = std::find_if(
                            current_step_it, max_steps_fwd, [&maneuver_relation](const auto &step) {
                                return step.from_id == maneuver_relation.to_node;
                            });
                        if (to_match == max_steps_fwd)
                        {
                            // If we didn't match one of the steps, also check if we're near the end
                            // of the route - if so, check the phantom node ID, it's the last
                            // edge-based-node in the route sequence
                            if (MAX_MANEUVER_DISTANCE >= 5 ||
                                (reversed_target ? phantoms.target_phantom.reverse_segment_id.id
                                                 : phantoms.target_phantom.forward_segment_id.id) !=
                                    maneuver_relation.to_node)
                            {
                                continue;
                            }
                        }

                        const auto via_node_coords =
                            BaseAPI::facade.GetCoordinateOfNode(maneuver_relation.via_node_id);

                        // search for corresponding via_node in the subsequent geometries
                        auto current_step_copy = current_step_it;
                        for (; current_step_copy != max_steps_fwd; ++current_step_copy)
                        {
                            // iterators over geometry of current step
                            auto begin =
                                leg_geometry.locations.begin() + current_step_copy->geometry_begin;
                            auto end =
                                leg_geometry.locations.begin() + current_step_copy->geometry_end;
                            auto via_match = std::find_if(begin, end, [&](const auto &location) {
                                return location == via_node_coords;
                            });
                            if (via_match == end)
                                continue;
                            // found a match; this route makes the turn that the maneuver relation
                            // wants to modify
                            done = true;
                            BOOST_ASSERT(maneuver_relation.override_type !=
                                         extractor::guidance::TurnType::Invalid);

                            // Now, if we matched the `via` on this geometry, it's the *next*
                            // step that needs to be updated - geometry of the current step goes
                            // away from the current turn location, and the last node in the
                            // geometry is the location of the next step.
                            ++current_step_copy;
                            // Make sure we don't go past the end of the list
                            if (current_step_copy < leg.steps.end())
                            {
                                current_step_copy->maneuver.instruction.type =
                                    maneuver_relation.override_type;
                                if (maneuver_relation.direction !=
                                    extractor::guidance::DirectionModifier::MaxDirectionModifier)
                                {
                                    current_step_copy->maneuver.instruction.direction_modifier =
                                        maneuver_relation.direction;
                                }
                            }
                            break;
                        }
                        if (done)
                            break;
                    }
                    if (done)
                        break;
                }
            }

            leg_geometries.push_back(std::move(leg_geometry));
            legs.push_back(std::move(leg));
        }

        auto route = guidance::assembleRoute(legs);
        boost::optional<util::json::Value> json_overview;
        if (parameters.overview != RouteParameters::OverviewType::False)
        {
            const auto use_simplification =
                parameters.overview == RouteParameters::OverviewType::Simplified;
            BOOST_ASSERT(use_simplification ||
                         parameters.overview == RouteParameters::OverviewType::Full);

            auto overview = guidance::assembleOverview(leg_geometries, use_simplification);
            json_overview = MakeGeometry(overview.begin(), overview.end());
        }

        std::vector<util::json::Value> step_geometries;
        for (const auto idx : util::irange<std::size_t>(0UL, legs.size()))
        {
            auto &leg_geometry = leg_geometries[idx];

            step_geometries.reserve(step_geometries.size() + legs[idx].steps.size());

            std::transform(
                legs[idx].steps.begin(),
                legs[idx].steps.end(),
                std::back_inserter(step_geometries),
                [this, &leg_geometry](const guidance::RouteStep &step) {
                    if (parameters.geometries == RouteParameters::GeometriesType::Polyline)
                    {
                        return static_cast<util::json::Value>(json::makePolyline<100000>(
                            leg_geometry.locations.begin() + step.geometry_begin,
                            leg_geometry.locations.begin() + step.geometry_end));
                    }

                    if (parameters.geometries == RouteParameters::GeometriesType::Polyline6)
                    {
                        return static_cast<util::json::Value>(json::makePolyline<1000000>(
                            leg_geometry.locations.begin() + step.geometry_begin,
                            leg_geometry.locations.begin() + step.geometry_end));
                    }

                    BOOST_ASSERT(parameters.geometries == RouteParameters::GeometriesType::GeoJSON);
                    return static_cast<util::json::Value>(json::makeGeoJSONGeometry(
                        leg_geometry.locations.begin() + step.geometry_begin,
                        leg_geometry.locations.begin() + step.geometry_end));
                });
        }

        std::vector<util::json::Object> annotations;

        // To maintain support for uses of the old default constructors, we check
        // if annotations property was set manually after default construction
        auto requested_annotations = parameters.annotations_type;
        if ((parameters.annotations == true) &&
            (parameters.annotations_type == RouteParameters::AnnotationsType::None))
        {
            requested_annotations = RouteParameters::AnnotationsType::All;
        }

        if (requested_annotations != RouteParameters::AnnotationsType::None)
        {
            for (const auto idx : util::irange<std::size_t>(0UL, leg_geometries.size()))
            {
                auto &leg_geometry = leg_geometries[idx];
                util::json::Object annotation;

                // AnnotationsType uses bit flags, & operator checks if a property is set
                if (parameters.annotations_type & RouteParameters::AnnotationsType::Speed)
                {
                    annotation.values["speed"] = GetAnnotations(
                        leg_geometry, [](const guidance::LegGeometry::Annotation &anno) {
                            auto val = std::round(anno.distance / anno.duration * 10.) / 10.;
                            return util::json::clamp_float(val);
                        });
                }

                if (requested_annotations & RouteParameters::AnnotationsType::Duration)
                {
                    annotation.values["duration"] = GetAnnotations(
                        leg_geometry, [](const guidance::LegGeometry::Annotation &anno) {
                            return anno.duration;
                        });
                }
                if (requested_annotations & RouteParameters::AnnotationsType::Distance)
                {
                    annotation.values["distance"] = GetAnnotations(
                        leg_geometry, [](const guidance::LegGeometry::Annotation &anno) {
                            return anno.distance;
                        });
                }
                if (requested_annotations & RouteParameters::AnnotationsType::Weight)
                {
                    annotation.values["weight"] = GetAnnotations(
                        leg_geometry,
                        [](const guidance::LegGeometry::Annotation &anno) { return anno.weight; });
                }
                if (requested_annotations & RouteParameters::AnnotationsType::Datasources)
                {
                    annotation.values["datasources"] = GetAnnotations(
                        leg_geometry, [](const guidance::LegGeometry::Annotation &anno) {
                            return anno.datasource;
                        });
                }
                if (requested_annotations & RouteParameters::AnnotationsType::Nodes)
                {
                    util::json::Array nodes;
                    nodes.values.reserve(leg_geometry.osm_node_ids.size());
                    std::for_each(leg_geometry.osm_node_ids.begin(),
                                  leg_geometry.osm_node_ids.end(),
                                  [this, &nodes](const OSMNodeID &node_id) {
                                      nodes.values.push_back(static_cast<std::uint64_t>(node_id));
                                  });
                    annotation.values["nodes"] = std::move(nodes);
                }

                annotations.push_back(std::move(annotation));
            }
        }

        auto result = json::makeRoute(route,
                                      json::makeRouteLegs(std::move(legs),
                                                          std::move(step_geometries),
                                                          std::move(annotations)),
                                      std::move(json_overview),
                                      facade.GetWeightName());

        return result;
    }

    const RouteParameters &parameters;
};

} // ns api
} // ns engine
} // ns osrm

#endif
