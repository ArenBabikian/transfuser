#!/usr/bin/env python

# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides Challenge routes as standalone scenarios
"""

from __future__ import print_function

import os
import math
import xml.etree.ElementTree as ET
import numpy.random as random

import py_trees

import carla

from agents.navigation.local_planner import RoadOption
from leaderboard.autoagents.dummy_agent import DummyAgent
import srunner.tools.scenario_helper as scenario_helper

# pylint: disable=line-too-long
from srunner.scenarioconfigs.scenario_configuration import ScenarioConfiguration, ActorConfigurationData
# pylint: enable=line-too-long
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle, ScenarioTriggerer, WaypointFollower
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenarios.control_loss import ControlLoss
from srunner.scenarios.follow_leading_vehicle import FollowLeadingVehicle
from srunner.scenarios.object_crash_vehicle import DynamicObjectCrossing
from srunner.scenarios.object_crash_intersection import VehicleTurningRoute
from srunner.scenarios.other_leading_vehicle import OtherLeadingVehicle
from srunner.scenarios.maneuver_opposite_direction import ManeuverOppositeDirection
from srunner.scenarios.junction_crossing_route import SignalJunctionCrossingRoute, NoSignalJunctionCrossingRoute

from srunner.scenariomanager.scenarioatomics.atomic_criteria_local import (CollisionTest,
                                                                     InRouteTest,
                                                                     RouteCompletionTest,
                                                                     OutsideRouteLanesTest,
                                                                     RunningRedLightTest,
                                                                     RunningStopTest,
                                                                     ActorSpeedAboveThresholdTest)

from leaderboard.utils.route_parser import RouteParser, TRIGGER_THRESHOLD, TRIGGER_ANGLE_THRESHOLD
from leaderboard.utils.route_manipulation import interpolate_trajectory

ROUTESCENARIO = ["RouteScenario"]

SECONDS_GIVEN_PER_METERS = 0.8
INITIAL_SECONDS_DELAY = 5.0

SLOWSPEED = 5
SPEED_TRANSFUSER_IN_JUNCTION = 3
SPEED_TRANSFUSER_OUT_JUNCTION = 4

NUMBER_CLASS_TRANSLATION = {
    "Scenario1": ControlLoss,
    "Scenario2": FollowLeadingVehicle,
    "Scenario3": DynamicObjectCrossing,
    "Scenario4": VehicleTurningRoute,
    "Scenario5": OtherLeadingVehicle,
    "Scenario6": ManeuverOppositeDirection,
    "Scenario7": SignalJunctionCrossingRoute,
    "Scenario8": SignalJunctionCrossingRoute,
    "Scenario9": SignalJunctionCrossingRoute,
    "Scenario10": NoSignalJunctionCrossingRoute
}


def oneshot_behavior(name, variable_name, behaviour):
    """
    This is taken from py_trees.idiom.oneshot.
    """
    # Initialize the variables
    blackboard = py_trees.blackboard.Blackboard()
    _ = blackboard.set(variable_name, False)

    # Wait until the scenario has ended
    subtree_root = py_trees.composites.Selector(name=name)
    check_flag = py_trees.blackboard.CheckBlackboardVariable(
        name=variable_name + " Done?",
        variable_name=variable_name,
        expected_value=True,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
    )
    set_flag = py_trees.blackboard.SetBlackboardVariable(
        name="Mark Done",
        variable_name=variable_name,
        variable_value=True
    )
    # If it's a sequence, don't double-nest it in a redundant manner
    if isinstance(behaviour, py_trees.composites.Sequence):
        behaviour.add_child(set_flag)
        sequence = behaviour
    else:
        sequence = py_trees.composites.Sequence(name="OneShot")
        sequence.add_children([behaviour, set_flag])

    subtree_root.add_children([check_flag, sequence])
    return subtree_root


def convert_json_to_transform(actor_dict):
    """
    Convert a JSON string to a CARLA transform
    """
    return carla.Transform(location=carla.Location(x=float(actor_dict['x']), y=float(actor_dict['y']),
                                                   z=float(actor_dict['z'])),
                           rotation=carla.Rotation(roll=0.0, pitch=0.0, yaw=float(actor_dict['yaw'])))


def convert_json_to_actor(actor_dict):
    """
    Convert a JSON string to an ActorConfigurationData dictionary
    """
    node = ET.Element('waypoint')
    node.set('x', actor_dict['x'])
    node.set('y', actor_dict['y'])
    node.set('z', actor_dict['z'])
    node.set('yaw', actor_dict['yaw'])

    return ActorConfigurationData.parse_from_node(node, 'simulation')


def convert_transform_to_location(transform_vec):
    """
    Convert a vector of transforms to a vector of locations
    """
    location_vec = []
    for transform_tuple in transform_vec:
        location_vec.append((transform_tuple[0].location, transform_tuple[1]))

    return location_vec


def compare_scenarios(scenario_choice, existent_scenario):
    """
    Compare function for scenarios based on distance of the scenario start position
    """
    def transform_to_pos_vec(scenario):
        """
        Convert left/right/front to a meaningful CARLA position
        """
        position_vec = [scenario['trigger_position']]
        if scenario['other_actors'] is not None:
            if 'left' in scenario['other_actors']:
                position_vec += scenario['other_actors']['left']
            if 'front' in scenario['other_actors']:
                position_vec += scenario['other_actors']['front']
            if 'right' in scenario['other_actors']:
                position_vec += scenario['other_actors']['right']

        return position_vec

    # put the positions of the scenario choice into a vec of positions to be able to compare

    choice_vec = transform_to_pos_vec(scenario_choice)
    existent_vec = transform_to_pos_vec(existent_scenario)
    for pos_choice in choice_vec:
        for pos_existent in existent_vec:

            dx = float(pos_choice['x']) - float(pos_existent['x'])
            dy = float(pos_choice['y']) - float(pos_existent['y'])
            dz = float(pos_choice['z']) - float(pos_existent['z'])
            dist_position = math.sqrt(dx * dx + dy * dy + dz * dz)
            dyaw = float(pos_choice['yaw']) - float(pos_choice['yaw'])
            dist_angle = math.sqrt(dyaw * dyaw)
            if dist_position < TRIGGER_THRESHOLD and dist_angle < TRIGGER_ANGLE_THRESHOLD:
                return True

    return False


class RouteScenario(BasicScenario):

    """
    Implementation of a RouteScenario, i.e. a scenario that consists of driving along a pre-defined route,
    along which several smaller scenarios are triggered
    """

    # TODO Eventually, I should create a IntersectionTestScenario

    category = "RouteScenario"

    def __init__(self, world, config, debug_mode=0, criteria_enable=True):
        """
        Setup all relevant parameters and create scenarios along route
        """
        self.config = config
        self.route = None
        self.sampled_scenarios_definitions = None
        self.world = world
        self.debug = debug_mode>0

        # NOTE Derives dense route for ego_vehicle
        self._update_route(world, config, debug_mode>0)

        # NOTE ego_vehicle creation below
        ego_vehicle = self._update_ego_vehicle()

        # Adding the other actors is in either of the two commands below
        # List of scenario object instances, each object is a type of scenario, witj (potentially) conditional behaviors
        # NOTE Does not get in here, since we are not using "scenario" files
        self.list_scenarios = self._build_scenario_instances(world,
                                                             ego_vehicle,
                                                             self.sampled_scenarios_definitions,
                                                             scenarios_per_tick=10,
                                                             timeout=self.timeout,
                                                             debug_mode=debug_mode>1)

        # NOTE most of the magic happens below
        super(RouteScenario, self).__init__(name=config.name,
                                            ego_vehicles=[ego_vehicle],
                                            config=config,
                                            world=world,
                                            debug_mode=debug_mode>1,
                                            terminate_on_failure=False,
                                            criteria_enable=criteria_enable)


    def _update_route(self, world, config, debug_mode):
        """
        Update the input route, i.e. refine waypoint list, and extract possible scenario locations

        Parameters:
        - world: CARLA world
        - config: Scenario configuration (RouteConfiguration)
        """

        # TEMPORARY : Below needed only for if we dont wanna load Transfuser agents
        # config.agent = DummyAgent('')

        # Transform the scenario file into a dictionary
        world_annotations = RouteParser.parse_annotations_file(config.scenario_file)

        # prepare route's dense trajectory (interpolate and add the GPS route)
        # print('--------------------')
        # print('INPUT')
        # print(config.trajectory)

        # below is replaced: now, we are only giving initial position and maneuver for ego, and we interpolate the path from that
        # gps_route, route = interpolate_trajectory(world, config.trajectory)
        wp = CarlaDataProvider.get_map().get_waypoint(config.ego_spec.transform.location)
        gps_route, routes = self.get_dense_path(wp, config.ego_spec.maneuver)
        assert len(routes) == 1
        route = routes[0]
        config.trajectory = [point[0].location for point in route]

        potential_scenarios_definitions, _ = RouteParser.scan_route_for_scenarios(
            config.town, route, world_annotations)

        self.route = route
        CarlaDataProvider.set_ego_vehicle_route(convert_transform_to_location(self.route))

        # Downsampling happens below
        config.agent.set_global_plan(gps_route, self.route)

        # Sample the scenarios to be used for this route instance.
        self.sampled_scenarios_definitions = self._scenario_sampling(potential_scenarios_definitions)

        # Timeout of scenario in seconds
        if config.timeout:
            self.timeout = config.timeout
        else:
            self.timeout = self._estimate_route_timeout()

        # Print route in debug mode
        if debug_mode:
            # print('OUTPUT')
            # print(self.route)
            self._draw_waypoints(world, self.route, vertical_shift=1.0, persistency=50000.0)

    def _update_ego_vehicle(self):
        """
        Set/Update the start position of the ego_vehicle
        """
        # move ego to correct position
        elevate_transform = self.route[0][0]
        elevate_transform.location.z += 0.5

        ego_vehicle = CarlaDataProvider.request_new_actor('vehicle.tesla.model3',
                                                          elevate_transform,
                                                          rolename='hero')

        spectator = CarlaDataProvider.get_world().get_spectator()
        ego_trans = ego_vehicle.get_transform()
        spectator.set_transform(carla.Transform(ego_trans.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

        return ego_vehicle

    def _estimate_route_timeout(self):
        """
        Estimate the duration of the route
        """
        route_length = 0.0  # in meters

        prev_point = self.route[0][0]
        for current_point, _ in self.route[1:]:
            dist = current_point.location.distance(prev_point.location)
            route_length += dist
            prev_point = current_point

        return int(SECONDS_GIVEN_PER_METERS * route_length + INITIAL_SECONDS_DELAY)

    # pylint: disable=no-self-use
    def _draw_waypoints(self, world, waypoints, vertical_shift, persistency=-1):
        """
        Draw a list of waypoints at a certain height given in vertical_shift.
        """
        for w in waypoints:
            wp = w[0].location + carla.Location(z=vertical_shift)

            size = 0.2
            if w[1] == RoadOption.LEFT:  # Yellow
                color = carla.Color(255, 255, 0)
            elif w[1] == RoadOption.RIGHT:  # Cyan
                color = carla.Color(0, 255, 255)
            elif w[1] == RoadOption.CHANGELANELEFT:  # Orange
                color = carla.Color(255, 64, 0)
            elif w[1] == RoadOption.CHANGELANERIGHT:  # Dark Cyan
                color = carla.Color(0, 64, 255)
            elif w[1] == RoadOption.STRAIGHT:  # Gray
                color = carla.Color(128, 128, 128)
            else:  # LANEFOLLOW
                color = carla.Color(0, 255, 0) # Green
                size = 0.1

            world.debug.draw_point(wp, size=size, color=color, life_time=persistency)

        world.debug.draw_point(waypoints[0][0].location + carla.Location(z=vertical_shift), size=0.2,
                               color=carla.Color(0, 0, 255), life_time=persistency)
        world.debug.draw_point(waypoints[-1][0].location + carla.Location(z=vertical_shift), size=0.2,
                               color=carla.Color(255, 0, 0), life_time=persistency)

    def _scenario_sampling(self, potential_scenarios_definitions, random_seed=0):
        """
        The function used to sample the scenarios that are going to happen for this route.
        """

        # fix the random seed for reproducibility
        rgn = random.RandomState(random_seed)

        def position_sampled(scenario_choice, sampled_scenarios):
            """
            Check if a position was already sampled, i.e. used for another scenario
            """
            for existent_scenario in sampled_scenarios:
                # If the scenarios have equal positions then it is true.
                if compare_scenarios(scenario_choice, existent_scenario):
                    return True

            return False

        def select_scenario(list_scenarios):
            # priority to the scenarios with higher number: 10 has priority over 9, etc.
            higher_id = -1
            selected_scenario = None
            for scenario in list_scenarios:
                try:
                    scenario_number = int(scenario['name'].split('Scenario')[1])
                except:
                    scenario_number = -1

                if scenario_number >= higher_id:
                    higher_id = scenario_number
                    selected_scenario = scenario

            return selected_scenario

        # The idea is to randomly sample a scenario per trigger position.
        sampled_scenarios = []
        for trigger in potential_scenarios_definitions.keys():
            possible_scenarios = potential_scenarios_definitions[trigger]

            scenario_choice = select_scenario(possible_scenarios)
            del possible_scenarios[possible_scenarios.index(scenario_choice)]
            # We keep sampling and testing if this position is present on any of the scenarios.
            while position_sampled(scenario_choice, sampled_scenarios):
                if possible_scenarios is None or not possible_scenarios:
                    scenario_choice = None
                    break
                scenario_choice = rgn.choice(possible_scenarios)
                del possible_scenarios[possible_scenarios.index(scenario_choice)]

            if scenario_choice is not None:
                sampled_scenarios.append(scenario_choice)

        return sampled_scenarios

    def _build_scenario_instances(self, world, ego_vehicle, scenario_definitions,
                                  scenarios_per_tick=5, timeout=300, debug_mode=False):
        """
        Based on the parsed route and possible scenarios, build all the scenario classes.
        """
        scenario_instance_vec = []

        if debug_mode:
            for scenario in scenario_definitions:
                loc = carla.Location(scenario['trigger_position']['x'],
                                     scenario['trigger_position']['y'],
                                     scenario['trigger_position']['z']) + carla.Location(z=2.0)
                world.debug.draw_point(loc, size=0.3, color=carla.Color(255, 0, 0), life_time=100000)
                world.debug.draw_string(loc, str(scenario['name']), draw_shadow=False,
                                        color=carla.Color(0, 0, 255), life_time=100000, persistent_lines=True)

        for scenario_number, definition in enumerate(scenario_definitions):
            # Get the class possibilities for this scenario number
            scenario_class = NUMBER_CLASS_TRANSLATION[definition['name']]

            # Create the other actors that are going to appear
            if definition['other_actors'] is not None:
                list_of_actor_conf_instances = self._get_actors_instances(definition['other_actors'])
            else:
                list_of_actor_conf_instances = []
            # Create an actor configuration for the ego-vehicle trigger position

            egoactor_trigger_position = convert_json_to_transform(definition['trigger_position'])
            scenario_configuration = ScenarioConfiguration()
            scenario_configuration.other_actors = list_of_actor_conf_instances
            scenario_configuration.trigger_points = [egoactor_trigger_position]
            scenario_configuration.subtype = definition['scenario_type']
            scenario_configuration.ego_vehicles = [ActorConfigurationData('vehicle.lincoln.mkz2017',
                                                                          ego_vehicle.get_transform(),
                                                                          'hero')]
            route_var_name = "ScenarioRouteNumber{}".format(scenario_number)
            scenario_configuration.route_var_name = route_var_name
            try:
                scenario_instance = scenario_class(world, [ego_vehicle], scenario_configuration,
                                                   criteria_enable=False, timeout=timeout)
                # Do a tick every once in a while to avoid spawning everything at the same time
                if scenario_number % scenarios_per_tick == 0:
                    if CarlaDataProvider.is_sync_mode():
                        world.tick()
                    else:
                        world.wait_for_tick()

            except Exception as e:
                print("Skipping scenario '{}' due to setup error: {}".format(definition['name'], e))
                continue

            scenario_instance_vec.append(scenario_instance)

        return scenario_instance_vec

    def _get_actors_instances(self, list_of_antagonist_actors):
        """
        Get the full list of actor instances.
        """

        def get_actors_from_list(list_of_actor_def):
            """
                Receives a list of actor definitions and creates an actual list of ActorConfigurationObjects
            """
            sublist_of_actors = []
            for actor_def in list_of_actor_def:
                sublist_of_actors.append(convert_json_to_actor(actor_def))

            return sublist_of_actors

        list_of_actors = []
        # Parse vehicles to the left
        if 'front' in list_of_antagonist_actors:
            list_of_actors += get_actors_from_list(list_of_antagonist_actors['front'])

        if 'left' in list_of_antagonist_actors:
            list_of_actors += get_actors_from_list(list_of_antagonist_actors['left'])

        if 'right' in list_of_antagonist_actors:
            list_of_actors += get_actors_from_list(list_of_antagonist_actors['right'])

        return list_of_actors

    # pylint: enable=no-self-use

    
    def _initialize_actors(self, config):
        """
        Default initialization of other actors.
        Override this method in child class to provide custom initialization.
        """
        if config.other_actors:
            new_actors = CarlaDataProvider.request_new_actors(config.other_actors)
            if not new_actors:
                raise Exception("Error: Unable to add actors")

            for new_actor in new_actors:
                self.other_actors.append(new_actor)

    def get_dense_path(self, starting_wp, maneuver, split_path = False):

        # TODO take location as input and transform to wp in here
        
        starting_loc = starting_wp.transform.location

        # STEP 1 : get path part 1 (actor reaches intersection, then performs maneuver)
        # post_man_wp = scenario_helper.generate_target_waypoint(starting_wp, turn=config.maneuver)
        coarse_plan, post_man_wp, pre_junc_wp = scenario_helper.generate_target_waypoint_list(starting_wp, turn=maneuver)
        post_man_loc = post_man_wp.transform.location
        pre_junc_loc = pre_junc_wp.transform.location
        plan_to_draw = [(t[0].transform, t[1]) for t in coarse_plan]
        # print('-----------------')
        # print('COARSE_PLAN')
        # print(coarse_plan)
        # print(post_man_loc)

        # STEP 2 : drive for some distance after completing maneuver
        final_wp, _ = scenario_helper.get_waypoint_in_distance(post_man_wp, 5)
        final_loc = final_wp.transform.location
        plan_to_draw.append((final_wp.transform, RoadOption.LANEFOLLOW))

        # STEP 3 : Derive the dense path, and get a plan from that
        if split_path:
            # STEP 3.2: derive 3 subpaths
            if pre_junc_loc != None:
                coarse_path_1 = [starting_loc, pre_junc_loc]
                _, dense_path_1 = interpolate_trajectory(self.world, coarse_path_1)
            else:
                dense_path_1 = None

            coarse_path_2 = [pre_junc_loc, post_man_loc]
            _, dense_path_2 = interpolate_trajectory(self.world, coarse_path_2)

            coarse_path_3 = [post_man_loc, final_loc]
            _, dense_path_3 = interpolate_trajectory(self.world, coarse_path_3)

            return None, [dense_path_1, dense_path_2, dense_path_3]
        else:
            # STEP 3.1: derive a single path        
            very_coarse_path = [starting_loc, post_man_loc, final_loc]
            # print('INPUT')
            # print(very_coarse_path)

            gps_dense_path, dense_path = interpolate_trajectory(self.world, very_coarse_path)
            # print('OUTPUT (ROUTE)')
            # print(dense_path)

            return gps_dense_path, [dense_path]

        # STEP TODO : Futur challenge is to not use the center of the lane
        # maybe ask non-ego to drive near the lane edge.
        # No idea where to start for this 


    def _create_behavior(self):
        """
        Basic behavior do nothing, i.e. Idle
        """
        scenario_trigger_distance = 1.5  # Max trigger distance between route and scenario

        behavior = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        subbehavior = py_trees.composites.Parallel(name="Behavior",
                                                   policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

        scenario_behaviors = []
        blackboard_list = []

        # Does not go in here, since we are not working with scenario files
        for i, scenario in enumerate(self.list_scenarios):
            if scenario.scenario.behavior is not None:
                route_var_name = scenario.config.route_var_name

                if route_var_name is not None:
                    scenario_behaviors.append(scenario.scenario.behavior)
                    blackboard_list.append([scenario.config.route_var_name,
                                            scenario.config.trigger_points[0].location])
                else:
                    name = "{} - {}".format(i, scenario.scenario.behavior.name)
                    oneshot_idiom = oneshot_behavior(
                        name=name,
                        variable_name=name,
                        behaviour=scenario.scenario.behavior)
                    scenario_behaviors.append(oneshot_idiom)

        # Add ego_vehicle behavior that manages the scenarios trigger conditions
        # This is doing a lot of extra stuff,, like handling trigger conditions, that we dont need
        scenario_triggerer = ScenarioTriggerer(
            self.ego_vehicles[0],
            self.route,
            blackboard_list,
            scenario_trigger_distance,
            repeat_scenarios=False
        )

        subbehavior.add_child(scenario_triggerer)  # make ScenarioTriggerer the first thing to be checked

        ########################
        # Add other_actor behavior here
        # for actor in self.config.other_actors:
        for i, actor in enumerate(self.other_actors):

            # STEP 0 : Set up
            config = self.config.other_actors[i]
            starting_wp = CarlaDataProvider.get_map().get_waypoint(config.transform.location)

            _, dense_routes = self.get_dense_path(starting_wp, config.maneuver, config.speed == 'transfuser')
            # print('OFFICIAL PLAN')
            # print(plan)

            actor_behavior = py_trees.composites.Sequence()

            # STEP 1 : Derive the plans
            if config.speed == 'transfuser':
                assert len(dense_routes) == 3
                # handle 3 SUBPLANS
                # 3m/s inside junction
                # 4m/s outside junction

                if dense_routes[0] != None:
                    plan_1 = self.route2plan(dense_routes[0])
                    wpfoll_1 = WaypointFollower(actor, SPEED_TRANSFUSER_OUT_JUNCTION, plan=plan_1, avoid_collision=False)
                    actor_behavior.add_child(wpfoll_1)

                plan_2 = self.route2plan(dense_routes[1])
                wpfoll_2 = WaypointFollower(actor, SPEED_TRANSFUSER_IN_JUNCTION, plan=plan_2, avoid_collision=False)
                actor_behavior.add_child(wpfoll_2)

                plan_3 = self.route2plan(dense_routes[2])
                wpfoll_3 = WaypointFollower(actor, SPEED_TRANSFUSER_IN_JUNCTION, plan=plan_3, avoid_collision=False)
                actor_behavior.add_child(wpfoll_3)

                if self.debug:
                    self._draw_waypoints(self.world, dense_routes[0], vertical_shift=1.0, persistency=50000.0)
                    wp_mid_1 = dense_routes[0][-1][0].location + carla.Location(z=1)
                    self.world.debug.draw_point(wp_mid_1, size=0.3, color=carla.Color(255, 0, 0), life_time=50000)
                    self._draw_waypoints(self.world, dense_routes[1], vertical_shift=1.0, persistency=50000.0)
                    wp_mid_2 = dense_routes[1][-1][0].location + carla.Location(z=1)
                    self.world.debug.draw_point(wp_mid_2, size=0.3, color=carla.Color(255, 0, 0), life_time=50000)
                    self._draw_waypoints(self.world, dense_routes[2], vertical_shift=1.0, persistency=50000.0)

            else:
                # assign the same speed to the SINGLE PLAN
                assert len(dense_routes) == 1
                dense_route = dense_routes[0]
                plan_to_draw=dense_route
                plan = self.route2plan(dense_route)

                wpfoll_behavior = WaypointFollower(actor, float(config.speed), plan=plan, avoid_collision=False)
                actor_behavior.add_child(wpfoll_behavior)

                if self.debug:
                    self._draw_waypoints(self.world, plan_to_draw, vertical_shift=1.0, persistency=50000.0)

            # STEP 2 : add post-post-junction subpath behavior
            actor_behavior.add_child(WaypointFollower(actor, SLOWSPEED, avoid_collision=False))

            subbehavior.add_child(actor_behavior)

        # NOTE below is irrelevant for us
        # subbehavior.add_children(scenario_behaviors)

        subbehavior.add_child(Idle())  # The behaviours cannot make the route scenario stop
        behavior.add_child(subbehavior)
        return behavior
    
    def route2plan(self, dense_route):
        get_waypoint = CarlaDataProvider.get_map().get_waypoint
        return [(get_waypoint(point[0].location), point[1]) for point in dense_route]


    def _create_test_criteria(self):
        """
        """
        criteria = []
        route = convert_transform_to_location(self.route)

        collision_criterion = CollisionTest(self.ego_vehicles[0], terminate_on_failure=True)

        route_criterion = InRouteTest(self.ego_vehicles[0],
                                      route=route,
                                      offroad_max=30,
                                      terminate_on_failure=True)
                                      
        completion_criterion = RouteCompletionTest(self.ego_vehicles[0], route=route)

        outsidelane_criterion = OutsideRouteLanesTest(self.ego_vehicles[0], route=route)

        red_light_criterion = RunningRedLightTest(self.ego_vehicles[0])

        stop_criterion = RunningStopTest(self.ego_vehicles[0])

        blocked_criterion = ActorSpeedAboveThresholdTest(self.ego_vehicles[0],
                                                         speed_threshold=0.1,
                                                         below_threshold_max_time=180.0,
                                                         terminate_on_failure=True,
                                                         name="AgentBlockedTest")

        criteria.append(completion_criterion)
        criteria.append(outsidelane_criterion)
        criteria.append(collision_criterion)
        criteria.append(red_light_criterion)
        criteria.append(stop_criterion)
        criteria.append(route_criterion)
        criteria.append(blocked_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
