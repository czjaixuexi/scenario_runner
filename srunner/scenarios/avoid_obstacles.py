#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Stop sign scenario:

"""

import random
import math
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, RunningStopTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               InTriggerDistanceToLocation,
                                                                               InTriggerDistanceToLocationAlongRoute,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_location_in_distance_from_wp, get_waypoint_in_distance


class AvoidingObstacles(BasicScenario):

    """
    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60, report_enable=False):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._transform = None

        self._stopped_vehicle_location = 20

        self._leading_vehicle_location = 40
        self._leading_vehicle_speed = 0.00


        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 1
        self._other_actor_transform = None

        super(AvoidingObstacles, self).__init__("AvoidingObstacles",
                                                        ego_vehicles,
                                                        config,
                                                        world,
                                                        debug_mode,
                                                        criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        # add actors from xml file
        for actor in config.other_actors:
            vehicle = CarlaDataProvider.request_new_actor(actor.model, actor.transform)
            self.other_actors.append(vehicle)
            # vehicle.set_simulate_physics(enabled=False)

        stopped_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._stopped_vehicle_location)
        self._other_actor_transform = carla.Transform(
            carla.Location(stopped_vehicle_waypoint.transform.location.x,
                           stopped_vehicle_waypoint.transform.location.y ,
                           stopped_vehicle_waypoint.transform.location.z),
            stopped_vehicle_waypoint.transform.rotation)
        stopped_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x-1,
                           self._other_actor_transform.location.y ,
                           self._other_actor_transform.location.z ),
            self._other_actor_transform.rotation)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # to avoid the other actor blocking traffic, it was spawed elsewhere
        # reset its pose to the required one
        stopped_vehicle_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)

        # let the leading vehicle drive until next intersection
        driving_to_next_intersection = py_trees.composites.Parallel(
            "Driving towards Intersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[1], self._leading_vehicle_speed))
        
        driving_to_next_intersection.add_child(InTriggerDistanceToNextIntersection(self.other_actors[1], 10))


        # stop vehicle
        stop = StopVehicle(self.other_actors[1], self._other_actor_max_brake)

        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[1],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        # sequence.add_child(stopped_vehicle_transform)
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))


        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        stop_sign_criterion = RunningStopTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)
        criteria.append(stop_sign_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
