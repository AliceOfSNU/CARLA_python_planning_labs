import carla
import numpy
import sys
import glob
import os
import random
from enum import Enum
from collections import deque
sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/*'))
print(glob.glob('/opt/carla-simulator/PythonAPI/carla/*'))
sys.path.append('/opt/carla-simulator/PythonAPI/carla/')
from agents.navigation.controller import VehiclePIDController

class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to other.

    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6
    
class AgentState(Enum):
    """
    Vehicle States
    """
    VOID = -1
    STOP = 0
    LANEFOLLOW = 1
    PREPLCLEFT = 2
    PREPLCRIGHT = 3
    CHANGELANELEFT = 4
    CHANGELANERIGHT = 5
    
    
class CarAgent():
    def __init__(self, world, vehicle_bp):
        self.vehicle = None
        while(self.vehicle is None):
            spawn_points = world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        self.world = world
        self.map = world.get_map()
        
        # state machine
        self.state = AgentState.STOP
        self.next_state = AgentState.STOP
        self.desired_state = AgentState.VOID
        
        # create controller. PID is tuned for dt=0.05
        dt = 1.0 / 20.0
        args_lateral_dict = {'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': dt}
        args_longitudinal_dict = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0, 'dt': dt}
        offset = 0
        max_throt = 0.75
        max_brake = 0.3
        max_steer = 0.8
        self.base_speed = 20.0
        self.target_speed = 20.0
        self.base_min_distance = 3.0
        self.controller = VehiclePIDController(self.vehicle,
                                        args_lateral=args_lateral_dict,
                                        args_longitudinal=args_longitudinal_dict,
                                        offset=offset,
                                        max_throttle=max_throt,
                                        max_brake=max_brake,
                                        max_steering=max_steer)
        
        
        # waypoints and trajectory
        # self.curr_wp = self.map.get_waypoint(self.vehicle.get_location())
        self.target_wp = None
        self.target_road_option = RoadOption.LANEFOLLOW
        self.wp_q = deque(maxlen = 100)
        self.target_lookahead = 1.0
        self.max_plan_length = 20
        self.min_plan_length = 5
        print("ego vehicle ready.state:", self.state)
        
        # behaviour planner state machine
        self.state_transition_cbs = {
            AgentState.LANEFOLLOW: self.on_lane_follow,
            AgentState.CHANGELANELEFT: self.on_change_lane_left,
            AgentState.CHANGELANERIGHT: self.on_change_lane_right
        }

        
    '''
    main 'update' function
    '''
        
    def run_step(self):
        # transition
        self.state = self.next_state
        self.next_state = AgentState.VOID
        if self.state in self.state_transition_cbs:
            self.state_transition_cbs[self.state]()
        
        self._validate_next_waypoints()
        if len(self.wp_q) < self.min_plan_length:
            self._compute_next_waypoints() # fill in next waypoints
        
        # calculate control from waypoint(motion planner)
        if len(self.wp_q)==0:
            print("no waypoints queued! stopping")
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            control.manual_gear_shift = False
            self.next_state = AgentState.STOP
        else:
            self.target_wp, self.target_road_option = self.wp_q[-1]
            control = self.controller.run_step(self.target_speed, self.target_wp)
        
        # apply control
        self.vehicle.apply_control(control)
        
        # transition
        self._update_next_state()

        
    
    def _compute_next_waypoints(self):
        if len(self.wp_q) < self.min_plan_length:
            k = self.min_plan_length - len(self.wp_q)
            last_wp = None
            if len(self.wp_q) == 0:
                last_wp = self.map.get_waypoint(self.vehicle.get_location())
            else:
                last_wp = self.wp_q[0][0]
            next_wps = list(last_wp.next(self.target_lookahead))
            if len(next_wps) == 0:
                return
            else:
                next_wps = next_wps[0].next_until_lane_end(self.target_lookahead)
                for wp in reversed(next_wps[:k]):
                    self.wp_q.appendleft((wp, RoadOption.LANEFOLLOW))
                    self.world.debug.draw_point(wp.transform.location, life_time=5.0)


    def _validate_next_waypoints(self):
        # Purge the queue of obsolete waypoints
        veh_location = self.vehicle.get_location()
        vehicle_speed = self.vehicle.get_velocity().length()
        _min_distance = self.base_min_distance + 0.5 *vehicle_speed
        num_waypoint_removed = 0
        for waypoint, _ in self.wp_q:
            if len(self.wp_q) - num_waypoint_removed == 1:
                min_distance = 1.0 # Don't remove the last waypoint until very close by
            else:
                min_distance = _min_distance
            if veh_location.distance(waypoint.transform.location) < min_distance:
                num_waypoint_removed += 1
            else:
                break
        if num_waypoint_removed > 0:
            for _ in range(num_waypoint_removed):
                wp, roadoption = self.wp_q[-1]
                if roadoption == RoadOption.CHANGELANELEFT or roadoption == RoadOption.CHANGELANERIGHT:
                    print("completed lane change")
                    self.done = True
                # visualize
                self.wp_q.pop()
        
    def _update_next_state(self):
        next_state = self.state
        if self.state == AgentState.CHANGELANELEFT and self.done \
            or self.state == AgentState.CHANGELANERIGHT and self.done:
            next_state = AgentState.LANEFOLLOW
            
        # if a specific state is ordered.
        if self.desired_state != AgentState.VOID:
            next_state = self.desired_state
            self.desired_state = AgentState.VOID
    
        self.next_state = next_state
        
    ##############################
    ##### getters, setters #######
    ##############################
    def set_speed(self, speed):
        self.target_speed = speed
        
    def set_desired_state(self, state :AgentState):
        self.desired_state = state
        
    def get_plan(self):
        return self.wp_q
    
    def done(self):
        return self.done
    
    ##############################
    ####### FSM CALLBACKS ########
    ##############################
        
    def on_lane_follow(self):
        self.target_speed = self.base_speed
        self.done = True
    
    def on_stop(self):
        self.target_speed = 0.0
        
    def on_change_lane_left(self):
        self.wp_q.clear()
        current_waypoint = self.map.get_waypoint(self.vehicle.get_location())
        left_lane = current_waypoint.get_left_lane()
        next_wps = left_lane.next(self.target_lookahead)
        if len(next_wps) == 0:
            print("no waypoints on the left! continuing on this lane")
            self.next_state = AgentState.LANEFOLLOW
        else: 
            self.wp_q.appendleft((next_wps[0], RoadOption.CHANGELANELEFT))
        self.target_speed = self.base_speed * 0.6
            
    def on_change_lane_right(self):
        self.wp_q.clear()
        current_waypoint = self.map.get_waypoint(self.vehicle.get_location())
        right_lane = current_waypoint.get_right_lane()
        next_wps = right_lane.next(self.target_lookahead)
        if len(next_wps) == 0:
            print("no waypoints on the right! continuing on this lane")
            self.next_state = AgentState.LANEFOLLOW
        else: 
            self.wp_q.appendleft((next_wps[0], RoadOption.CHANGELANERIGHT))
        self.target_speed = self.base_speed * 0.6
        