import carla
import numpy
from collections import deque
from collections import OrderedDict
from enum import Enum

class TrackState(Enum):
    VOID = -1
    TENTATIVE = 1
    ESTABLISHED = 2
    
class Track():
    def __init__(self, track_id, position, heading, velocity = 0.0, lane_id=None, s=-1.0):
        self.track_id = track_id
        self.position_track = deque(maxlen = 50)
        self.position_track.append(position)
        self.velocity = velocity
        self.heading = heading #degrees
        # lane related info
        self.lane_id = lane_id
        self.s = s
        self.front_vehicle = None
        self.hind_vehicle = None
        # validation
        self.state = TrackState.TENTATIVE
        self.n = 1
        self.establish_thres_n = 5
        
        
    def has_next(self):
        return self.front_vehicle is not None
    
    def has_prev(self):
        return self.hind_vehicle is not None
    
    def update(self, position, heading, velocity, lane_id = None, s=None):
        self.position = position
        if len(self.position_track) >= self.position_track.maxlen:
            self.position_track.popleft()
        self.position_track.append(position)
        self.velocity = velocity
        self.heading = heading
        if lane_id:
            self.lane_id = lane_id
        if s:
            self.s = s
        if self.n > self.establish_thres_n: self.state = TrackState.ESTABLISHED
        
    
        
class LaneTracks():
    def __init__(self, lane_id):
        self.lane_id = lane_id
        self.head:Track = None
        self.tail:Track = None
        self.length = 0
        
    def __len__(self):
        return self.length

    def insert(self, s, track: Track):
        if self.length > 0:
            curr = self.tail
            while curr != None and s > curr.s : curr = curr.front_vehicle
            # curr may be None
            track.front_vehicle = curr
            if curr != None:
                track.hind_vehicle = curr.hind_vehicle
                if track.hind_vehicle != None:
                    track.hind_vehicle.front_vehicle = track
                curr.hind_vehicle = track
                
        if track.hind_vehicle == None: self.tail = track
        if track.front_vehicle == None: self.head = track
        self.length += 1
    
    # place behind self.tail
    def push_back(self, track): 
        if self.tail == None:
            self.tail = self.head = track
        else:
            track.front_vehicle = self.tail
            self.tail.hind_vehicle = track
            self.tail = track
        self.length += 1
            
    def remove(self, track_id):
        curr = self.head
        while curr != None and curr.track_id != track_id: curr = curr.hind_vehicle
        if curr != None:
            if curr.has_next(): curr.front_vehicle.hind_vehicle = curr.hind_vehicle
            if curr.has_prev(): curr.hind_vehicle.front_vehicle = curr.front_vehicle
            if curr == self.head: self.head = curr.front_vehicle
            if curr == self.tail: self.tail = curr.hind_vehicle
        self.length -= 1
        assert(self.length >= 0)
        
         
class Tracker():
    def __init__(self, wmap, ego_vehicle):
        self.map = wmap
        self.monitor_vehicles = []
        self.tracks : dict[int, Track]= {}
        ## {lane_id: {is_opposite, vehicle_list}}
        self.tracks_by_lanes : dict[int, LaneTracks]= {}
        self.ego_vehicle = ego_vehicle
        print("initialized track management. updating registry on init:")
        self.update_track_registry({ego_vehicle.id: ego_vehicle})
        print("updating registry done.")
    
    def update_track_registry(self, vehicles_dict):
        print("updating registry")
        if self.ego_vehicle.id not in vehicles_dict:
            vehicles_dict[self.ego_vehicle.id] = self.ego_vehicle
            
        # delete all tracks we are not monitoring
        to_remove = []
        for vehicle_id in self.tracks:
            if vehicle_id not in vehicles_dict:
                to_remove.append(vehicle_id)
                r_track = self.tracks[vehicle_id]
                lane_id = r_track.lane_id
                if lane_id != None and lane_id in self.tracks_by_lanes:
                    self.tracks_by_lanes[lane_id].remove(r_track.track_id)
                    if len(self.tracks_by_lanes[lane_id]) == 0: self.tracks_by_lanes.pop(lane_id)
                print("removed stale track: ", vehicle_id)
        for track_id in to_remove:
            self.tracks.pop(track_id)
        
        self.monitor_vehicles = vehicles_dict
        
        for vid, vehicle in self.monitor_vehicles.items():
            # create a track
            if vid not in self.tracks:
                t = vehicle.get_transform()
                wp = self.map.get_waypoint(t.location)
                n_track_id = vid # association is simple.. using the vehicle id as trackid.
                n_track = Track(n_track_id, t.location, t.rotation.yaw, vehicle.get_velocity(), wp.lane_id, wp.s)
                self.tracks[n_track_id] = n_track
                
                if wp.lane_id not in self.tracks_by_lanes:
                    self.tracks_by_lanes[wp.lane_id] = LaneTracks(wp.lane_id)
                    self.tracks_by_lanes[wp.lane_id].push_back(n_track)
                    print("created lane track: ", wp.lane_id)
                # insert in location
                else: self.tracks_by_lanes[wp.lane_id].insert(wp.s, n_track)
                print("created track for vehicle:", vid)
                print("tracks: ", self.tracks)
                
            for track in self.tracks.values():
                track.n = min(track.n+1, 10)
                
                    
    def update_tracks(self):
        for track_id, track in self.tracks.items():
            vehicle = self.monitor_vehicles[track_id]
            assert(vehicle != None)
            t = vehicle.get_transform()
            wp = self.map.get_waypoint(t.location)
            if wp == None:
                print("[Tracker:update_tracks] failed to get wp for track", track_id)
                continue
            
            # account for dynamic lane changes
            prev_lane_id = track.lane_id
            if prev_lane_id != None and wp.lane_id != None and wp.lane_id != prev_lane_id:
                self.tracks_by_lanes[prev_lane_id].remove(track_id)
                if len(self.tracks_by_lanes[prev_lane_id]) <= 0:
                    self.tracks_by_lanes.pop(prev_lane_id)
                if wp.lane_id not in self.tracks_by_lanes:
                    self.tracks_by_lanes[wp.lane_id] = LaneTracks(wp.lane_id)
                    self.tracks_by_lanes[wp.lane_id].push_back(track)
                    print("created lane track: ", wp.lane_id)
                else: self.tracks_by_lanes[wp.lane_id].insert(wp.s, track)
                print("updated lane for ", track_id, " ", prev_lane_id, "->", wp.lane_id)
            track.update(t.location, t.rotation.yaw, vehicle.get_velocity(), lane_id = wp.lane_id, s = wp.s)
            