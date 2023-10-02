import numpy as np
import carla
class NoisyPoseObservation():
    def __init__(self):
        self.noise_covariance = np.eye(3)
        
    def get_obs(self):
        raise NotImplementedError
    
class NoisyVehiclePoseObservation(NoisyPoseObservation):
    def __init__(self, noise_covariance, vehicle):
        super.__init__(noise_covariance)
        self.vehicle = vehicle
        
    def get_obs(self):
        pose = self.vehicle.get_location()
        return pose

    