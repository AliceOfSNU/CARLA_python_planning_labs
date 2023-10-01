import pygame
import carla
from Agents import *
import random 
import numpy as np

class RenderObject(object):
    def __init__(self, width, height):
        init_image = np.random.randint(0,255,(height,width,3),dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0,1))

# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0,1))
    
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    actors_list = []
    # create a vehicle
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.bmw.*'))
    ego = CarAgent(world, vehicle_bp)
    actors_list.append(ego.vehicle)
    spectator = world.get_spectator()

    #create a camera
    camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego.vehicle)
    camera.listen(lambda image: pygame_callback(image, renderObject))
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()

    #pygame
    renderObject = RenderObject(image_w, image_h)
    pygame.init()
    gameDisplay = pygame.display.set_mode((image_w,image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
    # Draw black to the display
    gameDisplay.fill((0,0,0))
    gameDisplay.blit(renderObject.surface, (0,0))
    pygame.display.flip()
        
    #main loop
    crashed = False
    while not crashed:
        # Advance the simulation time
        world.tick()
        
        # step agent
        ego.run_step()
        
        # step pygame
        gameDisplay.blit(renderObject.surface, (0,0))
        pygame.display.flip()
        
        # move spectator
        transform = ego.vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50), carla.Rotation(pitch=-90)))
        
        # Collect key press events
        for event in pygame.event.get():
            # If the window is closed, break the while loop
            if event.type == pygame.QUIT:
                crashed = True
            # Parse effect of key press event on control state
            if event.type == pygame.KEYUP:
                if event.key == pygame.K_l:
                    ego.set_desired_state(AgentState.CHANGELANELEFT)
                elif event.key == pygame.K_r: #Perform right lane change
                    ego.set_desired_state(AgentState.CHANGELANERIGHT)
                elif event.key == pygame.K_f: #continue on the lane
                    ego.set_desired_state(AgentState.LANEFOLLOW)
                elif event.key == pygame.K_s: #stop
                    ego.set_desired_state(AgentState.STOP)
                elif event.key == pygame.K_q:
                    crashed = True
                    break
                print(ego.desired_state)

finally:
    print("Destroying actors")
    client.apply_batch([carla.command.DestroyActor(x) for x in actors_list])

