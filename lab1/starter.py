import pygame
import carla
from Agents import *
import numpy as np
from numpy import random
from Tracker import Tracker

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
    vehicles_list = []
    
    # create a vehicle
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.bmw.*'))
    ego = CarAgent(world, vehicle_bp)
    actors_list.append(ego.vehicle)
    spectator = world.get_spectator()

    #create a camera
    camera_init_trans = carla.Transform(carla.Location(x=-5, z=3), carla.Rotation(pitch=-20))
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=ego.vehicle)
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    renderObject = RenderObject(image_w, image_h)
    camera.listen(lambda image: pygame_callback(image, renderObject))
    
    #pygame
    pygame.init()
    gameDisplay = pygame.display.set_mode((image_w,image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
    
    # and spawn some more cars
    print("generating traffic")
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    if False:
        traffic_manager.set_respawn_dormant_vehicles(True)
    
    traffic_manager.set_random_device_seed(1)
    traffic_manager.set_synchronous_mode(True)
    synchronous_master = False
    settings = world.get_settings()
    if not settings.synchronous_mode:
        synchronous_master = True
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
    else:
        synchronous_master = False

    v_filter = 'vehicle.*'
    blueprints =  world.get_blueprint_library().filter(v_filter)
    blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
    blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
    blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
    blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
    blueprints = [x for x in blueprints if not x.id.endswith('t2')]
    blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
    blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
    blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]
    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)
    number_of_vehicles = 20
    if number_of_vehicles < number_of_spawn_points:
        random.shuffle(spawn_points)
    elif number_of_vehicles > number_of_spawn_points:
        msg = 'requested %d vehicles, but could only find %d spawn points'
        print(msg)
        number_of_vehicles = number_of_spawn_points


    ##################
    # Spawn vehicles #
    ##################
    batch = []
    for n, transform in enumerate(spawn_points):
        if n >= number_of_vehicles:
            break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        else:
            blueprint.set_attribute('role_name', 'autopilot')
        npc = world.try_spawn_actor(blueprint, transform)
        vehicles_list.append(npc)
        npc.set_autopilot(True)

    
    ###########################
    #### track management #####
    ###########################
    from Tracker import *
    trackmgmt = Tracker(world.get_map(), ego.vehicle)
    monitor_radius = 30.0
    
    
    # Draw black to the display
    gameDisplay.fill((0,0,0))
    gameDisplay.blit(renderObject.surface, (0,0))
    pygame.display.flip()
        
    #main loop
    crashed = False
    nticks = 0
    while not crashed:
        # Advance the simulation time
        world.tick()
        nticks = (nticks+1)%1000000
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

        # update tracks
        trackmgmt.update_tracks()
        # update monitored vehicles
        if nticks % 10 == 0:
            vehicle_dict = {}
            ego_location = ego.vehicle.get_location()
            for vehicle in vehicles_list:
                if vehicle.get_location().distance(ego_location) < monitor_radius:
                    vehicle_dict[vehicle.id] = vehicle
            trackmgmt.update_track_registry(vehicle_dict)
            #debugging
            #for vid, track in trackmgmt.tracks.items():
            #    txt = "track_id:{0}\nlane_id:{1}\ns:{2}".format(track.track_id, track.lane_id, track.s)
            #    world.debug.draw_string(track.position, txt, draw_shadow=False, color='0, 255, 0', life_time=1.0)

finally:
    print("Destroying actors")
    client.apply_batch([carla.command.DestroyActor(x) for x in actors_list])
    client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
    print("Destroyed all actors. Exit clean")
