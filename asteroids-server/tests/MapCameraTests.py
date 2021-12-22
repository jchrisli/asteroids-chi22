from interactions.CommandManager import MapCamera

MAP_CAM_PARAM_PATH = 'j5-720p-calib.json'

mc = MapCamera(MAP_CAM_PARAM_PATH)

intersection = mc.pixel_to_ground([640.0, 360.0], 1280.0, 720.0)
print(f"Intersection at {intersection}")