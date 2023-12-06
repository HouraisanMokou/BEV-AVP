carla_path = 'E:\\WindowsNoEditor\\CarlaUE4.exe'
carla_ip_addr = '127.0.0.1'
carla_port = 5000

carla_vehicle_bp_name = 'vehicle.tesla.model3'
carla_vehicle_transform = None  # (0, 0, 2, 0, 0, 0)
carla_vehicle_autopilot = True

image_params = {'image_size_x': 1920, 'image_size_y': 1080}
sensor_configuration = [
    ('sensor.camera.rgb', [0, 0, 2, 0, 0, 0], image_params),
    ('sensor.camera.rgb', [0, 0, 2, 0, 55, 0], image_params),
    ('sensor.camera.rgb', [0, 0, 2, 0, -55, 0], image_params),
    ('sensor.camera.rgb', [0, 0, 2, 0, 110, 0], image_params),
    ('sensor.camera.rgb', [0, 0, 2, 0, -110, 0], image_params),
    ('sensor.camera.rgb', [0, 0, 2, 0, 180, 0], image_params),
]  # [(bp,transform,param)...]
