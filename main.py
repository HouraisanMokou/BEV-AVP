import time
import os
import subprocess
import queue

import carla
import runtime
import random
import cv2
import numpy as np
from loguru import logger


class EgoAgent:
    def __init__(self):
        self.blueprint = None
        self.vehicle_tf = None
        self.vehicle = None
        self.world = None
        self.map = None

        self.image_size = None
        self.sensor_configuration = None
        self.sensor_queue_list = []
        self.sensor_actors = []
        self.sensor_type_list = []

    def setup(self, world, carla_map, blueprint, vehicle_tf, sensor_configuration):
        self.world = world
        self.map = carla_map
        self.blueprint = self.world.get_blueprint_library().find(blueprint)
        if vehicle_tf is None:
            self.vehicle_tf = random.choice(self.map.get_spawn_points())
        else:
            self.vehicle_tf = carla.Transform(
                carla.Location(vehicle_tf[0], vehicle_tf[1], vehicle_tf[2]),
                carla.Rotation(vehicle_tf[3], vehicle_tf[4], vehicle_tf[5])
            )
        self.sensor_configuration = sensor_configuration

    def spawn(self):
        self.vehicle = self.world.try_spawn_actor(self.blueprint, self.vehicle_tf)
        logger.info(f'spawn {self.blueprint} at {self.vehicle_tf}')
        for sc in self.sensor_configuration:
            self.register_sensor(sc)
        logger.info(f'all sensors are registered')

    def go(self):
        '''
        vehicle percept, decide, plan, and go (set transform)
        TODO: add BEV
        '''
        v = 1
        old_tf =self.vehicle.get_transform()
        yaw = old_tf.rotation.yaw
        direction = np.deg2rad(90 - yaw)
        move = carla.Location(x=v * np.sin(direction), y=v * np.cos(direction), z=0)
        new_tf = carla.Transform(
            old_tf.location+move,
            old_tf.rotation
        )
        self.vehicle.set_transform(new_tf)

    def register_sensor(self, single_sensor_configuration):
        sensor_blueprint = self.world.get_blueprint_library().find(single_sensor_configuration[0])
        attr = single_sensor_configuration[2]
        for attr_key, attr_value in zip(attr.keys(), attr.values()):
            sensor_blueprint.set_attribute(str(attr_key), str(attr_value))
        camera_tf = carla.Transform(
            carla.Location(single_sensor_configuration[1][0],
                           single_sensor_configuration[1][1],
                           single_sensor_configuration[1][2]),
            carla.Rotation(single_sensor_configuration[1][3],
                           single_sensor_configuration[1][4],
                           single_sensor_configuration[1][5]))
        sensor = self.world.spawn_actor(sensor_blueprint, camera_tf, attach_to=self.vehicle,
                                        attachment_type=carla.AttachmentType.Rigid)
        data_queue = queue.Queue()
        sensor.listen(data_queue.put)
        self.sensor_type_list.append(sensor_blueprint)
        self.sensor_actors.append(sensor)
        self.sensor_queue_list.append(data_queue)
        if 'camera' in single_sensor_configuration[0]:
            image_size = (single_sensor_configuration[2]['image_size_x'],
                          single_sensor_configuration[2]['image_size_y'])
            if self.image_size is not None and self.image_size != image_size:
                logger.error('camera does not have same size')
            self.image_size = image_size

    def get_sensor_data(self):
        batch_data = [self.rawdata2np(q.get().raw_data, self.sensor_configuration[idx][0])
                      for idx, q in enumerate(self.sensor_queue_list)]
        return batch_data

    def rawdata2np(self, raw_data, sensor_bp_name):
        if 'camera' in sensor_bp_name:
            return np.frombuffer(raw_data, dtype=np.dtype('uint8')) \
                       .reshape((self.image_size[1], self.image_size[0], 4))[:, :, :3]

    def close(self):
        self.vehicle.destroy()
        for s in self.sensor_actors:
            s.stop()
            s.destroy()
        self.blueprint = None
        self.vehicle_tf = None
        self.vehicle = None
        self.world = None
        self.map = None

        self.image_size = None
        self.sensor_configuration = None
        self.sensor_queue_list = []
        self.sensor_actors = []
        self.sensor_type_list = []


class WorldHandler:
    def __init__(self):
        self.carla_client = None
        self.world = None
        self.map = None
        self.world_settings = None

        self.cv_window_name = None

        self.ego_agent = None

    def setup(self, runtime):
        # connect to carla
        logger.info(f"{runtime.carla_path} -carla-rpc-port={runtime.carla_port}")
        subprocess.Popen(f"{runtime.carla_path} -carla-rpc-port={runtime.carla_port}")
        time.sleep(5)
        self.carla_client = carla.Client(runtime.carla_ip_addr, runtime.carla_port)
        world_name = self.carla_client.get_available_maps()[6]
        time.sleep(2)
        logger.info('client is connected')
        self.world = self.carla_client.load_world(world_name)
        time.sleep(2)
        logger.info('world is loaded')
        self.map = self.world.get_map()

        # set synchrony mode
        self.world_settings = self.world.get_settings()
        self.world_settings.fixed_delta_seconds = 0.05
        self.world_settings.synchronous_mode = True
        self.world.apply_settings(self.world_settings)

        self.carla_client.reload_world(False)
        self.world.tick()
        self.traffic_manager =self.carla_client.get_trafficmanager(8000)
        self.traffic_manager.set_synchronous_mode(True)

        self.cv_window_name = 'carla_simulation'

        self.ego_agent = EgoAgent()
        self.ego_agent.setup(self.world, self.map,
                             runtime.carla_vehicle_bp_name,
                             runtime.carla_vehicle_transform,
                             runtime.sensor_configuration)
        self.ego_agent.spawn()
        time.sleep(2)

        h, w = (runtime.image_params['image_size_x'], runtime.image_params['image_size_y'])
        opencv_window = cv2.namedWindow(self.cv_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.cv_window_name, w, h)

        if runtime.carla_vehicle_autopilot:
            self.ego_agent.vehicle.set_autopilot(True)
        try:
            while True:
                self.world.tick()
                logger.info(f'tick at {time.time()}')
                spectator = self.world.get_spectator()
                transform = self.ego_agent.vehicle.get_transform()
                spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                        carla.Rotation(pitch=-90)))

                time.sleep(0.03)
                batch_image = self.ego_agent.get_sensor_data()
                image2show = self.batch_data2image_on_windows(batch_image)

                cv2.imshow(self.cv_window_name, image2show)
                cv2.waitKey(1)
                if not runtime.carla_vehicle_autopilot:
                    self.ego_agent.go()
        except KeyboardInterrupt:
            logger.info('stop')
        finally:
            self._close()

    def batch_data2image_on_windows(self, batch_data):
        if len(self.ego_agent.sensor_configuration) == 1:
            return batch_data[0]
        if len(self.ego_agent.sensor_configuration) == 6:
            d1 = np.hstack([batch_data[2], batch_data[0], batch_data[1]])
            d2 = np.hstack([batch_data[3], batch_data[5], batch_data[4]])
            data = np.vstack([d1, d2])
            return data

    def _close(self):
        cv2.destroyWindow(self.cv_window_name)
        self.ego_agent.close()
        self.world_settings = self.world.get_settings()
        self.world_settings.synchronous_mode = False  # Set a variable time-step
        self.world.apply_settings(self.world_settings)

        self.carla_client = None
        self.world = None
        self.map = None
        self.world_settings = None
        self.sensor_configuration = None
        self.sensor_queue_list = []
        self.sensor_actors = []
        self.sensor_type_list = []

        self.cv_window_name = None


if __name__ == "__main__":
    world_handler = WorldHandler()
    world_handler.setup(runtime)
