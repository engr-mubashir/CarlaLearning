import glob
import os
import sys
import random
import time
import numpy as np
import cv2

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# Constants
IP = "127.0.0.1"
PORT = 2000
FOV = 90
IMAGE_WIDTH = 500
IMAGE_HEIGHT = 350
CAM_DISPLAY = True


class AutonomousVehicle:
    actors = []
    collisions = []
    distance = 0.0

    def __init__(self):
        # Connect and get the world
        self.client = carla.Client(IP, PORT)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.camera_transform = carla.Transform(carla.Location(x=1.0, z=1.5))

    def spawn_vehicle(self):
        self.vehicle_bp = self.blueprint_library.filter("vehicle.audi.a2")[0]
        self.vehicle_transform = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(self.vehicle_bp, self.vehicle_transform)
        # Save initial location to calculate distance
        self.initial_loc = self.vehicle.get_location()
        self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
        self.actors.append(self.vehicle)

    def spawn_camera_rgb(self):
        self.camera_bp = self.blueprint_library.find('sensor.camera.rgb')
        self.camera_bp.set_attribute("image_size_x", f"{IMAGE_WIDTH}")
        self.camera_bp.set_attribute("image_size_y", f"{IMAGE_HEIGHT}")
        self.camera_bp.set_attribute("fov", f"{FOV}")
        self.camera = self.world.spawn_actor(self.camera_bp, self.camera_transform, attach_to=self.vehicle)
        self.actors.append(self.camera)
        self.camera.listen(lambda data: self.process_image(data))
        time.sleep(1)

    def spawn_collision_sensor(self):
        self.collision_bp = self.blueprint_library.find("sensor.other.collision")
        self.collision_sensor = self.world.spawn_actor(self.collision_bp, self.camera_transform, attach_to=self.vehicle)
        self.actors.append(self.collision_sensor)
        self.collision_sensor.listen(lambda event: self.handle_collision(event))

    def calculate_distance(self):
        self.distance = carla.Location.distance(self.vehicle.get_location(), self.initial_loc)

    def handle_collision(self, event):
        # Maintain collision history
        self.collisions.append(event)
        # For time being stop the vehicle
        self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
        time.sleep(1)

    def process_image(self, image):
        # Save images to disk
        image.save_to_disk('output/%06d.png' % image.frame_number)

        # Process images
        raw = np.array(image.raw_data)
        array = raw.reshape((IMAGE_HEIGHT, IMAGE_WIDTH, 4))
        self.final = array[:, :, :3]
        if CAM_DISPLAY:
            cv2.imshow("", self.final)
            cv2.waitKey(1)


def main():
    # Instantiate object
    auto_obj = AutonomousVehicle()
    # Spawn vehicle
    auto_obj.spawn_vehicle()
    # Spawn front camera
    auto_obj.spawn_camera_rgb()
    # Spawn collision sensor
    auto_obj.spawn_collision_sensor()

    while True:
        print(auto_obj.distance)
        auto_obj.calculate_distance()

        if auto_obj.distance > 100.0:
            if auto_obj.camera.is_listening:
                auto_obj.camera.stop()
            print("Vehicle is still driving but image processing and storage is stopped!")


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
