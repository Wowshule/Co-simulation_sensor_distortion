import os
import sys
import time
import random
import time
import numpy as np
import cv2	
import threading
import carla
from queue import Queue
from queue import Empty
 
IM_WIDTH = 2048
IM_HEIIGHT = 1024 


def process_img(image,camera_queue):
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIIGHT,IM_WIDTH,4))
    i3 = i2[:,:,:3]
    

    blurred_image  = cv2.GaussianBlur(i3, (15, 15), 0)

    cv2.imwrite(os.path.join('./outputs/output_synchronized', '%06d.png' % image.frame),i3)
    cv2.imwrite(os.path.join('./outputs/output_synchronized_gauss', '%06d.png' % image.frame),blurred_image)

    # cv2.imshow("Image",blurred_image)
    # cv2.waitKey(25)

    camera_queue.put(image.frame)
    # cv2.destroyAllWindows()
    return None

# def timer(seconds):
#     print("定时器已启动！")
#     threading.Timer(seconds, timeout).start()

# def timeout():
# 	start_flag = 0
#     print("定时器时间到！")

def main():
    sensor_list =  []
    actor_list = []

    output_path = './outputs/output_synchronized'
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    
    output_path = './outputs/output_synchronized_gauss'
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    try:
        #create client
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        #save original_settings
        world =  client.get_world()
        original_settings = world.get_settings()

        # set to sync mode
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.05
        settings.synchronous_mode = True
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        # data queue
        camera_queue = Queue()

        #get blueprint libarary
        blueprint_library = world.get_blueprint_library()
        #Choose a vehicle blueprint which name is model3 and select the first one
        bp = blueprint_library.filter("model3")[0]
        print(bp)
        #Returns a list of recommended spawning points and random choice one from it
        spawn_point = world.get_map().get_spawn_points()[1]
        #spawn vehicle to the world by spawn_actor method
        vehicle = world.spawn_actor(bp,spawn_point)
        #control the vehicle
        vehicle.set_autopilot(enabled=True)

        #add vehicle to the actor list
        actor_list.append(vehicle)

        # add camera
        cam_bp = blueprint_library.find("sensor.camera.rgb")
        #set the attribute of camera
        cam_bp.set_attribute("image_size_x",f"{IM_WIDTH}")
        cam_bp.set_attribute("image_size_y",f"{IM_HEIIGHT}")
        cam_bp.set_attribute("fov","110")
        #add camera sensor to the vehicle
        spawn_point = carla.Transform(carla.Location(x=2.5,z=0.7))
        sensor = world.spawn_actor(cam_bp,spawn_point,attach_to=vehicle)

        actor_list.append(sensor)
        sensor_list.append(sensor)

        sensor.listen(lambda image: process_img(image, camera_queue))
        spectator = world.get_spectator()

        n = 0
        while(True):
            world.tick()
            loc = vehicle.get_transform().location
            spectator.set_transform(carla.Transform(carla.Location(x=loc.x,y=loc.y,z=35),carla.Rotation(yaw=0,pitch=-90,roll=0)))
            if(n > 100):
                break
            n = n+1

            # 使用queue.get()函数，在数据处理完成之前，阻止程序的推进
            try:
                for i in range(0, len(sensor_list)):
                    s_frame = camera_queue.get(True, 1.0)
                    print("    Frame: %d " % (s_frame))
            except Empty:
                print("   Some of the sensor information is missed")
    finally:
        world.apply_settings(original_settings)
        for actor in actor_list:
            actor.destroy()
        print("All cleaned up!")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')