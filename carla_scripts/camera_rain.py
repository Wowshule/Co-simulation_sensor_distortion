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


def get_noise(img, value=10):
    '''
    #生成噪声图像
    >>> 输入： img图像
        value= 大小控制雨滴的多少 
    >>> 返回图像大小的模糊噪声图像
    '''
 
    noise = np.random.uniform(0, 256, img.shape[0:2])
    # 控制噪声水平，取浮点数，只保留最大的一部分作为噪声
    v = value * 0.01
    noise[np.where(noise < (256 - v))] = 0
 
    # 噪声做初次模糊
    k = np.array([[0, 0.1, 0],
                  [0.1, 8, 0.1],
                  [0, 0.1, 0]])
 
    noise = cv2.filter2D(noise, -1, k)
 
    # 可以输出噪声看看
    '''cv2.imshow('img',noise)
    cv2.waitKey()
    cv2.destroyWindow('img')'''
    return noise

def rain_blur(noise, length=10, angle=0,w=1):
    '''
    将噪声加上运动模糊,模仿雨滴
    
    >>>输入
    noise：输入噪声图，shape = img.shape[0:2]
    length: 对角矩阵大小，表示雨滴的长度
    angle： 倾斜的角度，逆时针为正
    w:      雨滴大小
    
    >>>输出带模糊的噪声
    
    '''
    
    
    #这里由于对角阵自带45度的倾斜，逆时针为正，所以加了-45度的误差，保证开始为正
    trans = cv2.getRotationMatrix2D((length/2, length/2), angle-45, 1-length/100.0)  
    dig = np.diag(np.ones(length))   #生成对焦矩阵
    k = cv2.warpAffine(dig, trans, (length, length))  #生成模糊核
    k = cv2.GaussianBlur(k,(w,w),0)    #高斯模糊这个旋转后的对角核，使得雨有宽度
    
    #k = k / length                         #是否归一化
    
    blurred = cv2.filter2D(noise, -1, k)    #用刚刚得到的旋转后的核，进行滤波
    
    #转换到0-255区间
    cv2.normalize(blurred, blurred, 0, 255, cv2.NORM_MINMAX)
    blurred = np.array(blurred, dtype=np.uint8)
    '''
    cv2.imshow('img',blurred)
    cv2.waitKey()
    cv2.destroyWindow('img')'''
    
    return blurred


def alpha_rain(rain,img,beta = 0.8):
    
    #输入雨滴噪声和图像
    #beta = 0.8   #results weight
    #显示下雨效果
    
    #expand dimensin
    #将二维雨噪声扩张为三维单通道
    #并与图像合成在一起形成带有alpha通道的4通道图像
    rain = np.expand_dims(rain,2)
    rain_effect = np.concatenate((img,rain),axis=2)  #add alpha channel
 
    rain_result = img.copy()    #拷贝一个掩膜
    rain = np.array(rain,dtype=np.float32)     #数据类型变为浮点数，后面要叠加，防止数组越界要用32位
    rain_result[:,:,0]= rain_result[:,:,0] * (255-rain[:,:,0])/255.0 + beta*rain[:,:,0]
    rain_result[:,:,1] = rain_result[:,:,1] * (255-rain[:,:,0])/255 + beta*rain[:,:,0] 
    rain_result[:,:,2] = rain_result[:,:,2] * (255-rain[:,:,0])/255 + beta*rain[:,:,0]
    #对每个通道先保留雨滴噪声图对应的黑色（透明）部分，再叠加白色的雨滴噪声部分（有比例因子）
 
    return rain_result

IM_WIDTH = 640
IM_HEIIGHT = 480 


def process_img(image,camera_queue):
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIIGHT,IM_WIDTH,4))
    i3 = i2[:,:,:3]
    
    noise = get_noise(i3, 50)
    blur = rain_blur(noise,25,w=3)
    rain_image = alpha_rain(blur,i3)

    # cv2.imwrite(os.path.join('../outputs/output_synchronized', '%06d.png' % image.frame),i3)
    cv2.imwrite(os.path.join('./outputs/output_synchronized_rain', '%06d.png' % image.frame),rain_image)

    # cv2.imshow("Image",rain_image)
    # cv2.waitKey(25)

    camera_queue.put(image.frame)
    # cv2.destroyAllWindows()
    return None


def main():
    sensor_list =  []
    actor_list = []

    output_path = './outputs/output_synchronized_rain'
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
            if(n > 500):
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