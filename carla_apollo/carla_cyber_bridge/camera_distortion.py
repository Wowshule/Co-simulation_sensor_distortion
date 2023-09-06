from __future__ import absolute_import
from PIL import Image,ImageEnhance
import numpy as np
import cv2
from raindrop.dropgenerator import generateDrops, generateDrops_Mosaic
# from raindrop.config import cfg
import yaml
import threading
import os
import time


frame_id = 0
config_file = os.path.dirname(__file__) + "/config/sensor_distortion.yaml"
cfg = []
update_counter = 0
update_num = 100
lock = threading.Lock()


def rain_effect_on_window(carla_image_data_array):
    global frame_id
    global cfg

    image_array = cv2.cvtColor(carla_image_data_array, cv2.COLOR_BGR2RGB)
    rain_cfg = cfg['rain_on_window']

    if cfg['rain_on_window']['return_label'] == 1:
        distored_image_RGB, label_img = generateDrops(image_array, rain_cfg)
    else:
        distored_image_RGB = generateDrops(image_array, rain_cfg)

    distored_image = cv2.cvtColor(distored_image_RGB, cv2.COLOR_RGBA2BGRA)

    # debug print and img save
    if frame_id < 100 and cfg['rain_on_window']['debug_flag'] == 1:
        cv2.imwrite(os.path.join('./carla_cyber_bridge/ROLE/Output_image', '%06d.png' % frame_id),distored_image)
        print("save image {}".format(frame_id))
        frame_id += 1
    
    return distored_image




def fog_effect_on_window(carla_image_data_array):
    distored_image = carla_image_data_array
    return distored_image



def guas_effect(carla_image_data_array):
    global cfg
    distored_image = cv2.GaussianBlur(carla_image_data_array,(cfg['Gaussian_effect']['kernel'], cfg['Gaussian_effect']['kernel']), 0)
    return distored_image



def increase_brightness(carla_image_data_array):
    global cfg
    factor = cfg['increase_brightness']['brightness_factor']

    image_array = cv2.cvtColor(carla_image_data_array, cv2.COLOR_BGR2RGB)
    PIL_img = Image.fromarray(image_array)
      
    enhancer = ImageEnhance.Brightness(PIL_img)
    brightened_img = enhancer.enhance(factor)

    return cv2.cvtColor(np.asarray(brightened_img), cv2.COLOR_RGB2BGR)



def sunlight_effect(carla_image_data_array):
    global cfg
    sunlight_effect_cfg = cfg['sunshine_effect']
    
    # image_array = cv2.cvtColor(carla_image_data_array, cv2.COLOR_BGRA2RGBA)
    PIL_img = Image.fromarray(carla_image_data_array,'RGBA')
    width, height = PIL_img.size

    radius = min(width, height) // sunlight_effect_cfg['percent']
    factor = sunlight_effect_cfg['factor']
    x_offset = sunlight_effect_cfg['x_offset']
    y_offset = sunlight_effect_cfg['y_offset']
    position = ((width // 2 - x_offset), (height// 2 - y_offset))

    sun = Image.new("RGBA", PIL_img.size)
    d = np.array(sun)

    y, x = np.indices((height, width))
    dist = np.sqrt((x - position[0]) ** 2 + (y - position[1]) ** 2)

    mask = dist < radius
    d[..., :3][mask] = 255
    d[..., 3][mask] = ((1 - (dist[mask] / radius)**factor) * 255).astype(int)

    sun = Image.fromarray(d)
    # print(type(sun))
    # print(type(PIL_img))
    # time.sleep(10)
    distrotion_img = Image.alpha_composite(PIL_img, sun)

    # return cv2.cvtColor(np.asarray(distrotion_img), cv2.COLOR_RGB2BGR)
    return np.asarray(distrotion_img)





def rain_on_lens(carla_image_data_array):
    # global frame_id
    global cfg

    rain_on_lens_cfg = cfg['rain_on_lens']
    num_circles = rain_on_lens_cfg['Dropsnum']
    min_radius = rain_on_lens_cfg['minR']
    max_radius = rain_on_lens_cfg['maxR']
    blur_amount = rain_on_lens_cfg['blur_amount']

    img = cv2.cvtColor(carla_image_data_array, cv2.COLOR_BGR2RGB)  # Convert color style from BGR to RGB
    h, w, _ = img.shape
    
    # Create a black image to draw circles on
    mask = np.zeros((h, w), dtype=np.uint8)

    for _ in range(num_circles):
        # Randomly determine circle parameters
        center = (np.random.randint(0, w), np.random.randint(0, h))
        if min_radius != max_radius:
            radius = np.random.randint(min_radius, max_radius)
        else:
            radius = min_radius
        
        # Draw circle on the mask
        cv2.circle(mask, center, radius, (255), -1)

    # Adjust the blur amount to be an odd number if it is not
    blur_amount = blur_amount if blur_amount % 2 != 0 else blur_amount + 1

    # Blur the mask
    blurred_mask = cv2.GaussianBlur(mask, (blur_amount, blur_amount), 0)
    
    # Normalize the mask to keep it in range [0, 1]
    blurred_mask = blurred_mask.astype(float) / 255

    # Convert the mask to 3 channel
    blurred_mask = np.stack([blurred_mask]*3, axis=2)
    
    # Apply the mask to the image
    blurred_img = cv2.GaussianBlur(img, (blur_amount, blur_amount), 0)
    distored_image = (blurred_img * blurred_mask + img * (1 - blurred_mask)).astype(np.uint8)
    
    return cv2.cvtColor(distored_image, cv2.COLOR_RGB2BGR)




def read_and_update():
    global config_file
    global cfg
    global update_counter
    global update_num

    lock.acquire()
    if update_counter <= 0:
        
        cfg = yaml.safe_load(open(config_file))['sensor_distortion']
        update_counter = update_num
        
    else:
        update_counter -= 1

    lock.release()
    return cfg

    


def camera_distortion_effect(carla_image_data_array):
    choice = read_and_update()['distortion_mod']
    # print('Current cfg is {}'.format(cfg))
    # global cfg
    # print(cfg)
    if choice == 0:
        # rain_effect_on_window(carla_image_data_array)
        return carla_image_data_array
    
    elif choice == 1:
        return rain_effect_on_window(carla_image_data_array)
    
    elif choice == 2:
        # rain_effect_on_window(carla_image_data_array)
        return guas_effect(carla_image_data_array)

    elif choice == 3:
        # rain_effect_on_window(carla_image_data_array)
        return fog_effect_on_window(carla_image_data_array)
    
    elif choice == 4:
        # rain_effect_on_window(carla_image_data_array)
        return increase_brightness(carla_image_data_array)
    
    elif choice == 5:
        # rain_effect_on_window(carla_image_data_array)
        return sunlight_effect(carla_image_data_array)
    
    elif choice == 6:
        # rain_effect_on_window(carla_image_data_array)
        return rain_on_lens(carla_image_data_array)
    
    else:
        return carla_image_data_array