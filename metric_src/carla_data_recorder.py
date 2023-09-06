import math
import json
import csv
import matplotlib.pyplot as plt
from srunner.metrics.examples.basic_metric import BasicMetric
import os


class Datarecorder(BasicMetric):
    """
    Metric class DistanceToLaneCenter
    """

    def _create_metric(self, town_map, log, criteria):
        """
        Implementation of the metric.
        """

        # Get ego vehicle id
        ego_id = log.get_actor_ids_with_type_id("vehicle.tesla.*")[0]

        if len(log.get_actor_ids_with_role_name("scenario")) > 0:
            other_id = log.get_actor_ids_with_role_name("scenario")[0]
        else:
            other_id = None
        

        dist_to_lane_list = []
        dist_to_car_list = []
        steer_angle_list = []
        throttle_list = []
        brake_list = []
        ego_speed_list = []
        frames_list = []


        # Get the frames the ego actor was alive and its transforms
        if other_id != None:
            start_ego, end_ego = log.get_actor_alive_frames(ego_id)
            start_adv, end_adv = log.get_actor_alive_frames(other_id)
            start = max(start_ego, start_adv)
            end = min(end_ego, end_adv)
        else:
            start, end = log.get_actor_alive_frames(ego_id)

        # Get the projected distance vector to the center of the lane
        for i in range(start, end + 1):

            ego_location = log.get_actor_transform(ego_id, i).location
            ego_waypoint = town_map.get_waypoint(ego_location)

            ego_speed = log.get_actor_velocity(ego_id, i)
            ego_speed.z = 0

            ego_control = log.get_vehicle_control(ego_id, i)
            ego_steer = ego_control.steer * 100
            ego_throttle = ego_control.throttle * 100
            ego_brake = ego_control.brake * 100

            # Get the distance vector and project it
            a = ego_location - ego_waypoint.transform.location      # Ego to waypoint vector
            b = ego_waypoint.transform.get_right_vector()           # Waypoint perpendicular vector
            b_norm = math.sqrt(b.x * b.x + b.y * b.y + b.z * b.z)

            ab_dot = a.x * b.x + a.y * b.y + a.z * b.z
            dist_v = ab_dot/(b_norm*b_norm)*b
            dist = math.sqrt(dist_v.x * dist_v.x + dist_v.y * dist_v.y + dist_v.z * dist_v.z)

            # Get the sign of the distance (left side is positive)
            # c = ego_waypoint.transform.get_forward_vector()         # Waypoint forward vector
            # ac_cross = c.x * a.y - c.y * a.x
            # if ac_cross < 0:
            #     dist *= -1

            if (ego_speed.length() <= 0 and ego_brake == 0 and ego_throttle == 0 and ego_steer == 0):
                continue

            if other_id != None:
                adv_location = log.get_actor_transform(other_id, i).location
                dist_v_2_v = ego_location - adv_location
                dist_between_v = math.sqrt(dist_v_2_v.x * dist_v_2_v.x + dist_v_2_v.y * dist_v_2_v.y + dist_v_2_v.z * dist_v_2_v.z)

                dist_to_car_list.append(dist_between_v)
            
            dist_to_lane_list.append(dist)
            steer_angle_list.append(ego_steer)
            throttle_list.append(ego_throttle)
            brake_list.append(ego_brake)
            ego_speed_list.append(ego_speed.length())
            frames_list.append(i)

        if other_id != None:
            data = zip(frames_list ,throttle_list ,brake_list ,steer_angle_list ,ego_speed_list ,dist_to_lane_list, dist_to_car_list)
        else:
            data = zip(frames_list ,throttle_list ,brake_list ,steer_angle_list ,ego_speed_list ,dist_to_lane_list)
        

        # plt.plot(frames_list, dist_to_lane_list)
        # plt.plot(frames_list, steer_angle_list)
        plt.plot(frames_list, dist_to_lane_list)
        plt.ylabel('Distance [m]')
        plt.xlabel('Frame number')
        plt.title('Distance between the ego vehicle and the lane center over time')
        plt.show()

        save_directory = '/home/shule/carla-cosim/experiment_results/'

        effect = 'None'
        scenriao = 'Car_follow'
        # scenriao = 'Straight'
        test_time = '01'
        congif = 'None'
        csv_file_name = scenriao+"_"+effect+"_"+congif+"_"+test_time+ '.csv'
        csv_file_path = os.path.join(save_directory + csv_file_name)

        with open(csv_file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            if other_id == None:
                writer.writerow(['frames','throttle','brake','steer','speed','lane_d'])
                writer.writerows(data)
            else:
                writer.writerow(['frames','throttle','brake','steer','speed','lane_d','car_d'])
                writer.writerows(data)
            
        print('save {} in {}'.format(csv_file_name,csv_file_path))
            



