import rospy
import time
from robot import robot

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

import numpy as np
import math

path_pt_count = 200

def yaw_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z # in radians

def odom_callback(msg):
    robot_pose = msg
    robot_obj.set_center(robot_pose.pose.pose.position.x, robot_pose.pose.pose.position.y)
    
    robot_yaw = yaw_from_quaternion(robot_pose.pose.pose.orientation.x,
                                    robot_pose.pose.pose.orientation.y,
                                    robot_pose.pose.pose.orientation.z,
                                    robot_pose.pose.pose.orientation.w)
    #print(robot_yaw)
    robot_obj.alpha = robot_yaw

    robot_obj.set_odom_speed(robot_pose.twist.twist.linear.x, robot_pose.twist.twist.angular.z)


if __name__ == '__main__':
    rospy.init_node('kurs_node')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    robot_obj = robot(0.5)
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    cmd = Twist()

    robot_obj.set_tgt_point(0, 0.01)
    while not robot_obj.is_arrive():
        robot_obj.calc_tgt_speed()
        cmd.linear.x, cmd.angular.z = robot_obj.getSpeed()
        cmd_pub.publish(cmd)
    robot_obj.stop()
    cmd.linear.x, cmd.angular.z = robot_obj.getSpeed()
    cmd_pub.publish(cmd)

    target_path = np.zeros((path_pt_count + 1, 2))
    i = 2*math.pi/(path_pt_count - 1)
    for t in range(path_pt_count + 1):
        target_path[t, 0] = (0.5+math.cos(t*i))*math.cos(t*i) * 3
        target_path[t, 1] = (0.5+math.cos(t*i))*math.sin(t*i) * 3

    
    robot_obj.set_zero_offset(target_path[0,0],  target_path[0,1])
    #print(str(target_path[0,0]) +  str(target_path[0,1]))

    #fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    fig, ax1 = plt.subplots()
        # ax1.set_title("pose X")
        # ax1.xlabel("time, c")  # ось абсцисс
        # ax1.ylabel("x, m")  # ось ординат
    ax1.grid()  # включение отображение сетки
    #ax1.plot(target_path[:, 0], target_path[:, 1])  # построение графика
    ax1.scatter(target_path[:, 0], target_path[:, 1])  # построение графика

    robot_obj.begin_odom_loging()
    for i in range(len(target_path)):
    #for i in range(10):
        robot_obj.set_tgt_point(target_path[i, 0], target_path[i, 1])
        while not robot_obj.is_arrive():
            robot_obj.calc_tgt_speed()
            cmd.linear.x, cmd.angular.z = robot_obj.getSpeed()
            cmd_pub.publish(cmd)
    robot_obj.stop()
    robot_obj.end_odom_loging()
    cmd.linear.x, cmd.angular.z = robot_obj.getSpeed()
    cmd_pub.publish(cmd)
    #print(robot_obj.odom_log)

    robot_odom_log = np.asarray(robot_obj.odom_log)
    odom_len = np.shape(robot_odom_log)[0]
    #ax1.plot(robot_odom_log[:, 0], robot_odom_log[:, 1], linestyle=':', color='red', linewidth=2)
    ax1.plot(robot_odom_log[:, 0], robot_odom_log[:, 1], color='red', linewidth=2)

    ideal_path = np.zeros((300, 2))
    obs_error = np.zeros(300) 
    i = 2*math.pi/(300 - 1)
    for t in range(300):
        ideal_path[t, 0] = (0.5+math.cos(t*i))*math.cos(t*i) * 3
        ideal_path[t, 1] = (0.5+math.cos(t*i))*math.sin(t*i) * 3 

    for i in range(300):
        minDist = 10000

        for j in range(odom_len):
            dist = np.linalg.norm(robot_odom_log[j,:] - ideal_path[i, :])
            if dist < minDist:
                minDist = dist
        
        obs_error[i] = minDist
    
    print("mean err = " + str(np.mean(obs_error)))
    print("median err = " + str(np.median(obs_error)))


    fig, ax2 = plt.subplots()
    ax2.grid()
    ax2.set_title("absolute Err")
    ax2.set_xlabel("time, tick")  # ось абсцисс
    ax2.set_ylabel("err, m")  # ось ординат
    ax2.plot(range(300), obs_error, linewidth=2)

    tgt_vel_log, odom_vel_log = robot_obj.vel_log
    tgt_vel_log = np.asarray(tgt_vel_log)
    odom_vel_log = np.asarray(odom_vel_log)

    fig, (ax3, ax4) = plt.subplots(2 ,1)
    ax3.grid()
    ax3.set_title("lin speed")
    ax3.set_xlabel("time, tick")  # ось абсцисс
    ax3.set_ylabel("speed, m/s")  # ось ординат
    ax3.plot(range(odom_vel_log.shape[0]), tgt_vel_log[:, 0])
    ax3.plot(range(odom_vel_log.shape[0]), odom_vel_log[:, 0], linestyle='--', color='red')

    ax4.grid()
    ax4.set_title("ang speed")
    ax4.set_xlabel("time, tick")  # ось абсцисс
    ax4.set_ylabel("speed, rad/s")  # ось ординат
    ax4.plot(range(odom_vel_log.shape[0]), tgt_vel_log[:, 1])
    ax4.plot(range(odom_vel_log.shape[0]), odom_vel_log[:, 1], linestyle='--', color='red')




    plt.show()
    

        
        
