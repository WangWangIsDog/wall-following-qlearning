#! /usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import random
import rospy
import time
import math
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState

ALPHA = 0.2
GAMMA = 0.8
EPSILON = 0.9

actions = ['forward', 'right', 'left']

try:
    Q_table = np.load('q_table.npy')
    print("Found Q table")
except FileNotFoundError:
    table_size = (3, 3, 3, 3)
    Q_table = np.zeros(table_size + (len(actions),))

class QLearningTriton:
    def __init__(self):
        rospy.init_node('triton_q_learning', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.state = (1, 1, 1, 1)
        self.scan_data = None
        self.current_state = None
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.last_position = None
        self.stuck_counter = 0
        self.floating_counter = 0
        self.rate = rospy.Rate(10)
        self.reset_robot_pos()

        self.episode_rewards = []
        self.q_updates = []
    
    def laser_callback(self, data):
        self.scan_data = data

    def get_laser_scan_state(self):
        if self.scan_data is None:
            return (1, 1, 1, 1)
        else:
            front = min(min(self.scan_data.ranges[:10] + self.scan_data.ranges[-10:]), 3.0)
            left = min(min(self.scan_data.ranges[80:100]), 3.0)
            # left_front = min(min(self.scan_data.ranges[35:55]), 3.0)
            right = min(min(self.scan_data.ranges[260:280]), 3.0)
            right_front = min(min(self.scan_data.ranges[305:325]), 3.0)
            return self.discretize_state(front, left, right, right_front)

    def update_position(self,):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = get_state('triton', '')
            self.current_position = np.array([model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z])
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def discretize_state(self, front, left, right, right_front):
        front_state = 0
        left_state = 0
        right_state = 0
        right_front_state = 0

        if front > 1:
            front_state = 2
        elif front > 0.7:
            front_state = 1
        else:
            front_state = 0

        if left > 1:
            left_state = 2
        elif left > 0.5:
            left_state = 1
        else:
            left_state = 0

        if right > 1:
            right_state = 2
        elif right > 0.5:
            right_state = 1
        else:
            right_state = 0

        if right_front > 1.5:    # 1.5
            right_front_state = 2
        elif right_front > 0.7: # 0.7
            right_front_state = 1
        else:
            right_front_state = 0

        return (front_state, left_state, right_state, right_front_state)

    def choose_action_train(self, state, epsilon):
        if random.uniform(0, 1) < epsilon:
            return random.choice(range(len(actions)))
        else:
            return np.argmax(Q_table[state])

    def choose_action_test(self, state):
        return np.argmax(Q_table[state])

    def take_action(self, action):
        twist = Twist()
        if action == 0: # Forward
            twist.linear.x = 0.3
            twist.angular.z = 0
        elif action == 1: # Right
            twist.linear.x = 0.2
            twist.angular.z = -math.pi / 4
        elif action == 2: # Left
            twist.linear.x = 0.2
            twist.angular.z = math.pi / 4
        self.vel_pub.publish(twist)

    def reset_robot_pos(self):
        rospy.logwarn("Resetting robot position")
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            model_state = ModelState()
            model_state.model_name = 'triton'
            model_state.pose.position.x = random.uniform(-3.5, 3.5)
            model_state.pose.position.y = random.uniform(-3.5, 3.5)
            model_state.pose.position.z = 0.1
            model_state.pose.orientation.z = random.uniform(-1.0, 1.0)
            model_state.pose.orientation.w = random.uniform(-1.0, 1.0)
            set_state(model_state)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        self.last_position = None
        self.stuck_counter = 0
        self.floating_counter = 0

    def get_reward(self, state):

       # if state[0] == 0: 
        #    return -10
        #elif state[1] == 0 or state[2] == 0:
         #   return -5
        #elif state[2] == 1 and state[3] == 1:
         #   return 5
        #return 1

        if state[0] == 0:
            return -10
        elif state[2] == 0 or state[1] == 0:
            return -5
        elif state[2] == 1 and state[3] == 1:
            return 5
        elif state[2] == 2:
            return -2
        return 1

    def check_if_stuck(self):
        self.update_position()
        if self.last_position is not None and np.allclose(self.last_position, self.current_position[:2], atol=0.01):
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        self.last_position = self.current_position[:2]
        return self.stuck_counter > 50

    def check_if_floating(self):
        self.update_position()
        if abs(self.current_position[2]) > 0.1:
            self.floating_counter += 1
        else:
            self.floating_counter = 0
        return self.floating_counter > 5

    def train(self, episodes):
        self.update_position()
        for episode in range(episodes):
            print(f"Episode: {episode + 1}")
            global EPSILON
            total_reward = 0
            q_updates_count = 0
            EPSILON = max(0.1, EPSILON * 0.985)
            self.current_state = self.get_laser_scan_state()
            start_time = time.time()
            while time.time() - start_time < 30:
                 action = self.choose_action_train(self.current_state, EPSILON)
                 self.take_action(action)
                 reward = self.get_reward(self.current_state)
                 total_reward += reward
                 next_state = self.get_laser_scan_state()
                 if next_state != self.current_state:
                     print(f"current_state: {self.current_state}")
                     print(f"action: {actions[action]}")
                     print(f"next_state: {next_state}")
                     print("==============================")
            
                 best_next_action = np.max(Q_table[next_state])
                 Q_table[self.current_state + (action,)] += ALPHA * (reward + GAMMA * best_next_action - Q_table[self.current_state + (action,)])
                 
                 q_update_value = ALPHA * (reward + GAMMA * best_next_action - Q_table[self.current_state + (action,)])
                 if abs(q_update_value) > 1e-5:
                     q_updates_count += 1

                 self.current_state = next_state

                 if self.check_if_stuck() or self.check_if_floating():
                     rospy.logwarn("Robot is stuck or floating!")
                     self.reset_robot_pos()

                 self.rate.sleep()

            self.episode_rewards.append(total_reward)
            self.q_updates.append(q_updates_count)

        np.save('q_table.npy', Q_table)
        self.plot_learning_statistics()

    def test(self, duration=120):
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.current_state is not None: 
                 action = self.choose_action_test(self.current_state)
                 self.take_action(action)
            self.current_state = self.get_laser_scan_state() 
            self.update_position()

            if self.check_if_stuck() or self.check_if_floating():
                rospy.logwarn("Robot is stuck or floating! Resetting position..")
                self.reset_robot_pos()
            self.rate.sleep()

    def plot_learning_statistics(self):
        episodes = np.arange(len(self.episode_rewards))
        reward_smooth = np.convolve(self.episode_rewards, np.ones(10)/10, mode='valid')
        plt.figure(figsize=(12, 5))

        plt.subplot(1, 2, 1)
        plt.plot(episodes[:len(reward_smooth)], reward_smooth, label='Smoothed Reward', color='blue')
        plt.xlabel('Episodes')
        plt.ylabel('Accumulated Reward')
        plt.title('Accumulated Reward vs Episodes')
        plt.legend()

        plt.subplot(1, 2, 2)
        plt.plot(episodes, self.q_updates, label='Q-table Updates', color='red')
        plt.xlabel('Episodes')
        plt.ylabel('Number of Q-table Updates')
        plt.title('Rate of Q Learning Convergence')
        plt.legend()

        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'train'
    agent = QLearningTriton()
    if mode == 'train':
        agent.train(500)
    elif mode == 'test':
        agent.test()
    
