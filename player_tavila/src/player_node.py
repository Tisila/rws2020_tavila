#!/usr/bin/python
import math
import random
import numpy
import rospy
import tf
from geometry_msgs.msg import Transform, Quaternion
from rws2020_msgs.msg import MakeAPlay
import sys
#sys.path.append('../../../rws2020_moliveira/rws2020_lib/src')
#from rws2020_lib.utils import movePlayer, randomizePlayerPose, getDistanceAndAngleToTarget
#from std_msgs.msg import String


class Player:
    def __init__(self, player_name):
        self.player_name = player_name
        self.map_size = 8
        self.max_angle = math.pi / 30
        # Attributes initiated after...
        self.max_vel = 5
        self.transform = Transform()

        red_team = rospy.get_param('red_team')
        blue_team = rospy.get_param('blue_team')
        green_team = rospy.get_param('green_team')
        # print("Teams available:\nred: {}\nblue: {}\ngreen: {}".format(red_team, blue_team, green_team))

        if self.player_name in red_team:
            self.my_team, self.prey_team, self.hunter_team = 'red', 'green', 'blue'
            self.my_players, self.preys, self.hunters = red_team, green_team, blue_team
        elif self.player_name in green_team:
            self.my_team, self.prey_team, self.hunter_team = 'green', 'blue', 'red'
            self.my_players, self.preys, self.hunters = green_team, blue_team, red_team
        elif self.player_name in blue_team:
            self.my_team, self.prey_team, self.hunter_team = 'blue', 'red', 'green'
            self.my_players, self.preys, self.hunters = blue_team, red_team, green_team
        else:
            rospy.logerr("My name ({}) is not in any team, I want to play!".format(self.player_name))
            exit(0)

        rospy.loginfo("Hi! I am {} from the {} team, let's have some fun!".format(self.player_name, self.my_team))

        # rospy.logwarn("I am {} and I am on the {} team. {} players are going to die"
        #              .format(self.player_name, self.my_team, self.prey_team))
        # rospy.loginfo("I am afraid of them {}".format(self.hunters))

        rospy.Subscriber("make_a_play", MakeAPlay, self.callback_make_a_play)
        self.br = tf.TransformBroadcaster()
        self.transform = Transform()
        self.transform.translation.x = random.uniform(-self.map_size/2, self.map_size/2)
        self.transform.translation.y = random.uniform(-self.map_size/2, self.map_size/2)

    def callback_make_a_play(self, msg):
        # print("[MakeAPlay] the speed of cat is {}".format(msg.cat))
        self.max_vel = msg.cat
        # Make a play decision making
        velocity = self.max_vel

        max_vel, max_angle = msg.turtle, math.pi / 30

        if msg.red_alive:  # PURSUIT MODE: Follow any blue player (only if there is at least one blue alive)
            target = msg.red_alive[0]  # select the first alive blue player (I am hunting blue)
            distance, angle = self.get_distance_and_angle_to_target(self.listener, self.player_name, target)
            if angle is None:
                angle = 0
            vel = max_vel  # full throttle
        else:  # what else to do? Lets just move towards the center
            target = 'world'
            distance, angle = self.get_distance_and_angle_to_target(self.listener, self.player_name, target)
            vel = max_vel  # full throttle

        # Actually move the player
        self.move_player(self.br, self.player_name, self.transform, vel, angle, velocity)

    def get_distance_and_angle_to_target(self, tf_listener, my_name, target_name,
                                    time=rospy.Time(0), max_time_to_wait=1.0):
        try:
            tf_listener.waitForTransform(my_name, target_name, time, rospy.Duration(max_time_to_wait))
            (trans, rot) = tf_listener.lookupTransform(my_name, target_name, time)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
            rospy.logwarn(my_name + ': Could not get transform from ' + my_name + ' to ' + target_name)
            return None, None

        # compute distance and angle
        x, y = trans[0], trans[1]
        distance = math.sqrt(x ** 2 + y ** 2)
        angle = math.atan2(y, x)
        return distance, angle

    def randomize_player_pose(self, transform, arena_radius=8):
        """
        Randomizes the initial pose of a player. Based on the code by MGomes.
        :param transform: a geometry_msgs.msg.Transform() which will have the values of x,y and yaw randomized.
        :param arena_radius: the radius of the arena inside which the player can be positioned.
        """
        initial_r = arena_radius * random.random()
        initial_theta = 2 * math.pi * random.random()
        initial_x = initial_r * math.cos(initial_theta)
        initial_y = initial_r * math.sin(initial_theta)
        initial_rotation = 2 * math.pi * random.random()
        transform.translation.x = initial_x
        transform.translation.y = initial_y
        q = tf.transformations.quaternion_from_euler(0, 0, initial_rotation)
        transform.rotation = Quaternion(q[0], q[1], q[2], q[3])

    def move_player(self, tf_broadcaster, player_name, transform_now, vel, angle, max_vel):
        """
        Moves a player given its currrent pose, a velocity, and angle, and a maximum velocity
        :param tf_broadcaster: Used to publish the new pose of the player
        :param player_name:  string with the name of the player (must coincide with the name of the tf frame_id)
        :param transform_now: a geometry_msgs.msg.Transform() containing the current pose. This variable is updated with
                              the new player pose
        :param vel: velocity of displacement to take in x axis
        :param angle: angle to turn, limited by max_angle (pi/30)
        :param max_vel: maximum velocity or displacement based on the selected animal
        """
        max_angle = math.pi / 30

        if angle > max_angle:
            angle = max_angle
        elif angle < -max_angle:
            angle = -max_angle

        if vel > max_vel:
            vel = max_vel

        T1 = transform_now

        T2 = Transform()
        T2.rotation = tf.transformations.quaternion_from_euler(0, 0, angle)
        T2.translation.x = vel
        matrix_trans = tf.transformations.translation_matrix((T2.translation.x,
                                                              T2.translation.y,
                                                              T2.translation.z))

        matrix_rot = tf.transformations.quaternion_matrix((T2.rotation[0],
                                                           T2.rotation[1],
                                                           T2.rotation[2],
                                                           T2.rotation[3]))
        matrixT2 = np.matmul(matrix_trans, matrix_rot)

        matrix_trans = tf.transformations.translation_matrix((T1.translation.x,
                                                              T1.translation.y,
                                                              T1.translation.z))

        matrix_rot = tf.transformations.quaternion_matrix((T1.rotation.x,
                                                           T1.rotation.y,
                                                           T1.rotation.z,
                                                           T1.rotation.w))
        matrixT1 = np.matmul(matrix_trans, matrix_rot)

        matrix_new_transform = np.matmul(matrixT1, matrixT2)

        quat = tf.transformations.quaternion_from_matrix(matrix_new_transform)
        trans = tf.transformations.translation_from_matrix(matrix_new_transform)

        T1.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        T1.translation.x = trans[0]
        T1.translation.y = trans[1]
        T1.translation.z = trans[2]

        tf_broadcaster.sendTransform(trans, quat, rospy.Time.now(), player_name, "world")


def callback(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print("received a message containing string {}".format(msg.data))


def main():
    name = 'tavila'
    rospy.init_node(name, anonymous=False)
    player = Player(name)
    # rospy.Subscriber("chatter", String, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
