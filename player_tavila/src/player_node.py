#!/usr/bin/python
import rospy
from std_msgs.msg import String


class Player:
    def __init__(self, player_name):
        self.player_name = player_name
        rospy.logwarn("I am {}".format(self.player_name))
        red_team = rospy.get_param('red_team')
        blue_team = rospy.get_param('blue_team')
        green_team = rospy.get_param('green_team')
        print("Teams available:\n{}\n{}\n{}".format(red_team[0], blue_team[0], green_team[0]))


def callback(msg):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print("received a message containing string {}".format(msg.data))


def main():
    name = 'tavila'
    rospy.init_node(name, anonymous=False)
    player = Player(name)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
