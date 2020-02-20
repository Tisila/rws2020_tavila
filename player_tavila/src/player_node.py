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
        print("Teams available:\nred: {}\nblue: {}\ngreen: {}".format(red_team[0], blue_team[0], green_team[0]))

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

        rospy.logwarn("I am {} and I am on the {} team. {} players are going to die".format(self.player_name, self.my_team, self.prey_team))
        rospy.loginfo("I am afraid of them {}".format(self.hunters))


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
