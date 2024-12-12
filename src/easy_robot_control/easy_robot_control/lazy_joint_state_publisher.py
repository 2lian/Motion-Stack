"""Overloading the joint_state_publisher package so it does not publish joint states that
are not actively published"""

import joint_state_publisher.joint_state_publisher as jsp_lib
import sensor_msgs.msg
from joint_state_publisher.joint_state_publisher import JointStatePublisher as Jsp


class dummy_pub:
    def publish(self, msg):
        return


class JointStatePublisher(Jsp):

    def __init__(self, description_file):
        super().__init__(description_file)
        self.active_joints = set()

        # replaces self.pub.publish by by a call that performs 
        # self.delete_inactive_from_msg before publishing
        self.pure_pub = self.pub
        self.pub = dummy_pub()
        self.pub.publish = lambda msg: self.pure_pub.publish(
            self.delete_inactive_from_msg(msg)
        )

    def source_cb(self, msg):
        # adds the message's names to the set of active joints
        self.active_joints = self.active_joints | set(msg.name)
        super().source_cb(msg)

    def delete_inactive_from_msg(
        self, msg: sensor_msgs.msg.JointState
    ) -> sensor_msgs.msg.JointState:
        """Deletes joints that are not part of self.active_joints from a message"""
        new = sensor_msgs.msg.JointState()
        new.header = msg.header
        indexes = [
            index for index, name in enumerate(msg.name) if name in self.active_joints
        ]
        new.name = [msg.name[i] for i in indexes]
        if msg.position:
            new.position = [msg.position[i] for i in indexes]
        if msg.velocity:
            new.velocity = [msg.velocity[i] for i in indexes]
        if msg.effort:
            new.effort = [msg.effort[i] for i in indexes]
        return new


def main():
    jsp_lib.JointStatePublisher = JointStatePublisher
    jsp_lib.main()


if __name__ == "__main__":
    main()
