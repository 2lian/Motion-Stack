from typing import Dict

URDFJointName = str
CommandJointName = str
SensorJointName = str
CommandTopicName = str

prefix_top = "top_"
mymotor = "/maxon/canopen_motor/base_link1_joint_velocity_controller/command"
remap2topic: Dict[URDFJointName, CommandTopicName] = {
    "leg3_joint5": mymotor,
}

prefix_com = "com_"
remap_com: Dict[URDFJointName, CommandJointName] = {
    "leg3_joint5": f"base_link1_joint",
}

prefix_sens = "sens_"
remap_sens: Dict[SensorJointName, URDFJointName] = {
    f"base_link1_joint": "leg3_joint5",
}
