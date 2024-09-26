from typing import Dict

URDFJointName = str
CommandJointName = str
SensorJointName = str
CommandTopicName = str

prefix_top = "top_"
remap2topic: Dict[URDFJointName, CommandTopicName] = {
    "leg3_joint5": f"{prefix_top}test",
}

prefix_com = "com_"
remap_com: Dict[URDFJointName, CommandJointName] = {
    "leg3_joint5": f"{prefix_com}test",
}

prefix_sens = "sens_"
remap_sens: Dict[SensorJointName, URDFJointName] = {
    f"{prefix_com}test": "leg3_joint5",
}
