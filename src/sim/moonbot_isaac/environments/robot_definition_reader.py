import logging
import re
import ast
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Any, List
from xml.dom.minidom import Node as DomNode
from xml.dom.minidom import parseString

from environments.config import RobotConfig
from environments.isaac_utils import set_attr_cmd
from environments.ros_utils import replace_package_urls_with_paths

@dataclass
class IsaacAttribute:
    path: str
    name: str
    value: Any

    def parsed_value(self):
        """Convert XML string values to appropriate types using ast.literal_eval"""
        if isinstance(self.value, str):
            try:
                return ast.literal_eval(self.value)
            except (ValueError, SyntaxError):
                # Keep as string if can't convert
                return self.value
        return self.value


@dataclass
class JointInfo:
    name: str
    joint_type: str
    parent_link: str
    child_link: str


@dataclass
class URDFExtras:
    attributes: List[IsaacAttribute] = field(default_factory=list)
    joint_infos: List[JointInfo] = field(default_factory=list)

    def apply_to_robot_prim(self, robot_path: str):
        # remove trailing / from robot_path
        robot_path = robot_path.rstrip("/")

        for attribute in self.attributes:
            path = robot_path + attribute.path
            value = attribute.parsed_value()
            try:
                set_attr_cmd(path, attribute.name, value)
                logging.info(f"Set {attribute.name} of {path} to {value}")
            except Exception as e:
                raise RuntimeError(
                    f"Error while setting attribute {attribute.name} of {path} to {value}: {e}"
                )

    def get_joint_info(self, joint_name):
        for joint_info in self.joint_infos:
            if joint_info.name == joint_name:
                return joint_info
        return None


def process_robot_description(urdf, robot_name="robot"):
    # Replace package URIs with file paths
    urdf = replace_package_urls_with_paths(urdf)
    urdf_extras = URDFExtras()

    doc = parseString(urdf)

    # Change the robot name so all robots will have the same USD path in Isaac
    robot_element = doc.getElementsByTagName("robot")[0]
    robot_element.setAttribute("name", robot_name)

    # Remove comments from the URDF description
    def remove_comments(node):
        if node.nodeType == DomNode.COMMENT_NODE:
            node.parentNode.removeChild(node)
        else:
            for child in node.childNodes:
                remove_comments(child)

    remove_comments(doc)

    # Collect extra data from isaac_sim tags recursively
    def collect_extras(node, path=""):
        if node.nodeType == DomNode.ELEMENT_NODE:
            if node.tagName == "link" and node.hasAttribute("name"):
                path += f"/{node.getAttribute('name')}"
            # Joints will be added under `joints` scope
            elif node.tagName == "joint" and node.hasAttribute("name"):
                path += (
                    f"/joints/{node.getAttribute('name')}"
                )
            elif node.tagName == "collision":
                path += "/collisions"
            if node.tagName == "isaac_sim":
                for child in node.childNodes:
                    if child.nodeType == DomNode.ELEMENT_NODE:
                        if child.tagName == "attribute":
                            name = child.getAttribute("name")
                            value = child.getAttribute("value")
                            urdf_extras.attributes.append(
                                IsaacAttribute(path=path, name=name, value=value)
                            )

        for child in node.childNodes:
            collect_extras(child, path)

    collect_extras(doc)

    # Collect joint information
    def collect_joint_info(node):
        if node.nodeType == DomNode.ELEMENT_NODE:
            try:
                if node.tagName == "joint" and node.hasAttribute("name"):
                    joint_name = node.getAttribute("name")
                    joint_type = node.getAttribute("type")
                    parent_link = node.getElementsByTagName("parent")[0]
                    child_link = node.getElementsByTagName("child")[0]
                    if parent_link and child_link:
                        urdf_extras.joint_infos.append(
                            JointInfo(
                                joint_name,
                                joint_type,
                                parent_link.getAttribute("link"),
                                child_link.getAttribute("link"),
                            )
                        )
            except Exception as e:
                logging.error(
                    f"Error while collecting joint info from URDF: {e}\n{node.toxml()}"
                )

        for child in node.childNodes:
            collect_joint_info(child)

    collect_joint_info(doc)

    return doc.toxml(), urdf_extras


class RobotDefinitionReader:
    def __init__(
        self,
        robot_config: RobotConfig,
    ):
        self.robot_config = robot_config
        self.node = None
        self.description_received_fn = None
        self.urdf_doc = ""
        self.urdf_description = ""
        self.urdf_extras = None

    def on_description_received(self, _):
        if self.description_received_fn:
            self.description_received_fn(self.urdf_description)

    def handle_description_received(self, msg):
        self.urdf_doc = msg.data
        self.urdf_description = replace_package_urls_with_paths(self.urdf_doc)
        self.on_description_received(self.urdf_description)

    def get_robot_description(self):
        # Run the command and capture the output
        try:
            result = subprocess.run(
                [
                    "ros2",
                    "topic",
                    "echo",
                    self.robot_config.robot_description_topic,
                    "--field",
                    "data",
                    "--once",
                    "--full-length",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
            )
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Error while running the command: {e.stderr}")

        description = result.stdout
        # remove the last `---` line
        description = description[: description.rfind("---")].strip()

        # Return the output as a string
        return description

    def service_call(self):
        while True:
            try:
                robot_description = self.get_robot_description()

                self.urdf_doc = robot_description
                self.urdf_description, self.urdf_extras = process_robot_description(
                    robot_description, self.robot_config.name
                )
                self.on_description_received(self.urdf_description)
                
                break
            except Exception as e:
                logging.error(
                    f"Error while getting robot description: {e}. \n\nTrying again...\n\n"
                )
                time.sleep(1.0)

    def start_get_robot_description(self):
        thread = threading.Thread(target=self.service_call)
        thread.start()


class XacroReader:
    """
    Read a xacro file and convert it to URDF
    """

    def __init__(
        self,
        robot_config: RobotConfig,
    ):
        self.path = replace_package_urls_with_paths(robot_config.xacro_path)
        xacro_command = ["xacro", self.path]

        if robot_config.xacro_params:
            for key, value in robot_config.xacro_params.items():
                xacro_command.append(f"{key}:={value}")

        try:
            result = subprocess.run(
                xacro_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
            )
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Error while running the command: {e.stderr}")

        self.urdf_description, self.urdf_extras = process_robot_description(
            result.stdout, robot_config.name
        )
