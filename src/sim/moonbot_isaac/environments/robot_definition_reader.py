import logging
import re
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Any, List
from xml.dom.minidom import Node as DomNode
from xml.dom.minidom import parseString

from ament_index_python.packages import get_package_share_directory

from environments.config import RobotConfig
from environments.utils import set_attr


def replace_package_urls_with_paths(input_string):
    # Define the regex pattern to match substrings starting with "package://" and ending with a quote mark
    pattern = r"package://([^/]+)"  # Capturing group to extract package name

    # Find all matches of the pattern in the input string
    matches = re.findall(pattern, input_string)

    # Iterate through matches and replace package URLs with file paths
    for package_name in matches:
        try:
            package_path = get_package_share_directory(package_name)
            package_url = "package://" + package_name
            input_string = input_string.replace(package_url, package_path)
        except Exception as e:
            logging.error(f"Error while replacing package URL with path: {e}")

    return input_string


@dataclass
class IsaacAttribute:
    path: str
    name: str
    value: Any


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
            set_attr(robot_path + attribute.path, attribute.name, attribute.value)

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

    # Collect extra data from isaac_sim tags
    def collect_extras(node, path=""):
        if node.nodeType == DomNode.ELEMENT_NODE:
            if node.tagName in ["link", "joint"] and node.hasAttribute("name"):
                path += "/" + node.getAttribute("name")
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
                break
            except Exception as e:
                logging.error(
                    f"Error while getting robot description: {e}. \n\nTrying again...\n\n"
                )
                time.sleep(1.0)
        self.urdf_doc = robot_description
        self.urdf_description, self.urdf_extras = process_robot_description(
            robot_description, self.robot_config.name
        )
        self.on_description_received(self.urdf_description)

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

        try:
            result = subprocess.run(
                ["xacro", self.path],
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
