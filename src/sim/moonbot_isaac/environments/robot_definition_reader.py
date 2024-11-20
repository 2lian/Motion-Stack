import re
import threading

from ament_index_python.packages import get_package_share_directory
import subprocess
import logging
import time


def package_path_to_system_path(package_name, relative_path=""):
    package_share_path = get_package_share_directory(package_name)
    return package_share_path


def replace_package_urls_with_paths(input_string):
    # Define the regex pattern to match substrings starting with "package://" and ending with a quote mark
    pattern = r"package://([^/]+)"  # Capturing group to extract package name

    # Find all matches of the pattern in the input string
    matches = re.findall(pattern, input_string)

    # Iterate through matches and replace package URLs with file paths
    for package_name in matches:
        try:
            package_path = package_path_to_system_path(package_name)
            package_url = "package://" + package_name
            input_string = input_string.replace(package_url, package_path)
        except Exception as e:
            logging.error(f"Error while replacing package URL with path: {e}")  

    return input_string

class RobotDefinitionReader:
    def __init__(
        self,
    ):
        self.topic_name = None
        self.node = None
        self.description_received_fn = None
        self.urdf_doc = ""
        self.urdf_abs = ""

    def on_description_received(self, _):
        if self.description_received_fn:
            self.description_received_fn(self.urdf_abs)

    def handle_description_received(self, msg):
        self.urdf_doc = msg.data
        self.urdf_abs = replace_package_urls_with_paths(self.urdf_doc)
        self.on_description_received(self.urdf_abs)

    def get_robot_description(self):
        # Run the command and capture the output
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'echo', self.topic_name, '--field', 'data', '--once', '--full-length'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=True,
            )
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Error while running the command: {e.stderr}")
        
        description = result.stdout
        # remove the last `---` line
        description = description[:description.rfind("---")].strip()
        
        # Return the output as a string
        return description
    
    def service_call(self):
        while True:
            try:
                robot_description = self.get_robot_description()
                break
            except Exception as e:
                logging.error(f"Error while getting robot description: {e}. \n\nTrying again...\n\n")
                time.sleep(1.0)
        self.urdf_doc = robot_description
        self.urdf_abs = replace_package_urls_with_paths(self.urdf_doc)
        self.on_description_received(self.urdf_abs)


    def start_get_robot_description(self, topic_name):
        self.topic_name = topic_name

        thread = threading.Thread(target=self.service_call)
        thread.start()