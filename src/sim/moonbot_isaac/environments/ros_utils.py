import logging
import re
from ament_index_python.packages import get_package_share_directory


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