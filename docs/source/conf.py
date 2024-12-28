# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import os
import sys

# Add the src directory to the path
# sys.path.insert(0, os.path.abspath("../../src"))
# Add each ROS package to the path if needed
sys.path.insert(0, os.path.abspath("../../src/easy_robot_control"))
sys.path.insert(0, os.path.abspath("../../src/motion_stack"))
# sys.path.insert(0, os.path.abspath("../../src/moonbot_zero_tuto"))
# sys.path.insert(0, os.path.abspath("../../src/ros2_m_hero_pkg"))
# sys.path.insert(0, os.path.abspath("../../src/rviz_basic"))
# sys.path.insert(0, os.path.abspath("../../src/urdf_packer"))
# Add .md Documentation to the path
# sys.path.insert(0, os.path.abspath("../../Documentation/design_principles.md"))
# sys.path.insert(0, os.path.abspath("../../README.md"))


project = "Motion-Stack"
copyright = "2024, Elian Neppel"
author = "Elian Neppel, Ashutosh Mishra"
release = "0.1"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx_toolbox.more_autodoc",
    "sphinx.ext.autodoc",  # Automatically include docstrings
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",  # Support for Google/NumPy docstrings
    "sphinx.ext.viewcode",  # Add links to source code
    "sphinx_autodoc_typehints",  # Include type hints in docs
    "myst_parser", # .md parser
    "sphinx_markdown_builder",
    "sphinx_copybutton",
]

templates_path = ["_templates"]
exclude_patterns = ["setup.py"]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_static_path = ["_static"]

# Theme configuration
html_theme = "sphinx_rtd_theme"  # ReadTheDocs theme
# Members are shown in the order they appear in the source code
autodoc_member_order = "bysource"
autodoc_typehints = "description"

# Configure autodoc default options
autodoc_default_options = {
    "members": True,  # Include all documented members
    "undoc-members": False,  # Include undocumented members
    "show-inheritance": True,  # Show the inheritance diagram
    "inherited-members": False,  # Include members inherited from parent classes
}


extensions.append("sphinx.ext.autosummary")

# Automatically generate summary `.rst` files
autosummary_generate = True
github_username = "2lian"
github_repository= "https://github.com/2lian/Moonbot-Motion-Stack"
