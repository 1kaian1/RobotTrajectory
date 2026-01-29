# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'RobotTrajectory'
copyright = '2026, KaiMagnus'
author = 'KaiMagnus'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
]

import os
import sys
sys.path.insert(0, os.path.abspath('../../src/fhtw/soarMazeEscape'))

templates_path = ['_templates']
autodoc_mock_imports = ["rospy", "tf", "nav_msgs", "geometry_msgs", "numpy", "heapq", "scipy", "cv2"]
exclude_patterns = ['_build', '**.ipynb_checkpoints']




# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']
