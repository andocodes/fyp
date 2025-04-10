## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# --- Option 1: Minimal setup (Recommended for standard structure) ---
setup_args = generate_distutils_setup(
    packages=['butler_swarm'], 
    # package_dir={'': 'lib'} # Removed: Source code is now directly in src/butler_swarm
)

# --- Option 2: Explicit (If Option 1 fails, less common) ---
# setup_args = generate_distutils_setup(
#     packages=[
#         'butler_swarm', 
#         'butler_swarm.behaviours', 
#         'butler_swarm.utils'
#     ],
#     # package_dir={'': 'lib'} # Removed
# )


setup(**setup_args)
