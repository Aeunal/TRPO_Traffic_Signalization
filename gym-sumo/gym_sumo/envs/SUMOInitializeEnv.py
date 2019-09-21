from .sumo_env import SUMOEnv
import os

import os, sys, subprocess
from sys import platform

if platform == "win32":
    os.environ['SUMO_HOME'] = 'D:\\SUMO'

    try:
        import traci
        import traci.constants as tc
    except ImportError:
        if "SUMO_HOME" in os.environ:
            print(os.path.join(os.environ["SUMO_HOME"], "tools"))
            sys.path.append(
                os.path.join(os.environ["SUMO_HOME"], "tools")
            )
            try:
                import traci
                import traci.constants as tc
            except ImportError:
                raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")
        else:
            raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")
			
class SUMOEnv_Initializer(SUMOEnv):
	def __init__(self,mode='gui'):
		super(SUMOEnv_Initializer, self).__init__(mode=mode, simulation_end=36000)
