# import functions to read xml file and visualize commonroad objects
import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.draw_params import MPDrawParams

# generate path of the file to be opened
file_path = "DEU_Flensburg-1_1_T-1.xml"

# read in the scenario and planning problem set
scenario, planning_problem_set = CommonRoadFileReader(file_path).open()

# plot the planning problem and the scenario for the fifth time step
plt.figure(figsize=(25, 10))
rnd = MPRenderer()
rnd.draw_params.time_begin = 5
scenario.draw(rnd)
planning_problem_set.draw(rnd)
rnd.render()
plt.show()