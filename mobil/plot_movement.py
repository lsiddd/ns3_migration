import matplotlib.pyplot as plt 
import numpy as np

def read_file(filename, node_id, coord = "x"):
    with open(filename) as f:
        if coord == "x":
            lines = [float(i.split()[5])/1500 for i in f.readlines() if
                     "node_({})".format(node_id) in i
                    and "setdest" in i]
        elif coord == "y":
            lines = [float(i.split()[6])/1500 for i in f.readlines() if
                     "node_({})".format(node_id) in i
                    and "setdest" in i]
        else:
            raise ValueError("invalid coordinate")
        return lines
filename = "carroTrace.tcl"
for i in range(1):
    plt.plot(read_file(filename, i, "x"), read_file(filename, i, "y"))
plt.show()