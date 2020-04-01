import numpy as np
import matplotlib.pyplot as plt

with open('./raspberry/testing/200326.log', 'r') as file:
    lines = file.readlines()
data = np.array([[float(d)
                  for d in l.replace('\n', '').split(': ')[1].split(', ')]
                 for l in lines[10::]]).T

data
