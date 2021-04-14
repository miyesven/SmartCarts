import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os, sys

cwd = os.getcwd()
df1 = pd.read_csv('{}/concrete.csv'.format(cwd), names=['t', 'nsecs','x','y','qw'])
df2 = pd.read_csv('{}/carpet.csv'.format(cwd), names=['t', 'nsecs','x','y','qw'])

plt.scatter(df1['t'],df1['x'],s=2, label='concrete X')
plt.scatter(df1['t'],df1['y'],s=2, label='concrete Y')
plt.scatter(df2['t'],df2['x'],s=2, label='carpet X')
plt.scatter(df2['t'],df2['y'],s=2, label='carpet Y')
plt.xlabel('Time (nsecs)')
plt.ylabel('X, Y (m)')
plt.title('5 Rectangular Loops on Concrete and Carpet by Physical Robot : Odom Readings')
plt.legend()
plt.show()