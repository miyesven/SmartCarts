import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os, sys

concrete_x_arr = [0.275, 0.36, 0.53, 0.585, 0.535]
concrete_x_t_arr = [1.30e11, 2.47e11, 3.64e11, 4.92e11, 6.47e11]
concrete_y_arr = [0.13,0.215,0.265,0.31,0.45]
concrete_y_t_arr = [1.56e11, 2.73e11, 3.91e11, 5.18e11, 6.75e11]

carpet_x_arr = [0.665, 0.815, 0.875, 1.02, 1.06]
carpet_x_t_arr = [1.17e11, 2.75e11, 4.22e11, 5.46e11, 6.6e11] 
carpet_y_arr = [0.115, 0.130, 0.240, 0.295, 0.390]
carpet_y_t_arr = [1.45e11, 3.02e11, 4.5e11, 5.75e11, 6.8611e11]

cwd = os.getcwd()
df1 = pd.read_csv('{}/concrete.csv'.format(cwd), names=['t', 'nsecs','x','y','qw'])
df2 = pd.read_csv('{}/carpet.csv'.format(cwd), names=['t', 'nsecs','x','y','qw'])
df1['t'] = df1['t'] - df1['t'].min()
df2['t'] = df2['t'] - df2['t'].min()

starttime1 = df1[df1['x']>1.75]['t'].min()
starttime2 = df2[df2['x']>1.75]['t'].min()

timediff = starttime2 - starttime1
prev_x = 0

plt.scatter(df1['t']+timediff,df1['y'],s=2, color='cornflowerblue', alpha=0.6, label='(Odom) concrete Y')
plt.scatter(df2['t'],df2['y'],s=2, color='lightcoral', alpha=0.6, label='(Odom) carpet Y')
plt.plot(concrete_y_t_arr, concrete_y_arr, 'o--', color='royalblue', label='(Actual) concrete Y error')
plt.plot(carpet_y_t_arr, carpet_y_arr, 'o--', color='indianred', label='(Actual) carpet Y error')
plt.xlabel('Time (nsecs)')
plt.ylabel('X, Y (m)')
plt.title('5 Rectangular Loops on Concrete and Carpet by Physical Robot : Y Odom Readings & Y Actual Error')
plt.legend()

# plt.scatter(df1['t']-df1['t'].min()+df2['t'].min(),df1['x'],s=2, color='cornflowerblue', label='concrete X')
# plt.scatter(df1['t']-df1['t'].min()+df2['t'].min(),df1['y'],s=2, color='royalblue', label='concrete Y')
# plt.scatter(df2['t'],df2['x'],s=2, color='lightcoral', label='carpet X')
# plt.scatter(df2['t'],df2['y'],s=2, color='indianred', label='carpet Y')
# plt.xlabel('Time (nsecs)')
# plt.ylabel('X, Y (m)')
# plt.title('5 Rectangular Loops on Concrete and Carpet by Physical Robot : Odom Readings')
# plt.legend()
plt.show()