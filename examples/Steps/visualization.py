

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('data.csv')


z_x    = df.z_x
z0_x   = df.d_r_x + df.a_r_x * np.sin(df.p_r_x)
z_u_x  = df.d_u_x + df.a_u_x * np.sin(df.p_u_x)
z_e_x  = df.d_e_x + df.a_e_x * np.sin(df.p_e_x)

plt.plot(z_x, label='real')
#plt.plot(z0, label='state')
#plt.plot(z_u_x, label='unscent')
plt.plot(z_e_x, label='extended')

plt.legend()
plt.show()



z_y   = df.z_y
z0_y  = df.d_r_y + df.a_r_y * np.sin(df.p_r_y)
z_u_y = df.d_u_y + df.a_u_y * np.sin(df.p_u_y)
z_e_y = df.d_e_y + df.a_e_y * np.sin(df.p_e_y)

plt.plot(z_y, label='real')
#plt.plot(z0, label='state')
#plt.plot(z_u_y, label='unscent')
plt.plot(z_e_y, label='extended')

plt.legend()
plt.show()
