

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('data.csv')

t     = np.cumsum(df.dt)
z_x   = df.z_x
z_u_x = df.z_u_x
z_e_x = df.z_e_x

#plt.plot(t, z0, label='state')
#plt.plot(t, z_u_x, label='unscent')
plt.plot(t, z_e_x, label='extended')
plt.plot(t, z_x, label='measurement')
plt.legend()
plt.show()
