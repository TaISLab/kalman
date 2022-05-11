

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('data.csv')


z_x   = df.z_x
z_u_x = df.z_u_x
z_e_x = df.z_e_x

plt.plot(z_x, label='measurement')
#plt.plot(z0, label='state')
plt.plot(z_u_x, label='unscent')
plt.plot(z_e_x, label='extended')

plt.legend()
plt.show()
