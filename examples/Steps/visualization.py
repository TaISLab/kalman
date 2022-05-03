

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('data.csv')


z   = df.z
z0   = df.a_r * np.sin(df.p_r)
z_u = df.a_u * np.sin(df.p_u)
z_e = df.a_e * np.sin(df.p_e)

plt.plot(z, label='real')
#plt.plot(z0, label='state')
#plt.plot(z_u, label='unscent')
plt.plot(z_e, label='extended')

plt.legend()
plt.show()
