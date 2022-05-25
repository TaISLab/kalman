

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

dataframe = pd.read_csv('data.csv')

# measurements
dv    = dataframe.dv
df    = dataframe.df

# measurements according to simulated state
s_dv   = dataframe.t_v0 + dataframe.t_v1 * np.sin(dataframe.t_vp)
s_df   = dataframe.t_f0 + dataframe.t_f1 * np.sin(dataframe.t_vp+dataframe.t_d)

# measurements according to filters
# Extended
e_dv   = dataframe.e_v0 + dataframe.e_v1 * np.sin(dataframe.e_vp)
e_df   = dataframe.e_f0 + dataframe.e_f1 * np.sin(dataframe.e_vp+dataframe.e_d)
# Unscented
u_dv   = dataframe.u_v0 + dataframe.u_v1 * np.sin(dataframe.u_vp)
u_df   = dataframe.u_f0 + dataframe.u_f1 * np.sin(dataframe.u_vp+dataframe.u_d)


plt.figure()
plt.plot(dv, label='real')
plt.plot(s_dv, label='state')
plt.plot(u_dv, label='unscent')
plt.plot(e_dv, label='extended')
plt.title('speed')
plt.legend()

plt.figure()
plt.plot(df, label='real')
plt.plot(s_df, label='state')
plt.plot(u_df, label='unscent')
plt.plot(e_df, label='extended')
plt.title('force')
plt.legend()

plt.show()
