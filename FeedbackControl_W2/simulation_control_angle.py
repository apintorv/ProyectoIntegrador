import numpy as np
import pandas as pd

h = 0.1

K_pos = 1
K_th = 2


qd = np.array([[2,1]]).T
th_d = np.array([[np.pi/2]])

x = np.array([[0, 0, 0]]).T
u = np.array([[0, 0]]).T



tao = 0.01

dummy = np.array([[0]])
dummy = np.hstack((dummy, x.T))
out = dummy

flag_ctrl = 0
for t in np.arange(tao, 20, tao):

    B = np.array([
        [np.cos(x[2][0]), -h*np.sin(x[2][0])],
        [np.sin(x[2][0]), h*np.cos(x[2][0])],
        [0, 1]
    ])

    D = np.array([
        B[0],
        B[1]
    ])

    phi = np.array([B[2]])

    if flag_ctrl == 0:
      e_pos = x[0:2] - qd
      u = np.linalg.inv(D)@(-K_pos * e_pos)
    elif flag_ctrl == 1:
      e_th = x[2][0] - th_d
      u = phi.T@(-K_th * e_th)

    x = x + tao*(B@u)

    if np.linalg.norm(e_pos) < 0.001 and flag_ctrl == 0:
      flag_ctrl = 1
    elif np.abs(e_th) < 0.0001 and flag_ctrl == 1:
      flag_ctrl = 2

    dummy = np.array([[t]])
    dummy = np.hstack((dummy, x.T))

    out = np.vstack((out, dummy))

out_df = pd.DataFrame(out, columns=['t', 'x', 'y', 'theta'])

out_df.plot(x='t', y=['x', 'y', 'theta'], style='-')
out_df.plot(x='x', y=['y'],style ='-')