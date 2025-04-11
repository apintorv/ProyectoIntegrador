#BROWNIAN MOTION W3
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Parámetros del sistema
tao = 0.01  # Paso de integración

# Estados iniciales
x = np.array([[0] ]).T 
u = np.array([[0]])

dummy = np.array([[0]]) 
dummy = np.hstack((dummy, x.T))
out = dummy

for n in range(100):
  x = np.array([[0] ]).T 

  for t in np.arange(tao, 10, tao):
    
      w = np.random.normal(0,0.001, 1)
      x = x + tao*(w)
      
      dummy = np.array([[t]]) 
      dummy = np.hstack((dummy, x.T))
      out = np.vstack((out, dummy))

out_df = pd.DataFrame(out, columns=['t', 'x'])

out_df.plot(x= 't', y=['x'])
