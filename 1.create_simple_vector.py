
# Import libraries
import numpy as np
import matplotlib.pyplot as plt
 
# Vector origin location
X = [0]
Y = [0]
 
# Directional vectors
U = [2] 
V = [1] 
 
# Creating plot
plt.quiver(X, Y, U, V, color='b', units='xy', scale=1)
plt.title('Single Vector')
 
# x-lim and y-lim
plt.xlim(-2, 5)
plt.ylim(-2, 2.5)
 
# Show plot with grid
plt.grid()
plt.show()
