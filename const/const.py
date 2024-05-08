import numpy as np

NP_ZERO     = np.array([0, 0])  # Zero vector
DT          = 0.1               # Time step
E           = 0.1               # Acceptable error
R_ROBOT     = 0.02              # Robot radius
SS_RANGE    = 0.5               # Sensing range

W_MTG       = 1                 # Weight for move to goal
W_AC        = 1                 # Weight for avoid collision
W_RW        = 1                 # Weight for random walk

N           = 5                 # Number of robots
NUM_STEP    = 500               # Limit number of steps

