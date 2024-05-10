import numpy as np

NP_ZERO     = np.zeros(2)       # Zero vector
DT          = 0.1               # Time step
E           = 0.1               # Acceptable error
R_ROBOT     = 0.2               # Robot radius
SS_RANGE    = 2.0               # Sensing range
OFFSET      = 30                # Random range
W_MTG       = 1.5               # Weight for move to goal
W_AC        = 1.5               # Weight for avoid collision
W_RW        = 0.1               # Weight for random walk

N           = 30                # Number of robots
NUM_STEP    = 1500              # Limit number of steps

