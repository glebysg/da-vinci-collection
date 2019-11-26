#!/usr/bin/env python
import dvrk
import numpy as np

# Change the camera position to the desired one:
p = dvrk.psm('ECM')
ecm_joint_pos = [-0.1423403, -0.0895354, 0.1725039, 0.2168578]
p.move_joint(np.array([ecm_joint_pos]))
