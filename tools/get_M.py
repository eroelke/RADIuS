# rotation matrices

import numpy as np

def get_M(q, ax):
	if ax == 1:
		M = np.matrix([	[1,	0,	0], \
						[0,np.cos(q), np.sin(q)], \
						[0,-np.sin(q),np.cos(q)]	])
	elif ax == 2:
		M = np.matrix([	[np.cos(q), 0, -np.sin(q)], \
						[0, 		1, 		0], \
						[np.sin(q), 0, np.cos(q)]	])
	elif ax == 3:
		M = np.matrix([	[np.cos(q), np.sin(q), 0], \
						[-np.sin(q),np.cos(q), 0], \
						[0,		 0,			1]		])
	else:
		print('Error: Incorrect rotation axis specification. Defaulting to axis 3 (khat)')
		M = np.matrix([	[np.cos(q), np.sin(q), 0], \
						[-np.sin(q),np.cos(q), 0], \
						[0,		 0,			1]		])
	return M