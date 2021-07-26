# get_unit_NED
#
# Inputs:
#	-latitude and longitude, 
# Outputs:
#	-unit vectors in north, east, down directions
#	-planet centered, planet-fixed frame (PCPF)

import numpy as np
from get_M import get_M

def get_unit_NED(lat,lon):
	# compute in XYZ coords
	uNxyz = [ -np.sin(lat), 0, np.cos(lat) ]
	uExyz = [0, 1, 0]
	uDxyz = [ -np.cos(lat), 0, -np.sin(lat) ]
	
	# rotate by lon to PCPF frame
	PI = get_M(lon,3)
	uN = PI*uNxyz
	uE = PI*uExyz
	uD = PI*uDxyz
	
	return (uN,uE,uD)