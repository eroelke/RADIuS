# get_atm_data.py
#
# Inputs:
#	p = planet data structure
#	h = altitude (m)
#
# Outputs:
#	rho = atmospheric density (kg/m3)
#	pressure = atmospheric pressure (Pa)
#	T = atmospheric temp (K)
#	wind = [3x1] Zonal(east), Meridional (North), Vertical (m/s)
#
# atmospheric table structure
#	[alt, density, pressure, temp, windE, windN, windV]

import numpy as np

def get_atm_dat( p, h ):
	mode = p.atm.mode
	cols = p.atm.atm_table.shape[1]	# number of columns in table
	if (mode == 0):	#exponential model
		rho = p.atm.rho0 * np.exp( -h/p.atm.H )
		pressure = p.atm.p0 * np.exp( -h/p.atm.H )
		T = p.atm.T0		# better way to estimate?
		winds = np.array([0,0,0])
	elif (mode == 1):	# table w/o winds
		h_col = p.atm.table[:,0]
		atm = np.interp( p.atm.atm_table[:,0], p.atm.table[:,1:(cols+1)] )
		rho = atm[0]
		pressure = atm[1]
		T = atm[2]
		winds = np.array([0,0,0])
	elif (mode == 2):	# table w/ winds
		h_col = p.atm.table[:,0]
		atm = np.interp( p.atm.atm_table[:,0], p.atm.table[:,1:(cols+1)] )
		rho = atm[0]
		pressure = atm[1]
		T = atm[2]
		winds = np.array([atm[3], atm[4], atm[5]])
	else:
		rho = 0
		pressure = 0
		T = 0
		wind = np.array([0,0,0])
	
	return (rho,pressure,T,wind)