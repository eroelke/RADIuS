# get_atm_dat.py
# obtain atmospheric data from planet data given altitude
#	
# Inputs:
#	p: planet input data structure
#	h: altitude
#
# Outputs:
#	rho: atmospheric density (kg/m3)
#	pressure: atmospheric pressure (Pa)
#	T: atmospheric temperature (K)
#	winds: wind speeds (East, North, Up) (m/s)
#

import numpy as np

def get_atm_data( p, h )
	if ( p.atm.mode == 0 ):	# exponential atmosphere
		pressure = p.atm.p0 * np.exp( -h / p.atm.H )
		rho = p.atm.rho0 * np.exp( -h / p.atm.H )
		T = p.atm.T0			# need a better way to estimate this
		winds = np.array([0,0,0])
	elif ( p.atm.mode == 1 ):	# table, ignore winds
		#atm = np.interp( p.atm.table, ,  )
		rho = atm[0]
		pressure = atm[1]
		T = atm[2]
		wind = np.array([0,0,0])
	elif ( p.atm.mode == 2 ):	# table, with winds
		#atm = np.interp( p.atm.table, ,  )
		rho = atm[0]
		pressure = atm[1]
		T = atm[2]
		wind = np.array([atm[3], atm[4], atm[5]])
	else:
		rho = 0
		pressure = 0
		T = 0
		winds = np.array([0,0,0])
	# check valid values
	if ( numpy.isnan(rho) == True ):
		rho = 0
		pressure = 0
		T = 0
		winds = np.array([0,0,0])
	return (rho,pressure,T,winds)
	
# get column of data from table
def column( A, col ):
	
	