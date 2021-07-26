# calc_lat_lon

import numpy as np

def get_lat_lon(r_pp,re,rp):
	f = (re-rp)/re	# flattening
	el = 1 - (1-f)**2	# ellipticity
	rmag_pp = norm(r_pp)
	s = norm( [r_pp[0], r_pp[1]] )	# equatorial range
	
	# calc initial guess for reduced lat, planet-detic lat
	latr = np.arctan( r_pp[2]/( (1-f)*s ) )
	latd = get_latd(r_pp,re,f,el,s)
	latr2 = get_latr2(f,latd)
	diff = latr - latr2
	
	# iterate until difference is small
	while (diff > 1e-7):
		latr = latr2
		latd = get_latd(r_pp,re,f,el,s)
		latr2 = (f,latd)
		diff = latr - latr2
	
	lat = latd
	lon = np.arcsin( r_pp[1]/( numpy.sqrt( r_pp[0]**2 + r_pp[1] ) ) )
	
	# quadrant checks
	if ( r_pp[0] < 0 ):
		lon = pi - lon
	elif ( r[0] > 0 and r[1] < 0 ):
		lon += 2*np.pi
		
	# altitude calcs
	if ( re == rp ):
		alt = pos_ii_mag - re
	else:
		k = re/( np.sqrt( 1-el*( np.sin(lat)**2 ) ) )	# radius of curvature in vertical prime
		alt = s*np.cos(lat) + np.sin(lat)*( r_pp[2] + el*k*np.sin(lat) ) - k
		
	return (lat,lon,alt)
		
		
def get_latd(r_pp,re,f,el,s):
	latd = np.arctan( ( r_pp[2]+el*(1-f)*re*(np.sin(latr)**3)/(1-el) ) / \
		( s - el*re*(np.cos(latr)**3) )  )
	return latd
	
def get_latr2(f,latd):
	latr2 = np.arctan( ((1-f)*np.sin(latd))/np.cos(latd) )
	return latr2
	
def norm(x):
	x = np.linalg.norm(x)
	return x