# guid_mej_n.py
#	Run multi-event jettison Numerical Predictor Corrector algorithm
#
#	Inputs:
#		guid: guidance input structure
#	Outputs:
#		guid: guidance input structure

import sys
sys.path.append('./tools/')

from rv2oe import rv2oe

def guid_mej_n(guid_in,guid):
	#if guid.init == False
	#	# initialize the algorithm
	#	guid.jett = [False] * in0.guid.guid_struct.n_jett
	
	j_ind = guid.s.stage	# index of current system
	if ( guid.a_sens_pci >= guid.s.a_init ):	# check if reached sensible atmosphere
		guid.s.ncalls += 1	# increase counter for number of times func is called
		if ( j_ind <= guid_in.n_jett ):
			if ( guid_in.mcflag == True )
				guid = get_K_dens(guid,j_ind)
			else:
				guid.s.K_dens = 1
			
			# initialize predictor
			y0 = np.array([guid.r_pci, guid.v_pci])
			t0 = guid.t
			
			# run newton method
			dt = 1/guid_in.rate
			guid = mej_n_pred(guid,y0,t0,j_ind,0)
			p2 = mej_n_pred(guid,y0,y0,j_ind,dt)
			drdt = ( p2.ra_err - guid.ra_err )/dt
			d = guid.ra_err/drdt		#f/f' in newton method
			
			# bound newton method
			if ( d >= 30 ):
				d = 30
			elif ( d <= -30 ):
				d = -30
				
			# update next jettison time to try
			guid.tj_next[j_ind] -= d
	
			# check if store new jettison time
			if ( guid.ra_prev == 0):	# not yet initialized
				guid.ra_prev = guid.ra_err
			elif ( guid.ra_err <= guid.ra_prev ):
			
	
	return guid
		
		
def mej_n_pred(g,y0,t0,j_ind,dt):
	y = y0	# initialize state
	t = t0	# initialize time
	K = g.K_dens	# density corrector
	tj0 = g.tj_next[j_ind]	# jettison time to try
	flag = False	# termination flag
	h = 1/g.rate	# integration rate
	jflag = False
	
	while ( flag == False ):
		if ( (t >= tj0) and (jflag == False) ):
			jflag = True
			j_ind += 1
		# get current vehicle state parameters
		mass = g.mass[j_ind]
		area = g.Arefs[j_ind]
		cd = g.cds[j_ind]
		cl = g.cls[j_ind]
		
		# rk4 integrator
		k1 = calc_eom(y,t,g,area,mass,cl,cd,K)
		k2 = calc_eom( (y + 0.5*k1*h), t + h/2, g,area,mass,cl,cd,K)
		k3 = calc_eom( (y + 0.5*k2*h), t + h/2, g,area,mass,cl,cd,K)
		k4 = calc_eom( (y + k3*h), t + h, g,area,mass,cl,cd,K)
		y += (h/6)*( k1 + (2*k2) + (2*k3) + k4 )
		t += h
		
		# get altitude
		alt = np.linalg.norm(y[0:3])	# current altitude
		
		# check integrator termination
		if ( alt >= g.p.alt_bounds[1] ):		# alt > alt_max
			flag = True
		elif ( alt <= g.p.alt_bounds[0] ):
			flag = True
		# max integration time?
		
	# Calculate orbit parameters
	rf_pci = y[0:3]
	vf_pci = y[3:6]
	
	oe = oe2rv(y, g.p.mu)	# get orbital elements from final state
	a = oe[0]	# semi-major axis (m)
	e = oe[1]	# eccentricity
	g.ra = a*(1-e)	# porjected apoapsis from current state
	g.ra_err = g.ra - ( g.ha_tgt + g.p.rp )
	
	return g
	
# equations of motion calculation	
def calc_eom(y,t,g,area,mass,cl,cd,K):
	
		
		
	ydot = np.array([v_pci,a_pci])
		
		