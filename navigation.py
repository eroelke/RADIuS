# navigation.py
#	simulate navigation sensors for trajectory
#
# Inputs:
#	in0:	input structure
#	nav:	navigation state structure
#	t:		simulation time (s)
#	dt:		integration time step size (s)
#	i:		integration loop index
#	dat:	calculation data structure
#	
# Outputs:
#	nav:	navigation state structure
#

import numpy as np
import sys

sys.path.append('./tools/')

from init_structs import nav_struct
from get_M import get_M

def navigation(in0,nav,t,dt,i,dat):
	# get relevant params from input
	tau = in0.nav.tau
	P_SS = in0.nav.P_SS
	# get state
	r_pci = dat.r_N
	v_pci = dat.v_N
	a_sens_pci = ( dat.F_N - dat.Fg_N )/dat.mass
	
	if ( nav.mode == 1 ):	# add noise to state
		noise = np.dot( (1-np.exp(-2*dt/tau) ), nav.rnd[:,i] )
		x_ercv = np.dot( np.exp(-1*dt/tau), nav.x_ercv ) + noise
		rva_err = np.dot( P_SS, nav.x_ercv )
		
		# update state
		r_pci += rva_err[0:3]
		v_pci += rva_err[3:6]
		a_sens_pci += rva_err[6:9]
	
	theta_g = in0.p.omega * ( in0.traj.t0 + t )
	PI = get_M(theta_g,3)
	v_pcpf = np.dot( PI, ( v_pci - np.cross(in0.p.omega,r_pci) ) )
	r_pcpf = np.dot( PI, r_pci )
	a_sens_pcpf = np.dot( PI, a_sens_pci )
	
	lat_gd,lon,alt = get_lat_lon(r_pcpf,in0.p.rE,in0.p.rP)
	
	# udpate navigation structure
	nav = nav_struct(r_pci,r_pcpf,v_pci,v_pcpf,a_sens_pci,
	  a_sens_pcpf,lat_gd,lon,alt,t,rnd,x_ercv,rva_err)
	
	return nav	