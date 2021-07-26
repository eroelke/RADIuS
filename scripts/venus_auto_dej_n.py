# venus_auto_dej_n.py
#	Discrete-event drag modulation aerocapture at Venus using a Newton-method Numerical Predictor Corrector (NPC)
#
#
# Inputs:
#	ha_tgt: target apoapsis altitude (km)
#	
#
#
#
#

from input_structs import *



# function to run multi-event jettison
def venus_auto_mej_n:
	# default input struct
	in0 = default_in
	
	
	
	# create guidance structure
	h_bounds np.array([0, 150e3])
	masses = np.array([80 40])
	A_refs = np.pi * np.array([r1**2,r2**2])	# reference areas
	ra_tgt = 10e3
	cds = np.array([1.05] * 2])
	cls = np.zeros(2,1)
	K_bounds = np.array([0.1, 2])
	K_gain = 0.1
	mcflag = False	# not running monte carlo (density corrector is 1 for all time)
	tj0 = 80		# initial jettison time estimate
	n_jett = 1
	rate = 0.1		# guidance rate (Hz)
	atm_sens = 0.25
	mej_n_refs = create_mej(in0,h_bounds,masses,A_refs,ra_tgt,cds,cls,K_bounds,K_gain,mcflag,tj0,n_jett,rate,atm_sens)
	guid = create_guid('mej_n',mej_n_refs)
	in0.guid.guid_struct = guid
	
	
	# run simulation
	out = run_radius(in0)
	haf = out.haf/1000		#final apoapsis, km
	haf_err = haf - ha_tgt	# apoapsis error, km
	
		
	return (out,haf,haf_err,tjett)
	
	
class define_refs:
	def __init__(self,
	