# guidance.py
#	run guidance systems
#
# Inputs:
# 	in0: simulation input structure
#	guid: guidance simulation data structure
#	dat: trajectory data structure (this time step)
#	i: current time step
#	nav: navitation simulation data structure
#
# Outputs:
#	guid: guidance data structure
#

import sys
sys.path.append('./guid_funcs/')

def guidance(in0,guid,dat,i,nav):
	guid_in = in0.guid.guid_struct
	if ( in0.guid.mode == 'mej_n' ):
		# multi-event jettison guidance
		from guid_mej_n import guid_mej_n
		guid.t = nav.t	# current time
		guid.r_pci = nav.r_pci	# inertial position
		guid.v_pci = nav.v_pci	# inertial velocity
		guid.a_sens_pci = nav.a_sens_pci	# inertial sensed acceleration
		guid = guid_mej_n(guid_in,guid)
	elif ( in0.guid.mode == 'mej_bi' ):
	
	# etc...
	
	return guid