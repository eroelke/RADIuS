# control.py
#	Apply control to vehicle based on results of guidance
#
#
#
#

def control(in0,veh,ctrl,guid,tvec,i,dat):
	# get current state parameters
	aoa = dat.aoa
	bank = dat.phi
	dt = tvec[i] - tvec[i-1]
	t = tvec[i]
	
	aoa_cmd = guid.aoa_cmd
	bank_cmd = guid.bank_cmd
	
	# bank modes
	if (in0.ctrl.s_bank.mode == 0 ):
	elif (in0.ctrl.s_bank.mode == 1 ):
	
	# aoa modes
	if (in0.ctrl.s_aoa.mode == 0 ):
	elif (in0.ctrl.s_aoa.mode == 1 ):
	
	# decel modes
	if (in0.ctrl.s_decel.mode == 0 ):
	
	# jettison modes
	if ( (in0.guid.mode == 'mej_n') or (in0.guid.mode == 'mej_bi') ):
		if ( t >= guid.tj_curr and guid.stage < in0.guid.guid_struct.n_jett ):
			ctrl.jflags[guid.stage] = True
			guid.stage += 1
			veh.mass = guid.masses[guid.stage]
			veh.A_ref = guid.A_refs[guid.stage]
	
	return ctrl,guid,veh