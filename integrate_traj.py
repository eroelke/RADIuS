import math
import numpy as np
from define_out import init_dat
from calculations import traj_calcs

from calculations import traj_calcs
from init_structs import *

def run_traj(in0):
	out = initialize( in0, veh )
	terminate = False
	dt = 1/in0.sim.traj_rate	# integration time step
	dat_ratio = round(in0.sim.traj_rate/in0.sim.data_rate, 0)	# simulation/data rate ratio
	guid_ratio = round(in0.sim.traj_rate/in0.sim.guid_rate, 0)	# guidance call rate
	nav_ratio = round(in0.sim.traj_rate/in0.sim.nav_rate, 0)	# navigation call rate
	ctrl_ratio = round(in0.sim.traj_rate/in0.sim.ctrl_rate, 0)	# control call rate
	tvec = np.linspace(in0.s.traj.t0, dt, in0.s.traj.tf)	# time vector
	
	# initialize state
	x0 = [in0.traj.r0_PCI,	# position vector (ECI), m
		  in0.traj.v0_PCI,			# velocity vector (ECI), m/s
		  in0.veh.mass,			# mass, kg
		  in0.veh.prop.m_prop,		# main engine fuel mass, kg
		  in0.veh.prop.m_rcs,		# rcs fuel mass, kg
		  in0.traj.dr0,				# downrange, m
		  in0.traj.cr0,				# crossrange, m
		  0]							# integrated heat load
	t0 = tvec[0]			# initial time
	
	# initialize systems
	#guid = init_guid(in0)
	nav = init_nav(in0,tvec)
	ctrl = init_ctrl(in0)
	veh = init_veh(in0)
	
	veh = nav_struct(self,in0.nav.mode,seed,tau,r0_pci,r_pcpf,v_pci,v_pcpf,a_sens_pci,
	  a_sens_pcpf,lat_gd,lon,alt,t,x_ercv,rva_err)
	
	# get initial simulation data
	dat = traj_calcs( in0, t0, x0, veh, ctrl )
	dat_prev = dat	# save current trajectory calcs (for termination conditions)

	# update state
	x[i][7] -= dat.dm	# change in mass this time step
	x[i][8] = dat.m_prop	# new propellant mass
	az_pcpf0 = dat.az_pcpf	# initial azimuth (rad)
	
	# integration parameters
	N = len(tvec)	# num of integration steps
	di = 0			# data storage index
	for i in range(1,N):
		ti = tvec[i]		# current time
		xi = x[i]		# full row vector of state at current time step
		dat = traj_calcs( in0, ti, xi, in0.veh, in0.ctrl)
		dat_prev = dat
		
		# update state
		x[i][7] -= dat.dm	# change in mass this time step
		
		# check guidance
		if ( ( (i-1) % nav_ratio ) == 0):
			nav = navigation(in0,nav,ti,dt,i,dat)
		if ( ( (i-1) % guid_ratio ) == 0):
			guid = guidance()
		if ( ( (i-1) % ctrl_ratio ) == 0):
			ctrl = control(in0,ctrl,guid,t,i,dat)
		
		
		# data storage
		if ( i-1 % dat_ratio ) == 0:
			di += 1		# next data storage index
			out = store_dat( dat, dat_prev, di, ti, veh, guid, nav, ctrl, out, in0 )
			
			
			
		
		# check termination
		terminate = check_terminate( in0, xi, dat, dat_prev )
		if terminate == True
			# write final data
			if ( i-1 % dat_ratio ) != 0:
				di += 1		# next data storage index
				out = store_dat( dat, dat_prev, di, ti, veh, nav, guid, ctrl, out, in0 )
			break		
	return out
	
	


# check termination - single condition
	# add multiple possible conditions (and,or,nand)
def check_terminate( in0, x, dat, dat_prev ):
	terminate = False
	sign = np.sign( in0.sim.terminate.dir )	# +- 1
	value = in0.sim.terminate.val		# value to check crossing at
	if ( in0.sim.terminate.var == 0 ):	# time condition
		v = dat.time
		vp = dat_prev.time	
	elif ( in0.sim.terminate.var == 1 ):	# altitude
		v = dat.alt
		vp = dat_prev.alt
	elif ( in0.sim.terminate.var == 2 ):	# velocity
		v = dat.vmag_pp
		vp = dat_prev.vmag_pp
	elif ( in0.sim.terminate.var == 3 ):	# energy
		v = dat.xi
		vp = dat_prev.xi
		
	if ( sign > 0):				# positive crossing 
		if ( v >= value and v >= vp ):	# the value is increasing and we're above target value
			terminate = True
	else:						# negative crossing
		if ( v <= value and v <= vp ):	# the value is decreasing and we're below target value
			terminate = True
	return terminate

	
	