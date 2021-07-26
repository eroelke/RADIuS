# run_radius.py
#	Run simulation with all inputs
#
# RADIuS: Reentry Atmospheric DynamIcs Simulation
# 	Created By: Evan Roelke
#	Re-Entry Dynamics Simulation Lab (RDSL)
# 	CU Boulder
#
# Inputs:
#	in0: simulation input data stucture (see input_structs.py)
#
# Outputs:
#	out: simulation output data structure (see output_structs.py)
#

import numpy as np
import sys as s

# add all subfolders to path
s.path.append('./tools/')
s.path.append('./guid_funcs/')
s.path.append('./ctrl_funcs/')
s.path.append('./data/'

# import relevant functions
from define_out import init_dat
from calculations import traj_calcs
from traj_structs import *
from define_out import *

def run_traj(in0):
	out = initialize( in0, veh )
	terminate = False
	
	# get simulation ratios
	dt = 1/in0.sim.traj_rate	# integration time step
	dat_ratio = round(in0.sim.traj_rate/in0.sim.data_rate, 0)	# simulation/data rate ratio
	guid_ratio = round(in0.sim.traj_rate/in0.sim.guid_rate, 0)	# guidance call rate
	nav_ratio = round(in0.sim.traj_rate/in0.sim.nav_rate, 0)	# navigation call rate
	ctrl_ratio = round(in0.sim.traj_rate/in0.sim.ctrl_rate, 0)	# control call rate
	
	tvec = np.linspace(in0.s.traj.t0, dt, in0.s.traj.tf)	# time vector
	
	# create output structure
	out = create_output_struct(in0,tvec)
	
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
	
	nav = nav_struct(self,in0.nav.mode,seed,tau,r0_pci,r_pcpf,v_pci,v_pcpf,a_sens_pci,
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
			ctrl,guid,veh = control(in0,veh,ctrl,guid,t,i,dat)
					
		# data storage
		if ( i-1 % dat_ratio ) == 0:
			di += 1		# next data storage index
			out = store_dat( out, dat, dat_prev, di, ti, veh, guid, nav, ctrl, out, in0 )
			
			
			
		
		# check termination
		terminate = check_terminate( in0, xi, dat, dat_prev )
		if terminate == True
			# write final data
			if ( i-1 % dat_ratio ) != 0:
				di += 1		# next data storage index
				out = store_dat( out, dat, dat_prev, di, ti, veh, nav, guid, ctrl, out, in0 )
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
	  

# function to store data into output struct
def store_dat( out, dat, dat_prev, di, ti, veh, nav, guid, ctrl, out, in0 ):
	# out: output data structure (defined in define_out.py)
	# dat: trajectory data structure
	# dat_prev: previous time step data structure
	# di: data storage index
	# ti: current time index
	# veh: vehicle data struct
	# nav: navigation data struct
	# guid: guidance data struct
	# ctrl: control data struct
	# out: output data struct
	# in0: input data struct
	
	# store trajectory data
	out.traj.t[di] = ti
	out.traj.r_pci[di] = dat.r_pci
	out.traj.rmag_pci[di] = dat.rmag_pci
	out.traj.v_pci[di] = dat.v_pci
	out.traj.vmag_pci[di] = dat.vmag_pci
	out.traj.r_pcpf[di] = dat.r_pcpf		# position, pcpf frame (m)
	out.traj.rmag_pcpf[di] = dat.rmag_pcpf	# position mag, pcpf frame (m)
	out.traj.v_pcpf[di] = dat.v_pcpf		# velocity, pci frame (m/s)
	out.traj.vmag_pcpf[di] = dat.vmag_pcpf	# velocity mag, pcpf frame (m/s)
	out.traj.alt[di] = dat.alt			# altitude (m)
	out.traj.rho[di] = dat.rho			# atmospheric density (kg/m3)
	out.traj.azi_pcpf[di] = dat.azi_pcpf	# azimuth angle, pcpf frame (rad)
	out.traj.q[di] = dat.q				# dynamic pressure (wind-relative) (Pa)
	out.traj.fpa_pci[di] = dat.fpa_pci		# flight path angle, PCI frame (rad)
	out.traj.fpa_pcpf[di] = dat.fpa_pcpf	# flight path angle, PCPF frame (rad)
	out.traj.n[di] = dat.n					# g-loading (earth Gs)
	out.traj.mach[di] = dat.mach		# mach number
	out.traj.dr[di] = dat.dr			# downrange (m)
	out.traj.cr[di] = dat.cr			# crossrange (m)
	out.traj.qdot_s[di] = dat.qdot_s	# heat rate (W/m2)
	out.traj.lat[di] = dat.lat			# latitude (rad)
	out.traj.lon[di] = dat.lon			# longitude (rad)
	
	# store vehicle data
	out.v.mass[di] = dat.mass		# vehicle total mass (kg)
	out.v.m_prop[di] = dat.m_prop	# propellant mass (kg)
	out.v.area[di] = dat.A_ref		# reference area (m2)
	out.v.cl[di] = dat.cl		# lift coefficient
	out.v.cd[di] = dat.cd		# drag coefficient
	out.v.LD[di] = dat.LD		#lift-to-drag ratio
	out.v.aoa[di] = dat.aoa			# angle of attack (rad)
	out.v.phi[di] = dat.phi			# bank angle (rad)
	out.v.phi_rate[di] = phi_rate	# bank rate (rad/s)
	out.v.phi_accel[di] = phi_accel	# bank accel (rad/s2)
	out.v.ssa[di] = ssa			# sideslip angle (rad)
	out.v.r_cg[di] = r_cg		# center of gravity pos (m)
	out.v.Fg_pci[di] = dat.Fg_pci	# gravity force, pci frame (N)
	out.v.Fg_pcpf[di] = dat.Fg_pcpf	# gravity force, pcpf frame (N)
	out.v.D_pci[di] = dat.D_pci		# drag force, pci frame (N)
	out.v.Dmag_pci[di] = dat.Dmag_pci	# drag force mag, pci frame (N)
	out.v.L_pci[di] = dat.L_pci		# lift force, pci frame (N)
	out.v.Lmag_pci[di] = dat.Lmag_pci	# lift force mag, pci frame (N)
	out.v.D_decel[di] = dat.D_decel		# decelerator drag force (N)
	  
	# store guidance data (based on mode)
	if ( ( in0.guid.mode == 'mej_n' ) or ( in0.guid.mode == 'mej_bi' ) ):
		out.g.stage[di] = guid.stage
		out.g.a_mag[di] = guid.a_mag
		out.g.K_dens[di] = guid.K_dens
		out.g.tj_curr[di] = guid.tj_curr
		out.g.ra[di] = guid.ra
		out.g.ra_err[di] = guid.ra_err
		out.g.ra_prev[di] = guid.ra_prev
	#elif ():
	
	# store control data
	out.c.deploy_decel[di] = ctrl.deploy_decel
	out.c.jettison[di] = ctrl.jettison
	out.c.aoa[di] = ctrl.cmd_aoa
	out.c.aoa_rate[di] = ctrl.cmd_aoa_rate
	out.c.aoa_accel[di] = ctrl.cmd_aoa_accel
	out.c.bank[di] = ctrl.cmd_bank
	out.c.bank_rate[di] = ctrl.cmd_bank_rate
	out.c.bank_accel[di] = ctrl.cmd_bank_accel
	
	# store navigation data
	out.n.r_pci[di] = nav.r_pci
	out.n.r_pcpf[di] = nav.r_pcpf
	out.n.v_pci[di] = nav.v_pci
	out.n.v_pcpf[di] = nav.v_pcpf
	out.n.a_sens_pci[di] = nav.a_sens_pci
	out.n.a_sens_pcpf[di] = nav.a_sens_pcpf
	out.n.rva_err[di] = nav.rva_err
	out.n.lat_gd[di] = nav.lat_gd
	out.n.lon[di] = nav.lon
	out.n.alt[di] = nav.alt
	out.n.t[di] = nav.t	
	
	
	
	
	
	
	
	
	
	

	
	