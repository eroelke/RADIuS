# define trajectory calculation output data structure
# perform trajectory calculations (called at each integration step)
#
#
#
# Frames:
#	N: inertial (planet-centered inertial)
#	PCPF: planet-relative (planet-centered, planet-fixed)

import numpy as np
import sys

sys.path.append('./tools/')	# add all tools to path

from get_M import get_M
from calc_lat_lon_alt import get_lat_lon
from calc_NED import get_unit_NED

# trajectory calculations
def traj_calcs( in0, t, x, veh, ctrl ):
	# extract planet parameters
	wP = in0.p.omega		# planet rotation speed (rad/s)
	gamma = in0.p.atm.gamma		# specific heat ratio
	rP = in0.p.r_p		# planet polar radius
	rE = in0.p.r_e		# planet equatorial radius
	mu = in0.p.mu		# planet gravitational param (kg3/m2)
	j2 = in0.p.j2		# planet j2 harmonic
	R = in0.p.atm.R		# ideal gas constant
	
	# extract state variables
	r_N = np.array(x[0:3])	# inertial position
	v_N = np.array(x[3:6])	# inertial velocity
	m = x[6]
	rmag_N = norm(r_N)
	vmag_N = norm(v_N)
	
	# get planet-relative state
	thetaP = norm(wP)*(in0.traj.t0 + t)	# angle planet has turned (rad)
	PI = get_M(thetaP,3)		# DCM about k_hat: inertial to PCPF
	IP = np.transpose(PI)		# PCPF to inertial
	r_N_hat = r_N/rmag_N		# unit vector of inertial position
	r_pcpf = np.dot(PI,r_N)		# planet-relative position (planet-centered, planet-fixed frame)
	rmag_pcpf = norm(r_pcpf)
	r_pcpf_hat = r_pcpf/rmag_pcpf		# unit vector of relative position
	
	v_pci_pcpf = v_N - np.cross(wP,r_pci)	# velocity relative to planet in PCI frame (transport thm)
	v_pcpf = np.dot(PI,v_pci_pcpf)		# planet-centered, planet-fixed (pcpf) velocity
	vmag_pcpf = norm(v_pcpf)
	v_pcpf_hat = v_pcpf/vmag_pcpf
	
	# angular momentum calculations
	h_N = np.cross(r_N,v_N)		# inertial angular momentum
	hmag_N = norm(h_N)
	h_pcpf = np.cross(r_pcpf,v_pcpf)		# planet-relative ang. momentum
	hmag_pcpf = norm(h_pcpf)
	h_pcpf_hat = h_pcpf/hmag_pcpf
	
	# flight path angle calcs (limited to -pi to pi)
	fpa_N = np.arccos( np.median( [-1, 1, hmag_N/(rmag_N*vmag_N)] ) );	# inertial FPA
	if (np.dot(r_N,v_N) < 0):
		fpa_N = -fpa_N
	fpa_pcpf = np.arccos( np.median( [-1, 1, hmag_pcpf/(rmag_pcpf*vmag_pcpf)] ) );
	if (np.dot(r_pcpf,v_pcpf) < 0):
		fpa_pcpf = -fpa_pcpf
	
	# latitude, longitude, altitude calcs
	lat,lon,alt = get_lat_lon_alt(r_pcpf,rE,rP)
	
	# compute azimuth
	uN,uE,uD = get_unit_NED(lat,lon)
	vN = np.dot(v_pcpf,uN)	# velocity North (m/s)
	vE = np.dot(v_pcpf,uE)	# velocity East (m/s)
	az_pcpf = np.arctan2(vE,vN)	# planet-relative azimuth angle	
	
	# get atmospheric data (rho,p,T,winds)
	rho,p,T,winds = get_atm_data( in0.p, alt )
	eE = winds[0]	# zonal winds (east-west)
	eN = winds[1]	# meridional winds (north-south)
	eU = winds[2]	# vertical winds (up-down)
	
	# dynamic pressure calc
	wind_pcpf = eE*uE + eN*uN + (-eU)*nD	# wind (m/s), planet-relative. v_wind/planet
	v_pcpf_wr = v_pcpf - wind_pcpf	# v_sc/wind = v_sc/planet - v_wind/planet
	v_pcpf_wr_mag = norm(v_pcpf_wr)
	v_pcpf_wr_hat = v_pcpf_wr/v_pcpf_wr_mag	  # v_sc/wind unit vector
	q = 0.5 * rho * ( v_pcpf_wr_mag**2 )	# dynamic pressure (Pa)
	
	# mach number
	a = np.sqrt( gamma * R * T )	# speed of sound (m/s)
	mach = v_pcpf_mag / a			# mach number
	
	# c.g. location
	if ( in0.veh.cg_mode == 0 ): #constant cg
		cg = in0.veh.r_cg
	elif ( in0.veh.cg_mode == 1 ):	# linearly vary with q
		cg = in0.veh.r_cg			# for now
	else:
		cg = in0.veh.r_cg
	
	# gravity force calculation
	Fg_N_sq = mu*m*r_N/( rmag_N**3 )		# inverse-square gravity (2BP)
	Fg_j2_N = m * ( (3*J2*mu*(re**2) ) / ( 2*(rmag_N**5) )) * np.array([ \
	  ( ( (5*r_N[2]/(rmag_N**2)) - 1 )*( r_N[0] ) ), \	# I hat
	  ( ( (5*r_N[2]/(rmag_N**2)) - 1 )*( r_N[1] ) ), \	# J hat
	  ( r_N[2]*( (5*r_N[2]/(rmag_N**2)) - 3) ) \			# K hat
	  ])			# inertial frame (PCI) j2 force
	Fg_N = Fg_N_sq + Fg_j2_N	# total inertial gravitational force
	
	# aerodynamics
	n1 = skew(-h_pcpf_hat)
	rot1 = np.identity(3) + n1 + n1*n1
	n2 = skew(v_pcpf_wr_hat)
	rot2 = np.identity(3) + np.sin(veh.bank)*n2 + (1-np.cos(veh.bank))*n2*n2
	if ( in0.veh.aero.aero_mode == 0 ):		# constant cl, cd
		cd = in0.veh.aero.cd
		cl = in0.veh.aero.cl
	elif ( in0.veh.aero.aero_mode == 1 ):	# mach dependent table
		# interpolate between table values (table = [mach cl cd])
		aero = np.interp( in0.veh.aero.aero_table[:,0], in0.veh.aero.aero_table[:,1:3], mach )
		cl = aero[0]
		cd = aero[1]
		if ( np.isnan(cl) == 0 or np.isnan(cd) == 0 ):
			cl = 0
			cd = 0
	A_ref = np.pi * ( in0.veh.geometry.rc**2 )	# ref area = pi*r^2 (m^2)
	D_pcpf_hat = -v_pcpf_wr_hat		# drag is opposite velocity
	D_pcpf = q * cd * A_ref * D_pcpf_hat	# drag in planet-center, planet-fixed frame (PCPF)
	L_pcpf_hat = rot2 * rot1 * v_pcpf_wr_hat	# lift direction
	L_pcpf = q * cl * A_ref * L_pcpf_hat	# lift in PCPF coords
	D_N = np.dot(IP,D_pcpf)		# inertial drag force
	L_N = np.dot(IP,L_pcpf)		# inertial lift force
	
	# Propulsion
	T_N = np.array([0,0,0])			# initalize values
	T_pcpf = np.array([0,0,0])
	dm = 0
	if ( ctrl.s.ignition == True ):
		if ( in0.veh.prop.main.mode == 0 ):		# max thrust, gravity turn
			Tmag_pcpf = in0.veh.prop.main.Tmax
			T_pcpf = -Tmag_pcpf * v_pcpf_hat
			T_N = np.dot(IP,T_pcpf)
			dm = -Tmag_pcpf / ( in0.sim.earth_g * in0.veh.prop.main.isp )	# change in propellant mass
		#elif ( in0.veh.prop.main.mode == 1 ):	# time schedule thrust
		#	# to do
		
		# remove aerodynamic contributions (questionable assumption)
		D_N = np.array([0,0,0])
		D_pcpf = np.array([0,0,0])
		L_N = np.array([0,0,0])
		L_pcpf = np.array([0,0,0])
	
	# decelerator (parachute, IAD, etc) forces
	if ( ctrl.s.deploy_decel == True ):
		if ( in0.veh.decel.mode == 0 ):	# constant drag coeff.
			cd_decel = in0.veh.decel.cd
		elif ( in0.veh.decel.mode == 1 ):	# mach dependent
			cd_decel = np.interp( in0.veh.decel.table[:,0],in0.veh.decel.table[:,1], mach )
		else:
			cd_decel = 0
	else:
		cd_decel = 0
	A_decel = np.pi*( in0.veh.decel.diam / 2 )**2	# decel reference area
	Dmag_decel_pcpf = q * A_decel * cd_decel
	D_decel_pcpf = Dmag_decel_pcpf * (-v_pcpf_wr_hat)
	D_decel_N = np.dot(IP,D_decel_pcpf)		# inertial (PCI) decelerator drag force
	
	# calculate total force (inertial)
	F_N = D_N + L_N + D_decel_N + Fg_N + T_N		# drag + lift + decel_drag + gravity + thrust
	g_load = ( norm( F_N - Fg_N ) / m ) / in0.sim.earth_g	# g-loading (Earth G's)
	
	# calculate heat rate
	rn = in0.veh.geometry.rn	# vehicle nose radius
	qdot_s = in0.p.k * np.sqrt(rho) * vmag_pcpf**3	# heat rate (W/cm2)
	
	
	# create output data structure from class definition
	dat = define_dat(r_N, r_pcpf, lat, lon, az_pcpf, alt,
		  v_N, v_pcpf, fpa_N, fpa_pcpf, speed, v_pci_pcpf, v_wind, dv,
		  F_N, T_N, T_pcpf, Fg, fA_N, D_N, L_N, D_decel_N, g_load,
		  cl, cd, cd_decel, LD, q, mach, aoa, phi, ssa, A_ref,
		  m, cg, dm, m_prop,
		  rho, T, p, winds,
		  qdot_s,
		  xi)
	return dat
	
# trajectory calculation output structure
class define_dat:
	def __init__(self, r_N, r_pcpf, lat, lon, az_pcpf, alt,
		v_N, v_pcpf, fpa_N, fpa_pcpf, speed, v_pci_pcpf, v_wind, dv,
		F_N, T_N, T_pcpf, Fg, fA_N, D_N, L_N, D_decel_N, g_load,
		cl, cd, cd_decel, LD, q, mach, aoa, phi, ssa, A_ref,
		mass, cg, dm, m_prop,
		rho, T, p, winds,
		qdot_s,
		  xi):
		# [r,range], [v], [force], [aero], [mass], [atm], [heating], [energy]
		  
		self.r_N = r_N	# inertial position [3x1]  (m)
		self.r_pcpf = r_pcpf	# planet-relative position [3x1] (m)
		self.lat = lat		# latitude (rad)
		self.lon = lon		# longitude (rad)
		self.az_pcpf = az_pcpf	# planet-relative azimuth angle (rad)
		self.alt = alt		# altitude (m)
		self.v_N = v_N	# inertial velocity [3x1] (m/s)
		self.v_pcpf = v_pcpf	# planet-relative velocity [3x1] (m/s)
		self.fpa_N = fpa_N	# inertial flight path angle (rad)
		self.fpa_pcpf = fpa_pcpf	# planet-relative flight path angle (rad)
		self.speed = speed		# ground speed
		self.v_pci_pcpf = v_pci_pcpf	# planet-relative velocity in PCI frame [3x1] (m/s)
		self.v_wind = v_wind	# wind-relative velocity 
		self.dv = dv		# inertial change in velocity (instantaneous) PCI frame (m/s)
		self.F_N = F_N	# total inertial external force [3x1] (N)
		self.T_N = T_N		# total inertial thrust vector [3x1] (N)
		self.T_pcpf = T_pcpf		# total planet-relative thrust [3x1] (N)
		self.Fg = Fg			# gravitational force [3x1] (N)
		self.F_aero_N = fA_N		# total aerodynamic force [3x1] (drag + lift)
		self.D_N = D_N		# drag force vector [3x1] (N)
		self.L_N = L_N		# lift force vector [3x1] (N)
		self.D_decel_N = D_decel_N	# decelerator drag force [3x1] (N)
		self.g_load = g_load	# g-loading
		self.cl = cl			# lift coefficient
		self.cd = cd			# drag coefficient
		self.cd_decel = cd_decel	# decelerator drag coefficient
		self.LD = LD			# lift-to-drag ratio
		self.q = q				# dynamic pressure (Pa)
		self.mach = mach			# mach number
		self.aoa = aoa			# angle of attack (rad)
		self.phi = phi			# bank angle (rad)
		self.ssa				# sideslip angle (rad)
		self.A_ref = A_ref	 	# aerodynamic reference area (m2)
		self.mass = mass		# vehicle mass (kg)
		self.cg = cg			# cg position [3x1] (m)
		self.del_mass = dm		# instantaneous change in mass (kg)
		self.m_prop = m_prop		# propellant mass (kg)
		self.rho = rho			# atmospheric density at current altitude (kg/m3)
		self.temp = T			# atmospheric temp (K)
		self.pressure = p		# atmospheric pressure (Pa)
		self.winds = winds		# atmospheric winds [3x1] (m/s) - [zonal (E-W), meridional (N-S), vertical (up)]
		self.qdot_s = qdot_s	# heat rate (W/cm2)
		self.xi = xi			# energy	
	
# define norm
def norm(x):
	x = np.linalg.norm(x)
	return x
	
# define skew-symmetrix matrix	
def skew(x):
	A = np.array([[0,-x[2],x[1]],[x[2],0,-x[0]],[-x[1],x[0],0]])
	return A
