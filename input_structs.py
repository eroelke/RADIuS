# input_structs.py
#
#	Define all input data structures (classes) and defining functions
#
#
#

import numpy as np
import scipy.io as import_mat

### Overall Input Struct ###
class define_in0:
	def __init__(self,veh,guid,nav,ctrl,traj,sim,p):
		self.veh = veh		# vehicle input data structure
		self.guid = guid	# guidance input data structure
		self.ctrl = ctrl	# control input data structure
		self.nav = nav		# navigation input data structure
		self.traj = traj	# traj input data structure
		self.sim = sim		# simulation settings (rates, termination conditions)
		self.p = p			# planet input data structure
		
### simulation data structure ###
class define_sim:
	def __init__(self,traj_rate,data_rate,guid_rate,ctrl_rate,nav_rate,term):
		self.traj_rate = traj_rate
		self.data_rate = data_rate
		self.guid_rate = guid_rate
		self.ctrl_rate = ctrl_rate
		self.nav_rate = nav_rate
		self.terminate = term
		self.earth_g = 9.80665	# gravitational accel of earth (m/s2)
		
class define_terminate:
	def __init__(self,val,term_dir,term_var):
		self.val = val			# termination condition value
		self.dir = term_dir	# termination direction (+- 1)
		self.var = term_var	# termination variable to check
			# 0 = time, t
			# 1 = altitude, h
			# 2 = planet-relative velocity, vmag_pp
			# 3 = energy, xi
			
### trajectory substructure ###
class define_traj:
	def __init__(self,
	  r0_pci, v0_pci, v0_pcpf, fpa0_pci, lat0, lon0, az0		# state vector
	  t0,t_max,						# time
	  aoa0, phi0, ssa0,				# vehicle attitude
	  dr0, cr0,						# down/cross range
	  ):
		self.r0_pci = r0_pci	# planet-centered inertial (PCI) position (m)
		self.r0_pcpf = r0.pcpf	# planet-centered, planet-fixed initial position (m)
		self.v0_pci = v0_pci	# PCI velociy (m/s)
		self.v0_pcpf = v0_pcpf	# planet-centered, planet-fixed initial velocity (m/s)
		self.fpa0 = fpa0		# initial flight path angle (rad)
		self.lat0 = lat0		# initial latitude (rad)
		self.lon0 = lon0		# initial longitude (rad)
		self.az0 = az0			# initial azimuth (rad) - angular distance along horizon measured cw from North
		self.t0 = t0			# initial time (s)
		self.t_max = t_max		# maximum integration time (s)
		self.aoa0 = aoa0		# initial angle of attack (rad)
		self.phi0 = phi0		# initial bank angle (rad)
		self.ssa0 = ssa0		# initial side slip angle (rad)
		self.dr0 = dr0			# initial downrange (m)
		self.cr0 = cr0			# initial crossrange (m)	
		
### vehicle substructure ###		
class define_veh:
	def __init__(self,geometry,aero,prop,mass,decel):
		self.geometry = geometry	# vehicle geometry
		self.aero = aero		# vehicle aerodynamics
		self.prop = prop		# thrust capabilities (engine, rcs)
		self.m0 = mass			# vehicle initial mass
		self.decel = decel
		
class define_prop_system:
	def __init__(self,main,rcs):
		self.main = main		# main engine propellant system
		self.rcs = rcs			# rcs systems propellant system (bank angle steering/modulation)
	
class define_prop:
	def __init__(self,isp,T_max,m0,mode,sched):
		self.isp = isp		# specific impulse (s)
		self.Tmax = T_max	# maximum engine thrust (N)
		self.m_prop = m0	# initial propellant mass (kg)
		self.mode = mode	# thrust mode (0 = max(gravity turn), 1 = schedule)
		self.sched = sched	# thrust schedule (for mode=1)
		
class define_geometry:
	def __init__(self,rn,rc,delta,r_cg,area,chord,span,cg_mode):
		self.rn = rn		# nose radius (m)
		self.rc = rc		# backshell radius, (m)
		self.delta = delta	# half-cone angle (rad)
		self.r_cg = r_cg	# location of center of gravity [3x1] (m)
		self.ref_area = area	# reference area (m2)
		self.chord = chord		# chord length (m)
		self.span = span		# span length (m)
		self.cg_mode = cg_mode	# (0 = const. r_cg, 1 = linear variation with dynamic pressure) 
		
class define_aerodynamics:
	def __init__(self,cd,cl,aoa,r_cp,mode,aero_table):
		self.cd = cd	# drag coeff
		self.cl = cl	# lift coeff
		self.aoa = aoa	# angle of attack
		self.r_cp = r_cp	# location of center of pressure [3x1] (m)
		self.aero_mode = mode	# aerodynamics mode (0=const. cl/cd, 1=mach table as [mach cl cd])
		self.aero_table = table	# aerodynamics table (mach dependence)
		
class define_decelerator:
	def __init__(self,mode,D,cd,K,table):
		self.mode = mode	# decelerator mode (0=const. drag, 1=mach dependent, 2=mach dep. w/ dispersions)
		self.diam = D		# reference diameter (m)
		self.cd = cd		# decelerator drag coefficient
		self.K = K			#
		self.table = table	# mach-dependent drag coefficients
		
### guidance substructures ###
class define_guid:
	def __init__(self,guid_mode,guid_struct):
		self.mode = guid_mode			# guidance mode (for reference purposes) -> string
			# options:
				# mej_n : multi-event jettison, newton method
				# mej_bi: multi-event jettison, bisection method
				# ppj   : pure prediction jettison
				# dcfj	: deceleration curve fit, jettison
				# 
				# 
				# 
				# 
				# 
		self.guid_struct = guid_struct	# guidance algorithm-specific structure (defined by guid_mode)
	
class define_mej:		# multi-event jettison
	def __init__(self,p_model,masses,A_refs,ra_tgt,cds,cls,K_bounds,K_gain,mcflag,tj0,n_jett,rate,a_init,iter_max):
		self.p = p_model		# planet model
		self.masses = masses	# vehicle masses (kg)
		self.A_refs = A_refs	# vehicle reference areas (m2)
		self.ra_tgt = ra_tgt	#
		self.cds = cds
		self.cls = cls
		self.K_bounds = K_bounds
		self.K_gain = K_gain
		self.mcflag = mcflag
		self.tj0 = tj0
		self.n_jett = n_jett
		self.rate = rate
		self.a_init = a_init	# acceleration to determine when atmosphere 'starts'
		self.iter_max = iter_max	# number of iterations per guidance call
		
class define_p_model:			# define planet model for guidance algorithm
	def __init__(self,alt_bounds,rp,re,mu,j2,npole,omega,atm_table):
		self.alt_bounds = alt_bounds
		self.rp = rp
		self.re = re
		self.mu = mu
		self.j2 = j2
		self.npole = npole
		self.omega = omega
		self.atm_table = atm_table
		
### control input substructure ###
class define_ctrl:
	def __init__(self,s_aoa,s_bank,s_decel):
		self.s_aoa = s_aoa		# angle of attack control input structure
		self.s_bank = s_bank	# bank control input structure
		self.s_decel = s_decel	# decelerator control input structure

class define_ctrl_input:
	def __init__(self,mode,rate_lim,accel_lim):
		self.mode = mode
		self.rate_lim = rate_lim
		self.accel_lim = accel_lim
		
class define_ctrl_decel:
	def __init__(self,mode,var,value):
		self.mode = mode
		self.var = var		# variable to trigger decel
		self.value = value	# value to trigger decel deployment
		
### navigation substructures ###
class define_nav:
	def __init__(self,mode,seed,tau,P_SS):
		self.mode = mode	# (0 = perfect nav, 1 = sensor noise)
		self.seed = seed
		self.tau = tau
		self.P_SS = P_SS
	
### planet substructure ###
class define_planet:
	def __init__(self, p, r_e, r_p, r_m, mass, mu, g0, j2, k, omega, atm):
		self.planet = p		# planet string
		self.r_e = r_e		# equatorial radius, m
		self.r_p = r_p		# polar radius, m
		self.r_m = r_m		# mean radius, m
		self.mass = mass		# planet mass, kg
		self.mu = mu			# planet gravitational constant, m3/s2
		self.g0 = g0			# planet surface gravity, m/s2
		self.j2 = j2			# j2 harmonic
		self.k = k			# sutton-graves constant, kg^(0.5/m)
		self.omega = omega	# angular velocity
		self.atm = atm		# atmospheric data struct
		
class define_atm:
	def __init__(self, mode, R, gamma, H, p0, rho0, T0, atm_table):
		self.mode = mode
		self.R = R			# ideal gas constant (J/kg/K)
		self.gamma = gamma	# specific heat ratio
		self.H = H			# scale height (m)
		self.p0 = p0		# reference (surface) pressure (P)
		self.rho0 = rho0	# reference (surface) density (kg/m3)
		self.T0 = T0		# reference (surface) temp (K)
		atm.table = atm_table	# atmospheric table [alt, rho, pressure, temp, wind_zonal, wind_north, wind_vertical]
		
### class function calls ###
# overall input structure
def create_in(veh,guid,nav,ctrl,traj,sim,p):
	input = in0(veh,guid,nav,ctrl,traj,sim,p)
	return input
	
# vehicle funcs
def create_veh(geometry,aero,prop,mass,decel):
	veh = define_veh(geometry,aero,prop,mass,decel)
	return veh
def create_geom(rn,rc,delta,r_cg,area,chord,span,cg_mode):
	geom = define_geometry(rn,rc,delta,r_cg,area,chord,span,cg_mode)
	return geom
def create_aero(cd,cl,aoa,r_cp,mode,aero_table):
	aero = define_aero(cd,cl,aoa,r_cp,mode,aero_table)
	return aero
def create_prop_system(main,rcs):
	syst = define_prop_system(main,rcs)
	return syst
def create_prop(isp,T_max,m0,mode,sched):
	prop = define_prop(isp,T_max,m0,mode,sched)
	return prop
def create_decel(mode,D,cd,K,table):
	decel = define_decel(mode,D,cd,K,table)
	return decel
	
	
# guidance funcs
def create_guid(mode,struct):
	guid = define_guid(mode,struct)
	return guid
	
def create_mej(in0,h_bounds,masses,A_refs,ra_tgt,cds,cls,K_bounds,K_gain,mcflag,tj0,n_jett,rate,a_init):
	p_model = create_planet_model(h_bounds,in0)
	mej = define_mej(p_model,masses,A_refs,ra_tgt,cds,cls,K_bounds,K_gain,mcflag,tj0,n_jett,rate,a_init)
	return mej
	
def create_planet_model(h_bounds,in0):
	rp = in0.p.r_p
	re = in0.p.r_e
	mu = in0.p.mu
	j2 = in0.p.j2
	npole = np.array([0,0,1])
	omega = in0.p.omega
	atm_table = in0.p.atm.table		# nominal atmosphere table
	model = define_p_model(h_bounds,rp,re,mu,j2,npole,omega,atm_table)
	
# control functions
def create_ctrl(s_aoa,s_bank,s_decel):
	ctrl = define_ctrl(s_aoa,s_bank,s_decel)
	return ctrl
def create_aoa_ctrl(aoa_mode,aoa_rate_lim,aoa_accel_lim):
	s_aoa = define_ctrl_input(aoa_mode,aoa_rate_lim,aoa_accel_lim)
	return s_aoa
def create_bank_ctrl(bank_mode,bank_rate_lim,bank_accel_lim):
	s_bank = define_ctrl_input(bank_mode,bank_rate_lim,bank_accel_lim)
	return s_bank
def create_decel(decel_mode,decel_var,decel_val):
	s_decel = define_ctrl_decel(decel_mode,decel_var,decel_val)
	return s_decel
	
	
# navigation functions
def create_nav(mode,seed,tau,P_SS,state):
	nav = define_nav(mode,seed,tau,P_SS,state)
	return nav
def create_nav_state(r_N,r_pcpf,v_N,v_pcpf,a_sens_pci,a_sens_pcpf,lat_gd,lon,alt,dt,sn_ratio,t,t0,x_ercv,rva_err):
	state = define_nav_s(r_N,r_pcpf,v_N,v_pcpf,a_sens_pci,a_sens_pcpf,lat_gd,lon,alt,dt,sn_ratio,t,t0,x_ercv,rva_err)
	return state

# trajectory funcs
def create_traj(r0_pci, v0_pci, v0_pcpf, fpa0_pci, lat0, lon0, az0, t0,t_max, aoa0, phi0, ssa0, dr0, cr0):
	traj = define_traj(r0_pci, v0_pci, v0_pcpf, fpa0_pci, lat0, lon0, az0		# state vector
	  t0,t_max,						# time
	  aoa0, phi0, ssa0,				# vehicle attitude
	  dr0, cr0,						# down/cross range
	  )	
	return traj
# simulation settings func
def create_sim(traj_rate,data_rate,guid_rate,ctrl_rate,nav_rate,term):
	sim = define_sim(traj_rate,data_rate,guid_rate,ctrl_rate,nav_rate,term)
	return sim
def create_terminate():
	term = define_terminate(val,term_dir,term_var)
	return term
	
# planet func
def create_planet(p_str,atm_mode,file):
	atm_table = load_mat_table(file)
	if atm_mode > 3:
		atm_mode = 1
	if p_str.lower() == 'venus':	# venus
		v_atm = define_atm(atm_mode,188.92,1.2857,15900,65,9200000,100,atm_table)
		p = define_planet(1,6.0518e6,6.0518e6,6.0518e6,4.8685e24,3.24858592079e14,8.87,4.458e-6,1.896e-4,[0, 0, -2.9924205e-7],v_atm)
	elif p_str.lower() == 'earth':		# earth
		e_atm = define_atm(atm_mode,287.1,1.4005,8500,1.217,101400,300,atm_table)
		p = define_planet(2,6.3781e6,6.3568e6,6.3710e6,5.9736e24,3.9860e14,9.80665,1.0826e-3,1.7623e-4,[0, 0, 7.2921066e-5],e_atm)
	elif p_str.lower() == 'mars':		# mars
		m_atm = define_atm(atm_mode,188.92,1.2941,11100,0.02,636,273,atm_table)
		p = define_planet(3,3.3962e6,3.3762e6,3.3895e6,6.4185e23,4.2830e13,3.710,1.9605e-3,1.898e-4,[0, 0, 7.0882360e-5],m_atm)
	elif p_str.lower() == 'jupiter':
		j_atm = define_atm(atm_mode,3745.18,1.4348,27000,0.16,101400,100,atm_table)
		p = define_planet(4,7.1492e7,6.6854e7,6.9911e7,1.8986e27,1.2669e17,24.790,1.4736e-2,6.556e-5,[0, 0, 1.7585181e-4],j_atm)
	elif p_str.lower() == 'saturn':
		s_atm = define_atm(atm_mode,3892.46,1.3846,59500,0.19,101400,100,atm_table)
		p = define_planet(5,6.0268e7,5.4364e7,5.8232e7,5.6848e26,3.7931e16,10.440,1.6298e-2,6.356e-5,[0, 0, 1.6378841e-4],s_atm)
	elif p_str.lower() == 'titan':
		t_atm = define_atm(atm_mode,290.0,1.3846,40000,6.085,150000,100,atm_table)
		p = define_planet(6,2.5750e6,2.5750e6,2.5750e6,1.3455e23,8.9797e12,1.354,4e-5,1.7407e-4,[0, 0, 4.5606856e-6],t_atm)
	elif p_str.lower() == 'uranus':
		u_atm = define_atm(atm_mode,3614.91,1.3846,27700,0.42,101400,100,atm_table)
		p = define_planet(7,2.5559e7,2.4973e7,2.5362e7,8.6832e25,5.7940e15,8.870,3.3434e-3,6.645e-5,[0, 0, -1.0123720e-4],u_atm)
	elif p_str.lower() == 'neptune':
		n_atm = define_atm(atm_mode,3614.91,1.3846,19700,0.45,101400,100,atm_table)
		p = define_planet(8,2.4764e7,2.4341e7,2.4622e7,1.0243e26,6.8351e15,11.150,3.4110e-3,6.719e-5,[0, 0, 1.0833825e-4],n_atm)
	else:
		Print('Error: Planet data not available (yet)')
		exit()
	return p
	
	
	
# extra functions	
# load .mat table
def load_mat_table(file):
	mat = import_mat.loadmat(file) 
	table = mat['table']
	return table	
	
	
#testing	
#vehicle = veh(10,0,0,0,0,False)
#venus = define_planet('venus',1,'data/atm_venus_JPL.mat')
#print(venus.planet)




