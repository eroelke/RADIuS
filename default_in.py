# default_in.py
#	define default input data structure
#		planet: Earth, atm_mode = 1
#		vehicle: 45deg sphere cone, 2m backshell radius, 0.5m nose radius, 300kg
#		aero: cd = 1.05, cl = 0, aoa = 0, ssa = 0, aero_mode = 0 (const. cl/cd)
#		x0: h_atm = 150km, v_atm = 10km/s, fpa_atm = -5deg, [lat,lon,az_atm] = [0,0,90]
#		guid: none
#		nav: perfect knowledge
#		ctrl: none

from input_structs import *

def default_in:
	# simulation data structure
	term_val = 0			# hit ground
	term_dir = -1			# negative crossing
	term_var = uint8(1)		# altitude crossing
	term = create_terminate(term_val,term_dir,term_var)
	traj_rate = 10	# Hz, rate of integration
	data_rate = 1	# Hz, rate of data storage
	guid_rate = 0.5	# Hz, rate of guidance call
	ctrl_rate = 0.5	# Hz, rate of control call
	nav_rate = 10	# Hz, rate of navigation call
	sim = create_sim(traj_rate,data_rate,guid_rate,ctrl_rate,nav_rate,term)

	# vehicle struct
	rc = 2		#m, backshell radius
	rn = 0.5	#m, nose radius
	delta = deg2rad(45)	# half-cone angle
	r_cg = np.array([0,0,0])
	area = np.pi * (rc**2)
	chord = 0
	span = 0
	cg_mode = 0	# constant center of gravity
	geom = create_geom(rn,rc,delta,r_cg,area,chord,span,cg_mode)
	prop_main = create_prop(isp,T_max,m0,mode,sched)
	prop_rcs = create_prop(isp,T_max,m0,mode,sched)
	prop = create_prop_system(prop_main,prop_rcs)
	mass = 300	# kg
	cd = 1.05
	cl = 0
	aoa = 0
	r_cp = np.array([0,0,0])
	aero_mode = 0	# constant cl/cd
	aero_table = 0
	aero = create_aero(cd,cl,aoa,r_cp,aero_mode,aero_table)
	decel_mode = 0	# constant cd
	decel_D = 0		# no decel
	decel_cd = 0
	decel_K = 0
	decel_table = 0
	decel = create_decel(decel_mode,decel_D,decel_cd,decel_K,decel_table)
	veh = create_veh(geom,aero,prop,mass,decel)	
	
	# guidance nominal struct
	guid = create_guid()
	
	# nav nominal struct
	nav = create_nav()
	
	# control nominal struct
	ctrl = create_control()
	
	# define trajectoy
	alt0 = 150e3	# m
	v0_pcpf = 10e3	# m/s
	lat0 = 0
	lon0 = 0
	az0 = 90*np.pi/180
	t0 = 0
	t_max = 1000
	aoa0 = 0
	phi0 = 0
	ssa0 = 0
	dr0 = 0
	cr0 = 0
	traj = create_traj(r0_pci, v0_pci, v0_pcpf, fpa0_pci, lat0, lon0, az0, t0,t_max, aoa0, phi0, ssa0, dr0, cr0)
	
	# planet inputs
	planet = create_planet('earth', 0, './data/earth_atm.mat')
	
	in = create_in(veh,guid,nav,ctrl,traj,sim,planet)
	
	
def deg2rad(q):
	rad = q * np.pi/180
	return rad