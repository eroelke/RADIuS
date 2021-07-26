# init_structs.py
#	define state structures for trajectory integration (separate from input structs)
#
#
#
#
#
#
#

from calc_lat_lon_alt import get_lat_lon


### vehicle initialization ###
def init_veh(in0):
	veh = veh_struct(in0.traj.phi0,
	  in0.traj.aoa0,
	  in0.traj.ssa,
	  in0.veh.geometry.ref_area,
	  in0.veh.m0)

	return veh

### guidance initialization ###
def init_guid(in0):
	
	
	
	
	return guid

	
### navigation initialization ###
def init_nav(in0,tvec):
	mode = in0.nav.mode	# navigation mode
	tau = in0.nav.tau
	r_pci = in0.traj.r0_pci
	v_pci = in0.traj.v0_pci
	r_pcpf = np.zeros((3,1))
	v_pcpf = np.zeros((3,1))
	a_sens_pci = np.zeros((3,1))
	a_sens_pcpf = np.zeros((3,1))
	lat_gd,lon,alt = get_lat_lon(r_pcpf,in0.p.rE,in0.p.rP)
	alt = 0
	t = in0.traj.t0
	
	tlen = np.linalg.norm(tvec)
	if ( mode == 1 ):
		dt = 1/in0.sim.traj_rate
		rnd = np.matrix( np.random.randn(9,tlen) )
		x_ercv = np.dot( (1 - np.exp(-2*dt/tau) ),rnd[:,0] )
		rva_err = np.dot( in0.nav.P_SS, x_ercv )
	else:
		rnd = np.zeros((9,tlen))
		x_ercv = np.zeros((9,1))
		rva_err = np.zeros((9,1))
	# add sensor error
	r_pci += rva_err[0:3]
	v_pci += rva_err[3:6]
	a_sens_pci += rva_err[6:9]
		
	state = nav_struct(r_pci,r_pcpf,v_pci,v_pcpf,a_sens_pci,
	  a_sens_pcpf,lat_gd,lon,alt,t,rnd,x_ercv,rva_err)
	return nav

### control initialization ###
def init_ctrl(in0):
	ignition = False
	throttle = 1
	jflag = False
	decel_flag = False
	aoa_rate = in0.traj.aoa0	# initial aoa
	aoa_accel = 0
	bank_rate = in0.traj.phi0	# initial bank angle
	bank_accel = 0
	A_ref = in0.veh.geometry.ref_area
	t = in0.traj.t0	
	ctrl = ctrl_struct(aoa_rate,aoa_accel,bank_rate,bank_accel,
	  A_ref,ignition,throttle,jflag,decel_flag,t)
	return ctrl

### structures for integration ###
class veh_struct:
	def __init__(self,bank,aoa,ssa,A_ref,m):
		self.phi = bank
		self.aoa = aoa
		self.ssa = ssa
		self.A_ref = A_ref
		self.m = m
		
# integration navigation structure
class nav_struct:
	def __init__(self,mode,seed,tau,r0_pci,r_pcpf,v_pci,v_pcpf,a_sens_pci,a_sens_pcpf,
	  lat_gd,lon,alt,dt,sn_ratio,t,t0,x_ercv,rva_err):
		self.mode = mode
		self.seed = seed
		self.tau = tau
		self.r0_pci = r0_pci
		self.r_pcpf = r_pcpf
		self.v_pci = v_pci			# inertial velocity
		self.v_pcpf = v_pcpf		# planet-relative velocity in pcpf frame
		self.a_sens_pci = a_sens_pci
		self.a_sens_pcpf = a_sens_pcpf
		self.lat_gd = lat_gd
		self.lon = lon
		self.dt = dt
		self.sn_ratio = sn_ratio
		self.t = t
		self.t0 = t0
		self.x_ercv = x_ercv
		self.rva_err = rva_err

# integration control structure
class ctrl_struct:
	def __init__(self,aoa_rate,aoa_accel,bank_rate,bank_accel,A_ref,ignition,throttle,jflag,decel_flag,t):
		self.aoa_rate = aoa_rate
		self.aoa_accel = aoa_accel
		self.bank_rate = bank_rate
		self.bank_accel = bank_accel
		self.A_ref = A_ref
		self.ignition = ignition
		self.throttle = throttle
		self.jflag = jflag
		self.decel_flag = decel_flag
		self.t = t
		
		
		
### guidance structures ###
class mej_struct:			# multi-event jettison
	def __init__(self,jflags,stage,a_mag,K_dens,Aref,mass,tj_curr,tj_next,ra,ra_err,ra_err_opt,bank):
		self.jflags = jflags
		self.stage = stage
		self.a_mag = a_mag
		self.K_dens = K_dens
		self.Aref = Aref
		self.mass = mass
		self.tj_curr = tj_curr
		self.tj_next = tj_next
		self.ra = ra
		self.ra_err = ra_err
		self.ra_err_opt = ra_err_opt
		self.bank = bank

	
	
	
	
	
	
	