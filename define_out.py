# define_out.py
# 	-define output structure(s)
#	-array sizes based on max time length allocated in input structure
#
#

def create_output_struct( in0, tvec ):
	
	# define data type arrays
	N = len(tvec)+1	# max simulation length (accounts for final data point)
	N1 = [np.nan] * N
	N3 = np.matrix( [np.nan, np.nan, np.nan] * N )
	N5 = np.matrix( [np.nan, np.nan, np.nan,
	  np.nan, np.nan] * N )
	N9 = np.matrix( [np.nan, np.nan, np.nan, 
	  np.nan, np.nan, np.nan,
	  np.nan, np.nan, np.nan] * N )
	veh_out = veh_out_struct(N1,N1,N1,N1,N1,N1,N1,N1,N1,N1,N1,N1)
	
	traj_out = traj_out_struct( N1,N3,N1,N3,N1,N3,N1,N3,N1,
	  N1,N1,N1,N1,N1,N1,N1,N1,N1,N1,N1,N1,N1 )
	
	ctrl_out = ctrl_out_struct( N1,N5,N1,N1,N1,N1,N1,N1 )
	nav_out = nav_out_struct( N3,N3,N3,N3,N3,N3,N9,N1,N1,N1,N1 )
	
	# get guidance struct
	if ( in0.guid.mode == 'mej_n' ):
		guid_out = mej_n_out_struct()
	elif ( in0.guid.mode == 'mej_bi' ):
		guid_out = mej_bi_out_struct()
	#elif ():
	#elif ():
	#elif ():
	#elif ():
	else:
		guid_out = ''
	
	out = out_struct(traj_out,veh_out,guid_out,ctrl_out,nav_out)
	return out


	
# classes #
class out_struct:
	def __init__(self,traj_out,veh_out,guid_out,ctrl_out,nav_out):
		self.traj = traj_out
		self.v = veh_out
		self.g = guid_out
		self.c = ctrl_out
		self.n = nav_out
	 
# trajectory output struct	  
class traj_out_struct:
	def __init__(self,t,r_pci,rmag_pci,v_pci,vmag_pci,r_pcpf,rmag_pcpf,v_pcpf
	  vmag_pcpf,alt,rho,azi_pcpf,q,fpa_pci,fpa_pcpf,n,mach,
	  dr,cr,qdot_s,lat,lon):
		self.t = t				# time (s)
		self.r_pci = r_pci		# position, pci frame (m)
		self.rmag_pci = rmag_pci	# position mag, pci frame (m)	
		self.v_pci = v_pci			# velocity, pci frame (m/s)
		self.vmag_pci = vmag_pci	# velocity mag, pci frame (m/s)
		self.r_pcpf = r_pcpf		# position, pcpf frame (m)
		self.rmag_pcpf = rmag_pcpf	# position mag, pcpf frame (m)
		self.v_pcpf = v_pcpf		# velocity, pci frame (m/s)
		self.vmag_pcpf = vmag_pcpf	# velocity mag, pcpf frame (m/s)
		self.alt = alt			# altitude (m)
		self.rho = rho			# atmospheric density (kg/m3)
		self.azi_pcpf = azi_pcpf	# azimuth angle, pcpf frame (rad)
		self.q = q				# dynamic pressure (wind-relative) (Pa)
		self.fpa_pci = fpa_pci		# flight path angle, PCI frame (rad)
		self.fpa_pcpf = fpa_pcpf	# flight path angle, PCPF frame (rad)
		self.n = n					# g-loading (earth Gs)
		self.mach = mach		# mach number
		self.dr = dr			# downrange (m)
		self.cr = cr			# crossrange (m)
		self.qdot_s = qdot_s	# heat rate (W/m2)
		self.lat = lat			# latitude (rad)
		self.lon = lon			# longitude (rad)
		
# vehicle output struct
class veh_out_struct:
	def __init__(self,mass,m_prop,area,cl,cd,LD,aoa,phi,
	  phi_rate,phi_accel,ssa,r_cg,
	  Fg_pci,Fg_pcpf,D_pci,Dmag_pci,L_pci,Lmag_pci,D_decel):
		self.mass = mass		# vehicle total mass (kg)
		self.m_prop = m_prop	# propellant mass (kg)
		self.area = area		# reference area (m2)
		self.cl = cl		# lift coefficient
		self.cd = cd		# drag coefficient
		self.LD = LD		#lift-to-drag ratio
		self.aoa = aoa			# angle of attack (rad)
		self.phi = phi			# bank angle (rad)
		self.phi_rate = phi_rate	# bank rate (rad/s)
		self.phi_accel = phi_accel	# bank accel (rad/s2)
		self.ssa = ssa			# sideslip angle (rad)
		self.r_cg = r_cg		# center of gravity pos (m)
		self.Fg_pci = Fg_pci	# gravity force, pci frame (N)
		self.Fg_pcpf = Fg_pcpf	# gravity force, pcpf frame (N)
		self.D_pci = D_pci		# drag force, pci frame (N)
		self.Dmag_pci = Dmag_pci	# drag force mag, pci frame (N)
		self.L_pci = L_pci		# lift force, pci frame (N)
		self.Lmag_pci = Lmag_pci	# lift force mag, pci frame (N)
		self.D_decel = D_decel		# decelerator drag force (N)
	
# guidance output structs
class mej_n_out_struct:
	def __init__(self,jflags,

# control output struct
class ctrl_out_struct:
	def __init__(self,deploy_decel,jettison,cmd_aoa,cmd_aoa_rate,cmd_aoa_accel,
	  cmd_bank,cmd_bank_rate,cmd_bank_accel):
		self.deploy_decel = deploy_decel
		self.jettison = jettison
		self.aoa = cmd_aoa
		self.aoa_rate = cmd_aoa_rate
		self.aoa_accel = cmd_aoa_accel
		self.bank = cmd_bank
		self.bank_rate = cmd_bank_rate
		self.bank_accel = cmd_bank_accel

# navigation output struct
class nav_out_struct:
	def __init__(self,r_pci,r_pcpf,v_pci,v_pcpf,a_sens_pci,a_sens_pcpf,rva_err,
	  lat_gd,lon,alt,t):
		self.r_pci = r_pci
		self.r_pcpf = r_pcpf
		self.v_pci = v_pci
		self.v_pcpf = v_pcpf
		self.a_sens_pci = a_sens_pci
		self.a_sens_pcpf = a_sens_pcpf
		self.rva_err = rva_err
		self.lat_gd = lat_gd
		self.lon = lon
		self.alt = alt
		self.t = t

	
	
	
	
	
	
	
	
	
	
	