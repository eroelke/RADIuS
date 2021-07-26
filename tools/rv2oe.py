# rv->oe
	# compure orbital elements given r,v in ECI frame
		# a: semi-major axis (m)
		# e: eccentricity
		# i: inclination (rad)
		# W: longitude of ascending node (LAN) (rad)
		# w: argument of periapsis (AOP) (rad)
		# f: true anomaly at epoch (rad)
	# created Sep 2018, E. Roelke
	# Colorado Center for Astrodynamics Research
	# CU Boulder
def rv2oe(rv,mu):
	pi = numpy.pi

	r = rv[0:3]		# position vector in ECI frame (m)
	v = rv[3:6]		# velocity vector in ECI frame (m/s)
	rmag = numpy.linalg.norm(r)
	vmag = numpy.linalg.norm(v)
	
	a = rmag/(2 - ( rmag*(vmag^2)/mu )	# semi-major axis
	h = numpy.cross(r, v)				# angular momentum
	hmag = numpy.linalg.norm(h)
	evec = cross(v,h)/mu - r/rmag		# eccentricity vector
	e = numpy.linalg.norm(evec)		# eccentricity
	khat = [0,0,1]					# ECI frame vertical unit vector
	n = numpy.cross(khat,h/hmag)	# line of nodes (unit vector)
	nmag = numpy.linalg.norm(n)
	i = numpy.arccos( h[2]/hmag )	# inclination = acos(h.khat/hmag)
	W = numpy.arccos( n[0]/nmag )	# LAN = acos(n.ihat/nmag)
	w = numpy.arccos( numpy.dot(n,e)/(nmag*e) )
	f = numpy.arccos( numpy.dot(e,r)/(rmag*e) )
	
	# test quadrants for LAN
	if ( n[1] > 0 and W > pi ):
		W = (2*pi) - W
	elif ( n[1] < 0 and W < pi ):
		W = (2*pi) - W
	
	# test quadrants for AOP
	if ( e[2] > 0 and w > pi ):
		w = (2*pi) - w
	elif ( e[2] < 0 and w < pi ):
		w = (2*pi) - w
	
	# test quadrants for f
	if ( numpy.dot(r,v) > 0 and f > pi ):
		f = (2*pi) - f
	elif ( numpy.dot(r,v) < 0 and f < pi ):
		f = (2*pi) - f
	
	oe = [a,e,i,W,w,f]	# orbital elements
	return oe
	