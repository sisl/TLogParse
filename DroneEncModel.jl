#push!(LOAD_PATH, "/home/emueller/.julia/v0.3/GridInterpolations/src")
using PyPlot, BayesNets, JLD, HDF5
#include("/home/emueller/.julia/v0.3/BayesNets/src/BayesNets.jl")
include("/home/emueller/.julia/v0.3/droneencountermodel/src/TLogProcessor.jl")
#include("typesCA2DRP.jl")
#include("CA2DRP.jl")      # Need this for getReward, getNoiseSample, etc.  Might want to just rewrite them here and not link that stuff in.

# Usage:
# 1. Create a set of batch trajectories, optionally specifying the bayes nets and other parameters.  You may also create a filter function
# 	 that rejects trajectories not meeting your criteria (e.g. not moving fast enough):
# iES=createBatchTrajectories();  
# 2. Convert the encounter "set" (which is a single trajectory), into a set of trajectories rotated and shifted to create a set of trajectories
#    suitable for creating collision hazards with the ownship.
# itdb=convertEncSet2TrajDatabase(iES[1]);
# 3. Visualize the encounters: 
# plotEncounterSet(iES[1])
# plotIntruderDataBase(itdb)
# 4. Evaluate the algorithm(s) against these intruder trajectory data bases:
# tbd

# Need to worry about units here, and make sure that I'm consistent with the encounter model sampling.  Perhaps keep this
# in my "generic" units, but do the conversion when sampling from the encounter model.
type IntruderState
	altAGL::Float64
	altdot::Float64
	heading::Float64
	rrHeading::Float64
	rrHeaddot::Float64
	range::Float64
	vel::Float64   		# Note that this variable corresponds to dxdt in the BNs and the feature name
	vdot::Float64  		# Note that this variable corresponds to d2xdt2 in the BNs and the feature name
	x::Float64
	y::Float64
	x0::Float64
	y0::Float64
	xdd::Float64
	ydd::Float64
	t::Float64

	IntruderState(altAGL, altdot, heading, rrHeading, rrHeaddot, range, vel, vdot, x, y, x0, y0, xdd, ydd, t) = 
			  new(altAGL, altdot, heading, rrHeading, rrHeaddot, range, vel, vdot, x, y, x0, y0, xdd, ydd, t)
end

function IntruderState(;
	altAGL::Float64=0., 
	altdot::Float64=0., 
	heading::Float64=0.,
	rrHeading::Float64=0.,
	rrHeaddot::Float64=0.,
	range::Float64=0.,
	vel::Float64=0.,
	vdot::Float64=0.,
	x::Float64=0.,
	y::Float64=0.,
	x0::Float64=0.,
	y0::Float64=0.,
	xdd::Float64=0.,
	ydd::Float64=0.,
	t::Float64=0.)

	IntruderState(altAGL, altdot, heading, rrHeading, rrHeaddot, range, vel, vdot, x, y, x0, y0, xdd, ydd, t)
end

type IntruderEncounterSet

	iStateHist::Array{IntruderState}
	cpaLocs::Array{Float64}
	tCPAinit::Array{Float64}  	# The amount of time before the cpaLocations that the aircraft should start at (determines where in iStateHist we can select CPA locations)
	encAngles::Array{Array{}, 1}
	tCPAtraj::Array{Float64}    # The times along the intruder trajectory at which collisions have been selected to occur.  Corresponds to the t field in IntruderState()

	IntruderEncounterSet(iStateHist, cpaLocs, tCPAinit, encAngles, tCPAtraj) = 
				     new(iStateHist, cpaLocs, tCPAinit, encAngles, tCPAtraj)
end

function IntruderEncounterSet(;
	iStateHist::Array{IntruderState}=IntruderState[],
	cpaLocs::Array{Float64}=Float64[],
	tCPAinit::Array{Float64}=Float64[],
	encAngles::Array{Array{Float64,1},1}=Array[],
	tCPAtraj::Array{Float64}=Array[])

	IntruderEncounterSet(iStateHist, cpaLocs, tCPAinit, encAngles, tCPAtraj)
end

type IntruderTrajectory
	# These are all intruder locations, except for the ownship velocities that were used to 
	# rotate the intruder trajectory into the appropriate frame.  The ownship starting location
	# may be easily calculated as x0 = xCPA-voxI*tCPA, y0=yCPA-voyI*tCPA

	x::Array{Float64}
	y::Array{Float64} 
	h::Array{Float64}
	t::Array{Float64}
	encAng::Float64
	tCPA::Float64
	xCPA::Float64
	yCPA::Float64
	voxI::Float64
	voyI::Float64

	IntruderTrajectory(x,y,h,t,encAng,tCPA,xCPA,yCPA,voxI,voyI) = new(x,y,h,t,encAng,tCPA,xCPA,yCPA,voxI,voyI)
	
end

function loadBNs(;tName::AbstractString="/home/emueller/.julia/v0.3/droneencountermodel/data/TransNetworkNoDepend.xdsl", 
				  iName::AbstractString="/home/emueller/.julia/v0.3/droneencountermodel/data/InitNetwork.xdsl")
# iBN,tBN = loadBNs()

	splitext(tName)[2] == ".xdsl" || error("Bayes nets must be in .xdsl format")
	splitext(iName)[2] == ".xdsl" || error("Bayes nets must be in .xdsl format")

	iBN=readxdsl(iName)
	tBN=readxdsl(tName)

	return iBN, tBN
end

function noiTrajSelCriteria(ie::IntruderEncounterSet)
# This function always returns true, it does not filter any intruder trajectories out and is just a placeholder for the default
# filtering criteria for createBatchTrajectories.
	return true
end

function velrangeTrajSelCriteria(encSet::IntruderEncounterSet; maxV::Float64=340., minV::Float64=1.69)
# This function always returns true, it does not filter any intruder trajectories out and is just a placeholder for the default
# filtering criteria for createBatchTrajectories.
	#maxV = 200 * 6071/3600   # 200 kts converted to ft/s
	#minV = 1 * 6071/3600
	
	# if the CPA locations are (nearly) on top of each other, immediately reject the trajectory because it's not moving fast enough:

	tHist = Float64[encSet.iStateHist[i].t for i=1:length(encSet.iStateHist)]
	xHist = Float64[encSet.iStateHist[i].x for i=1:length(encSet.iStateHist)]
	yHist = Float64[encSet.iStateHist[i].y for i=1:length(encSet.iStateHist)] 
	hHist = Float64[encSet.iStateHist[i].altAGL for i=1:length(encSet.iStateHist)]
	vHist = Float64[encSet.iStateHist[i].vel for i=1:length(encSet.iStateHist)]

	# Just check the velocity at the CPA point:
	for i=1:size(encSet.cpaLocs,1)
		minDT, trajCPAInd = findmin(abs(tHist-encSet.tCPAtraj[i]))
		if (vHist[trajCPAInd]>maxV) || (vHist[trajCPAInd]<minV) 
			return false
		end
	end
	return true

end
function createBatchTrajectories(;tName::AbstractString="../data/TransNetworkNoDepend.xdsl", iName::AbstractString="../data/InitNetwork.xdsl",
								  numEncSets::Integer=100, saveTrajFile::AbstractString="", tCPAInit::Array{Float64}=[15.], 
								  encAngles::Array{Array{}}=Array[45.:45.:315.], psiTol::Float64=1e-6, psidotTol::Float64=1e-8, 
								  maxIters::Int64=20, VERBOSE::Bool=false, dt::Float64=0.2, tMax::Float64=120., bnp::BNParams=BNParams(), 
								  x0::Float64=0., y0::Float64=0., bearingInit::Float64=0., initEvidenceVal::Assignment=Assignment(), 
								  maxAttempts::Array{Int64}=[10000,100], t0::Float64=0., numCPAsPerTraj::Integer=1, minRolloutTime::Real=15., 
								  iTrajValid::Function=noiTrajSelCriteria)
# 	# This function returns a set of trajectories and ownship starting locations that can be used to test and compare a set
# 	# of algorithms.  It can/will be callable from my algorithm simulation functions.
	# iES=createBatchTrajectories(numEncSets=1);
	# iES=createBatchTrajectories(numEncSets=1, iTrajValid=x->velrangeTrajSelCriteria(x,maxV=10., minV=1.));


	iES = Array(IntruderEncounterSet,numEncSets)
	iBN,tBN = loadBNs()

	maxTcpa = maximum(tCPAInit)   # Will use this to select CPA locations that aren't so close together that we're reusing the same intruder trajectory portions
	numTselected = 0
	numAttempts = 0
	while (numTselected<numEncSets) && (numAttempts<100*numEncSets)
		# Select the locations for CPA randomly along the intruder's trajectory
		cpasSelected = 0
		attempts = 0
		tCPAmat = -100*maxTcpa*ones(Float64,numCPAsPerTraj)  # Initialize them to something far outside the required tCPA val so it doesn't affect the following check
		while (cpasSelected<numCPAsPerTraj) && (attempts < 100*numCPAsPerTraj)
			tCPAcandidate = (tMax-(minRolloutTime+maxTcpa))*rand()+maxTcpa
			if !in(true,abs(tCPAcandidate-tCPAmat).<maxTcpa)
				cpasSelected+=1
				tCPAmat[cpasSelected] = tCPAcandidate
			end
			attempts+=1
		end
		if cpasSelected<numCPAsPerTraj
			warn("number of cpas selected for trajectory $(numTselected+1) ($cpasSelected) is less than the number requested ($numCPAsPerTraj)")
		end
		tCPAmat = tCPAmat[1:cpasSelected]

		stHist = simulateIntruderTraj(iBN, tBN, psiTol=psiTol, psidotTol=psidotTol, maxIters=maxIters, 
								  VERBOSE=VERBOSE, dt=dt, tMax=tMax, bnp=bnp, x0=x0, y0=y0, bearingInit=bearingInit, 
								  initEvidenceVal=initEvidenceVal, maxAttempts=maxAttempts, t0=t0)

		cpaLocs = Array(Float64, length(tCPAmat), 2)
		tCPAmatrnd = Array(Float64,length(tCPAmat))
		for (j,tCPA) in enumerate(tCPAmat)
			tInd = round(Int64,tCPA/dt)   # Approximation of the closest time index to the selected tCPA
			cpaLocs[j,:] = [stHist[tInd].x, stHist[tInd].y]
			tCPAmatrnd[j] = stHist[tInd].t
		end

		ie = IntruderEncounterSet(stHist, cpaLocs, tCPAInit, encAngles, tCPAmatrnd)
		if iTrajValid(ie)
			numTselected+=1
			iES[numTselected]=ie			
		end
		numAttempts+=1
	end

	if numTselected<numEncSets
		warn("number of trajectories returned ($numTselected) is less than the number requested ($numEncSets)")
	end

	if !isempty(saveTrajFile)
		JLD.save(saveTrajFile, "iES", iES)
	end

	return iES

end

function rotateAndShiftTraj(encSet::IntruderEncounterSet, rotAng::Float64, dx::Float64, dy::Float64, trajStartInd::Int64, 
							cpaInd::Int64, tShift::Float64)

	# Remove the states before the initial time-to-CPA point.  We want our trajectories to start then, not before.
	tNew = [encSet.iStateHist[i].t for i=trajStartInd:length(encSet.iStateHist)] - tShift
	xHist = [encSet.iStateHist[i].x for i=trajStartInd:length(encSet.iStateHist)] - dx
	yHist = [encSet.iStateHist[i].y for i=trajStartInd:length(encSet.iStateHist)] - dy
	hHist = [encSet.iStateHist[i].altAGL for i=trajStartInd:length(encSet.iStateHist)]

	xyHist = [cos(rotAng) -sin(rotAng); sin(rotAng) cos(rotAng)] * [xHist yHist]'  # Rotates a point counter clockwise, preserving the coordinate frame
	xNew = xyHist[1,:]'
	yNew = xyHist[2,:]'

	hNew = hHist.-minimum([0.; hHist])    # Shift the altitudes.  If all altitudes are positive, don't shift

	return xNew, yNew, hNew, tNew
end


function convertEncSet2TrajDatabase(encSetArr::Array{IntruderEncounterSet}; voxI::Float64=1., voyI::Float64=0., xCPA::Float64=0., yCPA::Float64=0.)

	itdb=IntruderTrajectory[]
	for i=1:length(encSetArr)
		itdb = [itdb; convertEncSet2TrajDatabase(encSetArr[i], voxI=voxI, voyI=voyI, xCPA=xCPA, yCPA=yCPA)]
	end

	return itdb
end

function convertEncSet2TrajDatabase(encSet::IntruderEncounterSet; voxI::Float64=1., voyI::Float64=0., xCPA::Float64=0., yCPA::Float64=0.)
# Returns a set of trajectories as specified by an encounter set, suitable for batch simulation.
# itdb=convertEncSet2TrajDatabase(encSet);

	trajNum=1
	intTrajDB = Array(IntruderTrajectory, size(encSet.cpaLocs,1)*length(encSet.tCPAinit)*length(encSet.encAngles[1]))
	for cpaInd=1:size(encSet.cpaLocs,1)
		# For this CPA location, determine the intruders velocity so we can appropriately rotate it for consistency with
		# ownship's velocity and the specified encounter angle.
		xShift = encSet.cpaLocs[cpaInd,1]-xCPA    # How much to subtract off each x position to get it to the right CPA location
		yShift = encSet.cpaLocs[cpaInd,2]-yCPA    # How much to subtract off each x position to get it to the right CPA location
		timeHist = Float64[encSet.iStateHist[i].t for i=1:length(encSet.iStateHist)]
		minDT, trajCPAInd = findmin(abs(timeHist.-encSet.tCPAtraj[cpaInd]))    # This is the index of the state history corresponding to the CPA point
		intHead = encSet.iStateHist[trajCPAInd].heading    # We need to rotate the trajectory so that this heading is consistent with voxI, voyI and the encounter angle
		for (tCPAind, tCPA) in enumerate(encSet.tCPAinit)
			startDT, trajStartInd = findmin(abs(timeHist.-encSet.tCPAtraj[cpaInd]+tCPA))    # The index of the beginning of the trajectory (given the starting/init time to CPA)
			for (encAngind, encAng) in enumerate(encSet.encAngles[1])
				# Rotate the trajectories into the appropriate reference frame as determined by voxI and voyI
				ownHead = atan2(voyI, voxI)   # Note that we're using east as zero degrees, counter-clockwise positive rotation. which is different from the convention in the encounter model but consistent with the batch simulation code.  
				rotateAng = pi/180*encAng + ownHead - (pi/2-intHead)  # Rotation angle (counter-clockwise from east) is the sum of the encounter angle, ownship heading and negative (pi/2-intHead): since intHead is positive clockwise from north
				tShift = encSet.tCPAtraj[cpaInd]-tCPA    # Shift each trajectory so that the beginnig is tCPA seconds from CPA
				intTrajDB[trajNum] = IntruderTrajectory(rotateAndShiftTraj(encSet, rotateAng, xShift, yShift, trajStartInd, trajCPAInd, tShift)...,
										 								   encAng, tCPA, xCPA, yCPA, voxI, voyI)	
			    trajNum += 1			
			end
		end
	end

	return intTrajDB
end

function plotIntruderDataBase(itdb::IntruderTrajectory; plotVelocity::Bool=false)
	plotIntruderDataBase([itdb], plotVelocity=plotVelocity)
end

function plotIntruderDataBase(itdb::Array{IntruderTrajectory}; plotVelocity::Bool=false)
# Plots all the intruder trajectories in an encounter set.  Each CPA location is plotted in a separate figure, while the 
# trajectories with different encounter angles are plotted in the same figure.  Also shows the CPA location and the 
# ownship starting point.

	lastTrajLength = -1
	for (i,it) in enumerate(itdb)
		if length(it.x)!=lastTrajLength
			# Each new encounter location will get its own plot
			PyPlot.figure()
			xlabel("x position (ft)")
			ylabel("y position (ft)")
			x0 = it.xCPA-it.voxI*it.tCPA
			y0 = it.yCPA-it.voyI*it.tCPA
			PyPlot.plot(x0,y0,"o")
			PyPlot.plot(it.xCPA, it.yCPA, "rd")
			PyPlot.plot([x0, it.xCPA], [y0, it.yCPA], "r")
			lastTrajLength=length(it.x)
		end
		PyPlot.plot(it.x,it.y, "x-")
		PyPlot.plot(it.x[1],it.y[1],"s")
	end

	if plotVelocity
		lastTrajLength = -1
		for (i,it) in enumerate(itdb)
			if length(it.x)!=lastTrajLength 
				PyPlot.figure()
				dx=diff(it.x)
				dy=diff(it.y)
				dt=diff(it.t)
				v=sqrt(dx.^2+dy.^2)./dt
				minDT, trajCPAInd = findmin(abs(it.t.-it.tCPA))
				PyPlot.plot(it.t[2:end], v)
				PyPlot.plot(it.tCPA, v[trajCPAInd-1], "rd")
				xlabel("Time (s)")
				ylabel("Velocity (ft/s)")
				lastTrajLength=length(it.x)
			end
		end
	end

end

function plotEncounterSet(encSet::IntruderEncounterSet; voxI::Float64=1., voyI::Float64=0., xCPA::Float64=0., yCPA::Float64=0., plotVelocity::Bool=false)
	# Plots all the intruder trajectories in an encounter set.  Each CPA location is plotted in a separate figure, while the 
	# trajectories with different encounter angles are plotted in the same figure.  Also shows the CPA location and the 
	# ownship starting point.  Calls convertEncSet2TrajDatabase() and plotIntruderDataBase()

	itdb=convertEncSet2TrajDatabase(encSet, voxI=voxI, voyI=voyI, xCPA=xCPA, yCPA=yCPA)

	plotIntruderDataBase(itdb, plotVelocity=plotVelocity)

end

function plotEncounterSet(encSet::Array{IntruderEncounterSet}; voxI::Float64=1., voyI::Float64=0., xCPA::Float64=0., yCPA::Float64=0., plotVelocity::Bool=false)
	# Plots all the intruder trajectories in an encounter set.  Each CPA location is plotted in a separate figure, while the 
	# trajectories with different encounter angles are plotted in the same figure.  Also shows the CPA location and the 
	# ownship starting point.  Calls convertEncSet2TrajDatabase() and plotIntruderDataBase()

	itdb=convertEncSet2TrajDatabase(encSet, voxI=voxI, voyI=voyI, xCPA=xCPA, yCPA=yCPA)

	plotIntruderDataBase(itdb, plotVelocity=plotVelocity)

end

function setInitState(x0::Float64, y0::Float64, x::Float64, y::Float64, xd::Float64, yd::Float64, xdd::Float64, ydd::Float64, 
					  h::Float64, hd::Float64; t::Float64=0.)
# Will probably just want to specify a few states and then calculate all the others.  This function will do that.
	
	dx = x-x0
	dy = y-y0
	R = sqrt(dx^2+dy^2)

	psi_t = atan2(dx, dy)
	psi_t_dot = 0.
	if R!=0
		psi_t_dot = (xd*dy - yd*dx)/R^2
	end

	psi_h = atan2(xd, yd)
	psi_h_dot = 0.
	if (xd^2+yd^2)!=0.
		psi_h_dot = (xdd*yd - ydd*xd)/(xd^2+yd^2)
	end

	psi_v = mod(psi_h - psi_t, 2*pi)
	psi_v_dot = psi_h_dot - psi_t_dot

	v = sqrt(xd^2+yd^2)
	v_dot = sqrt(xdd^2 + ydd^2)

	IntruderState(h, hd, psi_h, psi_v, psi_v_dot, R, v, v_dot, x, y, x0, y0, xdd, ydd, t)	
end

function getNextState(intState::IntruderState, hdot_t2::Float64, vdot_t2::Float64, psivdot_t2::Float64, dt::Float64; 
					  psiTol::Float64=1e-5, psidotTol::Float64=1e-6, maxIters::Int64=20, VERBOSE::Bool=false)
# Given a current state and the values of the d/dt terms (h, v, psi) at the next time step, return
# a state that represents the next state.  Note that psivdot is the range-relative heading rate, a pretty wierd
# state, but one that hopefully captures the necessary behavior from the simulated trajectories.
	
	# Basic state updates:
	h_t1 = intState.altAGL
	hdot_t1 = intState.altdot
	v_t1 = intState.vel
	vdot_t1 = intState.vdot
	psiv_t1 = intState.rrHeading
	psivdot_t1 = intState.rrHeaddot

	h_t2 = h_t1 + 0.5*(hdot_t1 + hdot_t2)*dt
	v_t2 = v_t1 + 0.5*(vdot_t1 + vdot_t2)*dt
	psiv_t2 = mod(psiv_t1 + 0.5*(psivdot_t1 + psivdot_t2)*dt, 2*pi)

	# If v_t2 < 0 then strange things are going to happen.  My assumption is that velocity is always positive, but the
	# Bayes net could certainly sample a "deceleration" (negative vdot_t2) when velocity is low, which would indeed
	# cause a negative velocity.  I could either just set the velocity to zero in that case, or take the absolute value
	# of the velocity and rotate the heading and rrheading vectors 180 degrees.  Would have ot figure out what to do
	# with psivdot_t2
	if v_t2 < 0. 
		v_t2=0.
		vdot_t2=0. 
	end
	t1 = intState.t

	# Derived variable state updates:
	x_t1 = intState.x
	y_t1 = intState.y
	x0 = intState.x0
	y0 = intState.y0
	psih_t1 = intState.heading
	xd_t1 = v_t1*sin(psih_t1)
	yd_t1 = v_t1*cos(psih_t1)
	xdd_t1 = intState.xdd
	ydd_t1 = intState.ydd
	v_t1==0. ? psihdot_t1=0. : psihdot_t1 = (xdd_t1*yd_t1 - ydd_t1*xd_t1)/(v_t1^2)

	# Initialize variables for iterative psihdot search (plus other variables)
	psih_t2 = psih_t1
	psihdot_t2 = psihdot_t1
	psih_old = Inf
	psihdot_old = Inf
	iters = 0
	xdd_t2 = xdd_t1
	ydd_t2 = ydd_t1
	xd_t2 = xd_t1
	yd_t2 = yd_t1
	x_t2 = x_t1
	y_t2 = y_t1
	R_t2 = sqrt(x_t1^2+y_t1^2)
	while ((abs(psih_t2-psih_old) > psiTol) | (abs(psihdot_t2-psihdot_old) > psidotTol)) & (iters<maxIters)
		xdd_t2 = vdot_t2*sin(psih_t2) + v_t2*psihdot_t2*cos(psih_t2)
		ydd_t2 = vdot_t2*cos(psih_t2) - v_t2*psihdot_t2*sin(psih_t2)
		xd_t2 = xd_t1 + 0.5*(xdd_t1+xdd_t2)*dt
		yd_t2 = yd_t1 + 0.5*(ydd_t1+ydd_t2)*dt
		x_t2 = x_t1 + 0.5*(xd_t1+xd_t2)*dt
		y_t2 = y_t1 + 0.5*(yd_t1+yd_t2)*dt
		R_t2 = sqrt(x_t2^2+y_t2^2)

		psih_old = psih_t2
		psihdot_old = psihdot_t2
		psih_t2 = atan2(xd_t2,yd_t2)
		#	psihdot_t2 = (xdd_t2*yd_t2 - ydd_t2*xd_t2)/(xd_t2^2+yd_t2^2)  # Try the following instead?:
		R_t2==0. ? psitdot_t2 = 0. : psitdot_t2 = (xd_t2*(y_t2-y0) - yd_t2*(x_t2-x0))/R_t2^2
		psihdot_t2 = psivdot_t2-psitdot_t2

		# if VERBOSE
		# 	display("After iteration $(iters+1), error on heading is $(psih_t2-psih_old) and error on heading rate is $(psihdot_t2-psihdot_old)")
		# end

		iters+=1
	end
	t2 = t1+dt
	

	IntruderState(h_t2, hdot_t2, psih_t2, psiv_t2, psivdot_t2, R_t2, v_t2, vdot_t2, x_t2, y_t2, intState.x0, intState.y0, xdd_t2, ydd_t2, t2)

end

function simulateAndPlotIntruderTraj()

	iBN, tBN = loadBNs()

	stHistory = simulateAndPlotIntruderTraj(iBN, tBN);

end

function simulateAndPlotIntruderTraj(initBN::BayesNet, tranBN::BayesNet; 
							  psiTol::Float64=1e-6, psidotTol::Float64=1e-8, maxIters::Int64=20, VERBOSE::Bool=false, dt::Float64=0.2, 
							  tMax::Float64=120., bnp::BNParams=BNParams(), x0::Float64=0., y0::Float64=0., bearingInit::Float64=0., 
							  initEvidenceVal::Assignment=Assignment(), maxAttempts::Array{Int64}=[10000,100], t0::Float64=0.)
# stHistory = simulateAndPlotIntruderTraj(iBN, tBN);

	stHistory = simulateIntruderTraj(initBN, tranBN, psiTol=psiTol, psidotTol=psidotTol, maxIters=maxIters, VERBOSE=VERBOSE,
											dt=dt, tMax=tMax, bnp=bnp, bearingInit=bearingInit, initEvidenceVal=initEvidenceVal,
											maxAttempts=maxAttempts, t0=t0)

	plotIntruderTrajectory(stHistory, dt=dt)

	return stHistory
end	

function simulateIntruderTraj(initBN::BayesNet, tranBN::BayesNet; 
							  psiTol::Float64=1e-6, psidotTol::Float64=1e-8, maxIters::Int64=20, VERBOSE::Bool=false, dt::Float64=0.2, 
							  tMax::Float64=120., bnp::BNParams=BNParams(), x0::Float64=0., y0::Float64=0., bearingInit::Float64=0., 
							  initEvidenceVal::Assignment=Assignment(), maxAttempts::Array{Int64}=[10000,100], t0::Float64=0.)
# Perhaps this function could be used to create trajectories, which my simulation program can then play back directly rather
# than sampling the encounter model during my simulations.  The first etnry in maxAttempts is for the initial distribution, the
# second is for the transition distribution.
# stHistory = simulateIntruderTraj(iBN, tBN);

	stHistory = IntruderState[]
	t = 0.

	# Init state: sample from the initial bayes net:
	iState = setInitState(sampleBayesInitial(initBN, x0=x0, y0=y0, evidenceVal=initEvidenceVal, bnp=bnp,
											 maxAttempts=maxAttempts[1], bearingInit=bearingInit, VERBOSE=VERBOSE)..., t=t0)
	stHistory = [iState]

	while ~trajComplete(iState, tMax=tMax)

		# Sample from bayes net to get the next state variables
		hdot_t2, vdot_t2, psivdot_t2 = sampleBayesTransition(tranBN, iState, bnp=bnp, maxAttempts=maxAttempts[2], VERBOSE=VERBOSE)

		# integrate next state variables to get the complete state
		iState = getNextState(iState, hdot_t2, vdot_t2, psivdot_t2, dt, psiTol=psiTol, psidotTol=psidotTol, maxIters=maxIters, VERBOSE=VERBOSE)

		# record the state
		stHistory = [stHistory; iState]

	end

	return stHistory

end



function trajComplete(iState::IntruderState; tMax::Float64=120)
	# For now, will just simulate to a max time

	if iState.t>= tMax
		return true
	end

	return false
end

function plotIntruderTrajectory(stHistory::Array{IntruderState}; dt::Float64=0.2)

	x = Array(Float64,length(stHistory))
	y = Array(Float64,length(stHistory))
	t = Array(Float64,length(stHistory))
	h = Array(Float64,length(stHistory))
	v = Array(Float64,length(stHistory))

	for i=1:length(stHistory)
		x[i] = stHistory[i].x
		y[i] = stHistory[i].y
		t[i] = (i-1)*dt
		h[i] = stHistory[i].altAGL
		v[i] = stHistory[i].vel
	end

	figure()
	PyPlot.plot(x,y, "x-")
	xlabel("X position (ft)")
	ylabel("Y position (ft)")

	
	figure()
	PyPlot.plot(t,h)
	xlabel("Time (s)")
	ylabel("Altitude (ft)")

	figure()
	PyPlot.plot(t,v)
	xlabel("Time (s)")
	ylabel("Velocity (kts)")


end

function convertVals2Bins(bn::BayesNet, evidenceVal::Assignment; bnp::BNParams=BNParams())

	evidenceBin = Assignment()

	for key in keys(evidenceVal)
		binInd=1
		while (binInd<length(bnp.binDisc[key])) && (evidenceVal[key]>bnp.binDisc[key][binInd+1])
			binInd+=1
		end
		# If the value is larger than the last boundary in the discretization, the value will be binned in the last bin.
		binInd = clamp(binInd,1, length(domain(bn,key).elements))
		# evidenceBin[key] = bn.domains[bn.index[key]].elements[binInd]    # This line was for when indexing into an integer
		evidenceBin[key] = bn.nodes[bn.name_to_index[key]].domain.elements[binInd]

		# evidenceBin[key] = binInd
	end

	return evidenceBin

end

# function getBinIndfromKeyString(binVal::Int64; bnp::BNParams=BNParams())

# 	return 1

# end

function convertBins2Bounds(bn::BayesNet, samp::DataFrame; bnp::BNParams=BNParams())
	# Given a dataframe with a sample, look up the bin boundaries that correspond to the bin names (strings) that
	# comprise the domains of the BayesNet.  The nodes of the BayesNet, bn, will be labeled with the same symbol
	# as the dataframe columns.  The row of samp with data will have strings that are in the domains of the 
	# appropriate node in the BN. 

	j,n = size(samp)
	if j == 0
		display("Zero samples in bin->bound conversion, exiting.")
		return -1
	end

	if j>1
		display("More than one sample in bin->bound conversion, will only return the bins of the first: $(samp)")
		samp = samp[1,:]
	end

	binBounds = DataFrame()

	for key in names(samp)
		if key!=:p  	# This is a probability key generated by the likelihood-weighted sampling process.  Shouldn't need it.
			bins = bnp.binDisc[key]   # This is an array of floats with the bin boundaries for key (one of the bin labels)
			binVal = samp[key][1]  	  # This is the text string label for the bin in the sample corresponding to the above bin discretization
			#indBin = find(bn.domains[bn.index[key]].elements.==binVal)[1]   # This is the bin index corresponding to the string label
			
			binInd = find(bn.nodes[bn.name_to_index[key]].domain.elements.==binVal)[1]

			#binVal = samp[key][1]
			#binInd = find(domain(bn,key).elements.==binVal)[1]
			binBounds[key]= bins[binInd:binInd+1]
		end
	end

	return binBounds

end

function disc2contSample(binBounds::DataFrame)

	contSamp = DataFrame()
	for key in names(binBounds)
		l,h = binBounds[key]
		contSamp[key] = (h-l)*rand() + l
	end

	return contSamp

end

function testEvidenceBins(;rangeInit=10., rrHeadingInit=0., rrHeaddotInit=0., altAGLInit=5., altdotInit=0., dxdtInit=5., d2xdt2Init=0.)
	# This is just a testing function, allows me to quickly set a couple variables to make sure they're getting sampled correctly.

	evals = Assignment()
	evals[:range] = rangeInit
	# evals[:rrHeading] = rrHeadingInit
	# evals[:rrHeaddot] = rrHeaddotInit
	evals[:altAGL] = altAGLInit
	# evals[:altdot] = altdotInit
	evals[:dxdt] = dxdtInit
	# evals[:d2xdt2] = d2xdt2Init

	return evals
end

function convertRRVars2EuclVars(contSample::DataFrame; x0::Float64=0., y0::Float64=0., bearingInit::Float64=0.)
	# bearingInit in degrees

	R = contSample[:range][1]
	rrHeading = contSample[:rrHeading][1]
	rrHeaddot = contSample[:rrHeaddot][1]
	v = contSample[:dxdt][1]
	vd = contSample[:d2xdt2][1]

	x = x0 + R*sin(bearingInit)
	y = y0 + R*cos(bearingInit)

	xd = v*sin(bearingInit+rrHeading)
	yd = v*cos(bearingInit+rrHeading)

	bearingdot = (xd*(y-y0)-yd*(x-x0))/R^2

	xdd = vd*sin(bearingInit+rrHeading) + v*(rrHeaddot+bearingdot)*cos(bearingInit+rrHeading)
	ydd = vd*cos(bearingInit+rrHeading) - v*(rrHeaddot+bearingdot)*sin(bearingInit+rrHeading)

	h = contSample[:altAGL][1]
	hd = contSample[:altdot][1]

	return x, y, xd, yd, xdd, ydd, h, hd 

end

function convertiStateHist2EuclHist(stHist::Array{IntruderState})
	# The conversion from state history to euclidean space is actually quite easy since there are
	# many more parameters than the seven basic features in the encounter model.  Can directly
	# extract x,y,xdd,ydd and just use simple geometry to get xd and yd.

	bearingInit = atan2(stHist[1].x, stHist[2].y)
	euclHist = Array(Float64, length(stHist), 8)
	contSample = DataFrame()
	for i=1:length(stHist)
		contSample[:range] = stHist[i].range
		contSample[:rrHeading] = stHist[i].rrHeading
		contSample[:rrHeaddot] = stHist[i].rrHeaddot
		contSample[:dxdt] = stHist[i].vel
		contSample[:d2xdt2] = stHist[i].vdot
		contSample[:altAGL] = stHist[i].altAGL
		contSample[:altdot] = stHist[i].altdot

		euclHist[i,:] = collect(convertRRVars2EuclVars(contSample, x0=stHist[i].x0, y0=stHist[i].y0, bearingInit=bearingInit))
	end

	return euclHist

end

function convertAvUnits2Basic(samp::DataFrame)

	sampConv = DataFrame()
	for key in names(samp)

		if key == :rrHeaddot
			sampConv[key] = pi/180 * samp[key]
		elseif key == :rrHeading
			sampConv[key] = pi/180 * samp[key]
		elseif key == :rrHeaddot_tp1
			sampConv[key] = pi/180 * samp[key]
		else
			sampConv[key] = samp[key]
		end
	end

	return sampConv
end

function sampleBayesInitial(iBN::BayesNet; x0::Float64=0., y0::Float64=0., evidenceVal::Assignment=Assignment(), bnp::BNParams=BNParams(),
							maxAttempts::Int64=1000, bearingInit::Float64=0., VERBOSE::Bool=false)
	# The initial heading is not part of the bayes net, and shouldn't affect the dynamics or interactions with other aircraft, so a 
	# default value of 0 will be used.  
	# Function returns a continuous sample of the bayes net, optionally accepting set values for specific parameters in evidenceVal.
	# All variables coming out of this model will be in ft, ft/s, or ft/s^2.

	# x0, y0, x, y, xd, yd, xdd, ydd, h, hd = sampleBayesInitial(iBN)
	# x0, y0, x, y, xd, yd, xdd, ydd, h, hd = sampleBayesInitial(iBN, x0=0., y0=0., evidenceVal=Assignment(), bnp=BNParams(), maxAttempts=1000, bearingInit=0.)


	# If there is evidence, set that in the bayes net.  First have to get bins:
	evidenceBins = convertVals2Bins(iBN, evidenceVal, bnp=bnp)

	# Sample the network, getting bins.
	initSamp = rand_table_weighted(iBN, numSamples=1, consistentWith=evidenceBins)
	ind = 1
	while (size(initSamp,1)==0) && (ind<maxAttempts)
		initSamp = rand_table(iBN, numSamples=1, consistentWith=evidenceBins)
		ind+=1
	end
	if size(initSamp,1)==0
		if VERBOSE
			display("Could not find initial sample consistent with evidence.  Returning random sample.")
		end
		ind = 1
		while (size(initSamp,1)==0) && (ind<maxAttempts)
			initSamp = rand_table(iBN, numSamples=1)
			ind+=1
		end
	end

	# Convert the bin assignments to a table of boundaries for the bins
	binSampleBounds = convertBins2Bounds(iBN, initSamp, bnp=bnp)

	# Pull a continuous sample from the bins.
	contSample = disc2contSample(binSampleBounds)

	# Need to do the unit conversion right after sampling from the BN so that our state conversion equations are more
	# straightforward.  This should be cleaner becaues only the BN will be in aviation units, everything else will
	# be done in ft, ft/s, (check on altitude rate, make sure it's not in ft/min), radians.
	contSampleBasic = convertAvUnits2Basic(contSample)

	# Convert the sample from the existing variables to the starter variables
	x, y, xd, yd, xdd, ydd, h, hd = convertRRVars2EuclVars(contSampleBasic, x0=x0, y0=y0, bearingInit=bearingInit)

	# To test with a specific initial condition:
	# x=0.
	# y=0.
	# xd=0.
	# yd=0.
	# xdd=0.
	# ydd=0.
	# h=10.
	# hd=0.

	return x0, y0, x, y, xd, yd, xdd, ydd, h, hd

end

function setTransitionEvidence(tBN::BayesNet, iState::IntruderState; bnp::BNParams=BNParams())
# this function sets the seven relevant feature values in iState into a dictionary, returning a mapping from
# features to feature values (not bins).  These feature

	evidenceVal = Assignment()

	evidenceVal[:altAGL] = iState.altAGL
	evidenceVal[:altdot] = iState.altdot
	evidenceVal[:d2xdt2] = iState.vdot
	evidenceVal[:rrHeaddot] = iState.rrHeaddot
	evidenceVal[:range] = iState.range
	evidenceVal[:dxdt] = iState.vel
	evidenceVal[:rrHeading] = iState.rrHeading

	convertVals2Bins(tBN, evidenceVal, bnp=bnp)

end

function zeroOutSpanningBins!(sample, binSampleBounds)

	for key in names(sample)
		if (maximum(binSampleBounds[key])>0.) & (minimum(binSampleBounds[key])<0.)
			sample[key]=0.
		end
	end

end


function sampleBayesTransition(tBN::BayesNet, iState::IntruderState; bnp::BNParams=BNParams(), maxAttempts::Int64=1000, VERBOSE::Bool=false)

	# Set the given evidence (pulls out the relevant state from iState, sets bins)
	#display("$iState")
	evidenceBins = setTransitionEvidence(tBN, iState, bnp=bnp)

	# Pull a sample from network, getting bins
	tSamp = rand_table_weighted(tBN, numSamples=1, consistentWith=evidenceBins)
	ind=1
	while (size(tSamp,1)==0) && (ind<maxAttempts)
		tSamp = rand_table_weighted(tBN, numSamples=1, consistentWith=evidenceBins)
		ind+=1
	end
	if size(tSamp,1)==0
		if VERBOSE
			display("Could not find transition sample consistent with evidence.  Returning random sample.")
		end
		ind = 1
		while (size(tSamp,1)==0) && (ind<maxAttempts)
			tSamp = rand_table_weighted(tBN, numSamples=1)
			ind+=1
		end
	end

	# Go through each bin, comparing whether it is a different bin from the last bin.  If it's the same, resample a continuous point
	# with probability 1-epsilon (epsilon ~ 0.02-0.08, or learn that value from the data). If its a different bin, just sample agoin.
	# As a first implementation I could just resample every time.

	# Convert the bin assignments to a table of boundaries for the bins
	binSampleBounds = convertBins2Bounds(tBN, tSamp, bnp=bnp)

	# Pull a continuous sample from the bins.
	contSample = disc2contSample(binSampleBounds)

	# Check whether the bin contains zero. If it does, return exactly zero.  Note that this will operate on all fields, while
	# we only care about zeroing out the t+1 fields (altdot, rrHeaddot and vdot).  Implementation will be cleaner if we 
	# zero out every field and then just return the appropriate ones.
	zeroOutSpanningBins!(contSample, binSampleBounds)

	# Need to do the unit conversion right after sampling from the BN so that our state conversion equations are more
	# straightforward.  This should be cleaner becaues only the BN will be in aviation units, everything else will
	# be done in ft, ft/s, (check on altitude rate, make sure it's not in ft/min), radians.
	contSampleBasic = convertAvUnits2Basic(contSample)

	hdot_t2 = contSampleBasic[:altdot_tp1][1]
	vdot_t2 = contSampleBasic[:d2xdt2_tp1][1]
	psivdot_t2 = contSampleBasic[:rrHeaddot_tp1][1]

	return hdot_t2, vdot_t2, psivdot_t2

end