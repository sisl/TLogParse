using HDF5, JLD, DataFrames, DataArrays, PyPlot, Interpolations, Dierckx

# This program is desigen to process Mission Planner .tlog files into Julia data structures.  It relys on calls
# to the C# programe TLogReader.exe, which converts the tlog file into a text file.  The usage for that program is:
# >> TLogReader.exe inputfile.tlog [outputfile.txt] [verbose: true/false] [FieldMatchFile.txt]

# For the verbose flag, simply type "true" or "false", there are no - characters.  The output file is optional, if
# not specified the output will be the same name as the .tlog file but with a .txt extension.  The last parameter
# (they must be specified in this order) is a list of field names to convert into the text file (1 per line).  If omitted, every
# field will be converted (resulting in a file about 10x as large as the .tlog).  To reduce the output size, which
# is recommended becaues there are well over a hundred parameters in the .tlog file and most won't be useful, specify
# the parameter name only.  See the following list of parameters, which are separated from their MAVLink-specified 
# data types by the three characters "_._".  The first entry indicates that the parameter name is "time_unix_usec" 
# while its type is "mavlink_system_time_t".  The type was retained because there are several parameters, like "alt" 
# that are written several times as different types.  I wanted to preserve the ability to differentiate between 
# them.  Note that if you specify "alt" in the FieldMatchFile you'll get all three different flavors of that variable.

# Note also that users specify which telemetry parameters to pass over MAVLink and at what rate, and that there is no
# guarantee that every parameter will be received/stored at each time step.  That's bandwidth limited.  So the presence
# of a parameter in one .tlog file is no guarantee that it will be in another.  See the Mission Planner documentation
# for an explanation of these parameters (http://planner.ardupilot.com/wiki/mission-planner-overview/)

type FilterParams

	distanceOutlier::Float64 		# Points thta increase path length more than this amount will be removed (probably from spurrious position estimates or data corruption).  May use velocity instead
	velocityOutlier::Float64		# Velocity points (either position/timestamp or actual velocity?) outside this threshold will be removed.
	keyParams::Array{AbstractString}  # These params are the key variables used to construct the model, so any data points in which they are NaN will be removed.
	latParam::Int64		# Index into keyParams indicating which parameter is the latitude metric (special filtering required on this)
	lonParam::Int64 	# Index into keyParams indicating which parameter is the longitude metric (special filtering required on this)
	params2Smooth::Array{AbstractString}   # Will apply smoothing to these listed parameters (probably all the key parameters?)
	smoothingSigma::Array{Float64}		   # The standard deviation (in seconds) of the gaussian smoothing function for each variable
	tInterval::Float64  		# The interval to interpolate each trajectory into.
	maxSmoothSigma::Float64

	timeStr::AbstractString
	latStr::AbstractString
	lonStr::AbstractString
	altStr::AbstractString 
	vxStr::AbstractString 
	vyStr::AbstractString

	useSplineFit::Bool  	    # If true, uses a 1D cubic spline for interpolation using the Dierckx.jl package (see https://github.com/kbarbary/Dierckx.jl for definitions of the following parameters) 
						 	    # If false, uses the linear interpolation capability in Interpolations.jl
	degSpline::Int64 			# Degree of spline fit (up to 5, default is cubic, or 3)
	bcSpline::AbstractString    # Options are "zero", "extrapolate", "error" and "nearest"
	smthFactorSpline::Float64   # Smoothing factor (higher is smoother, but approx may not be close to the data points).  By means of this parameter, the user can control the tradeoff between closeness of fit and smoothness of fit of the approximation. if s is too large, the spline will be too smooth and signal will be lost ; if s is too small the spline will pick up too much noise.

	FilterParams(distanceOutlier, velocityOutlier, keyParams, latParam, lonParam, params2Smooth, smoothingSigma, tInterval, maxSmoothSigma, timeStr, latStr, lonStr, altStr, vxStr, vyStr, useSplineFit, degSpline, bcSpline, smthFactorSpline) = 
			 new(distanceOutlier, velocityOutlier, keyParams, latParam, lonParam, params2Smooth, smoothingSigma, tInterval, maxSmoothSigma, timeStr, latStr, lonStr, altStr, vxStr, vyStr, useSplineFit, degSpline, bcSpline, smthFactorSpline)
end

function FilterParams()

	distanceOutlier = 25.  # Outlier distance in m
	velocityOutlier = 100. # Maximum velocity in m/s (100 m/s equates to about 195 kts)
	keyParams = ["lat_._mavlink_global_position_int_t", "lon_._mavlink_global_position_int_t",
		 		 "alt_._mavlink_global_position_int_t", "vx_._mavlink_global_position_int_t",
				 "vy_._mavlink_global_position_int_t"]
	latParam = 1
	lonParam = 2
	params2Smooth = ["lat_._mavlink_global_position_int_t", "lon_._mavlink_global_position_int_t",
		 		 	 "alt_._mavlink_global_position_int_t", "vx_._mavlink_global_position_int_t",
				 	 "vy_._mavlink_global_position_int_t"]
	smoothingSigma = [2., 2., 2., 2., 2.]
	tInterval = 0.2 			# When I set this to 1.0 the trajectories are frequently much more "linear" and discretized looking than the original data. 0.1 or 0.2 matches the original data much better.  Could be a fair amount of data, but I'm going to want to sample the trajectories faster than 1 sec anyway.
	maxSmoothSigma = 4.  		# Number of standard deviations over which to smooth (3-4 should be sufficient)

	# The following are the field names of the variables I'll use to make the encounter model. Want to use
	# these rather than the alternatives.  I may want to replace the above keyParams and params2Smooth with only
	# these.  Though note that some metrics are derived from these, like horizontal velocity and climb rate.
	timeStr = "time_unix_usec_._mavlink_system_time_t"
	latStr = "lat_._mavlink_global_position_int_t"
	lonStr = "lon_._mavlink_global_position_int_t"
	altStr = "alt_._mavlink_global_position_int_t"
	vxStr = "vx_._mavlink_global_position_int_t"
	vyStr = "vy_._mavlink_global_position_int_t"

	useSplineFit = true  	# If true, uses a 1D cubic spline for interpolation using the Dierckx.jl package (see https://github.com/kbarbary/Dierckx.jl for definitions of the following parameters) 
						 	# If false, uses the linear interpolation capability in Interpolations.jl
	degSpline = 3 			# Degree of spline fit (up to 5, default is cubic, or 3)
	bcSpline = "nearest"    # Options are "zero", "extrapolate", "error" and "nearest"
	smthFactorSpline = 0.0  # Smoothing factor (higher is smoother, but approx may not be close to the data points).  By means of this parameter, the user can control the tradeoff between closeness of fit and smoothness of fit of the approximation. if s is too large, the spline will be too smooth and signal will be lost ; if s is too small the spline will pick up too much noise.

	return FilterParams(distanceOutlier, velocityOutlier, keyParams, latParam, lonParam, params2Smooth, smoothingSigma, tInterval, maxSmoothSigma, timeStr, latStr, lonStr, altStr, vxStr, vyStr, useSplineFit, degSpline, bcSpline, smthFactorSpline)
end

function FilterParams(smoothingSigma::Array{Float64})

	distanceOutlier = 25.  # Outlier distance in m
	velocityOutlier = 100. # Maximum velocity in m/s (100 m/s equates to about 195 kts)
	keyParams = ["lat_._mavlink_global_position_int_t", "lon_._mavlink_global_position_int_t",
		 		 "alt_._mavlink_global_position_int_t", "vx_._mavlink_global_position_int_t",
				 "vy_._mavlink_global_position_int_t",  "vz_._mavlink_global_position_int_t",
				 "hdg_._mavlink_global_position_int_t", "airspeed_._mavlink_vfr_hud_t"]
	latParam = 1
	lonParam = 2
	params2Smooth = ["lat_._mavlink_global_position_int_t", "lon_._mavlink_global_position_int_t",
		 		 	 "alt_._mavlink_global_position_int_t", "vx_._mavlink_global_position_int_t",
				 	 "vy_._mavlink_global_position_int_t"]
	#smoothingSigma = [2., 2., 5., 1., 1.]
	tInterval = 1.
	maxSmoothSigma = 4.  		# Number of standard deviations over which to smooth (3-4 should be sufficient)

	# The following are the field names of the variables I'll use to make the encounter model. Want to use
	# these rather than the alternatives.  I may want to replace the above keyParams and params2Smooth with only
	# these.  Though note that some metrics are derived from these, like horizontal velocity and climb rate.
	timeStr = "time_unix_usec_._mavlink_system_time_t"
	latStr = "lat_._mavlink_global_position_int_t"
	lonStr = "lon_._mavlink_global_position_int_t"
	altStr = "alt_._mavlink_global_position_int_t"
	vxStr = "vx_._mavlink_global_position_int_t"
	vyStr = "vy_._mavlink_global_position_int_t"

	useSplineFit = true  	# If true, uses a 1D cubic spline for interpolation using the Dierckx.jl package (see https://github.com/kbarbary/Dierckx.jl for definitions of the following parameters) 
						 	# If false, uses the linear interpolation capability in Interpolations.jl
	degSpline = 3 			# Degree of spline fit (up to 5, default is cubic, or 3)
	bcSpline = "nearest"    # Options are "zero", "extrapolate", "error" and "nearest"
	smthFactorSpline = 0.0  # Smoothing factor (higher is smoother, but approx may not be close to the data points).  By means of this parameter, the user can control the tradeoff between closeness of fit and smoothness of fit of the approximation. if s is too large, the spline will be too smooth and signal will be lost ; if s is too small the spline will pick up too much noise.


	return FilterParams(distanceOutlier, velocityOutlier, keyParams, latParam, lonParam, params2Smooth, smoothingSigma, tInterval, maxSmoothSigma, timeStr, latStr, lonStr, altStr, vxStr, vyStr, useSplineFit, degSpline, bcSpline, smthFactorSpline)
end

function getUnitConversions()
# Creates the static unit conversions necessary to get aviation units.
 # roll_._mavlink_attitude_t, -0.01629377
 # pitch_._mavlink_attitude_t, -0.03017771
 # yaw_._mavlink_attitude_t, 1.848189
 # rollspeed_._mavlink_attitude_t, -0.005241281
 # pitchspeed_._mavlink_attitude_t, -0.0002181679
 # yawspeed_._mavlink_attitude_t, -0.001133392
 # alt_._mavlink_vfr_hud_t, 0.88
 # lat_._mavlink_gps_raw_int_t, 54566201
 # lon_._mavlink_gps_raw_int_t, 1004433554
 # alt_._mavlink_gps_raw_int_t, 14660
 # eph_._mavlink_gps_raw_int_t, 180
 # epv_._mavlink_gps_raw_int_t, 65535
 # vel_._mavlink_gps_raw_int_t, 1
 # lat_._mavlink_global_position_int_t, 54566163
 # lon_._mavlink_global_position_int_t, 1004433462
 # alt_._mavlink_global_position_int_t, 11070
 # vx_._mavlink_global_position_int_t, 1
 # vy_._mavlink_global_position_int_t, 1
 # vz_._mavlink_global_position_int_t, -1

	unitDict = Dict{Symbol,Float64}()

	unitDict[symbol("time_unix_usec_._mavlink_system_time_t")] = 10.0^-6      # Converts microseconds to seconds
	unitDict[symbol("alt_._mavlink_global_position_int_t")] = 10.0^-3 * 3.28 	# Converts mm to ft
	unitDict[symbol("alt_._mavlink_gps_raw_int_t")] = 10.0^-3 * 3.28 	# Converts mm to ft
	unitDict[symbol("lat_._mavlink_gps_raw_int_t")] = 10.0^-7
	unitDict[symbol("lat_._mavlink_global_position_int_t")] = 10.0^-7 		# These are originally in degrees*10^7, so will result in degrees again
	unitDict[symbol("lon_._mavlink_gps_raw_int_t")] = 10.0^-7
	unitDict[symbol("lon_._mavlink_global_position_int_t")] = 10.0^-7
	unitDict[symbol("vx_._mavlink_global_position_int_t")] = 10.0^-2 * 3600 * 3.28 / 6071  # Converts from cm/s to kts
	unitDict[symbol("vy_._mavlink_global_position_int_t")] = 10.0^-2 * 3600 * 3.28 / 6071  # Converts from cm/s to kts
	unitDict[symbol("vz_._mavlink_global_position_int_t")] = 10.0^-2 * 3.28 / 60  # Converts from cm/s to ft/min
	unitDict[symbol("climb_._mavlink_vfr_hud_t")] = 1.  # Not sure what units these were
	unitDict[symbol("airspeed_._mavlink_vfr_hud_t")] = 1.  # Not sure what units these were
	unitDict[symbol("hdg_._mavlink_global_position_int_t")] = 1.  # Not sure what units these were
	unitDict[symbol("vel_._mavlink_gps_raw_int_t")] = 10.0^-2 * 3600 * 3.28 / 6071  # Converts from cm/s to kts

	return unitDict

end

function extractFeatures(tlogDatArray::Array{DataFrame})
	# This function extracts a set of "features" from the raw tlog data.  The set is currently large because
	# I don't know which features I'l lend up using.


end

# function loadTLog(;readpath::AbstractString="./tlogout")
# # This function loads tlog jld files from the specified read path and returns them  in a tlog data array.
# # tlogDatArray, fParamArray = loadTLog(readpath="./")

# 	tlogDatArray = DataFrame[]
# 	fParamArray = FilterParams[]
# 	fileList = readdir(readpath)

# 	for i=1:length(fileList)
# 		if length(fileList[i])>5
# 	 		if (fileList[i][end-2:end] == "jld")
# 	 			display("Found jld file: $(fileList[i])")
# 	 			#try
# 		 			varsLoaded = @load fileList[i];
# 		 		#catch
# 		 		#	varsLoaded = Symbol[]
# 		 		#end
# 	 			if in(symbol("tlogDatInterp"),varsLoaded)
# 		 			tlogDatArray = [tlogDatArray; tlogDatInterp]
# 		 			display("Reading data from $(fileList[i])")
# 		 			if in(symbol("filterParams"),varsLoaded)
# 		 				fParamArray = [fParamArray; filterParams]
# 		 			else
# 		 				fParamArray = [fParamArray; FilterParams()]
# 		 			end
# 		 		end
# 	 		end
# 	 	end
# 	 end

# 	 return tlogDatArray, fParamArray

# end

function processTLog(;readpath::AbstractString="./", writepath::AbstractString="./", createTextLogs::Bool=false, 
					  jldSaveNameBase::AbstractString="tlogData", fieldMatchFile::AbstractString="FieldMatchFile.txt", 
					  VERBOSE::Bool=false, timeKey::AbstractString="time_unix_usec_._mavlink_system_time_t",
					  maxTlogsinArray::Int64=20, tlogParseExe::AbstractString="./TLogReader.exe", 
					  filterParams::FilterParams=FilterParams())
# This function can either convert tlog files into text files, or convert the data into
# Julia data structures and leave off the large text files.  Set createTextLogs to true
# to retain the text files. If false, they will be deleted.
# tlogDatArray, udArray, timeKey = processTLog();
# tlogDatArray, udArray, timeKey = processTLog(readpath="./", writepath="./", createTextLogs=false, jldSaveNameBase="tlogData.jld", fieldMatchFile="FieldMatchFile.txt", VERBOSE=false, timeKey="time_unix_usec_._mavlink_system_time_t");

	outFileName = string(writepath,"templog.txt")
	tlogDatArray = DataFrame[]
	udArray = DataArray[]
	fileList = readdir(readpath)
	unitDict = getUnitConversions()

	for i=1:length(fileList)
		if length(fileList[i])>5
	 		if fileList[i][end-3:end] == "tlog"
	 			if VERBOSE
			 		display("Found tlog file: $(fileList[i])")
			 	end
		 		if createTextLogs
		 			outFileName = string(writepath, fileList[i][1:end-5], ".txt")
		 		end

		 		if VERBOSE
			 		display("Running command: $tlogParseExe $(string(readpath,fileList[i])) $outFileName $VERBOSE $(string(readpath,fieldMatchFile))")
				end

		 		tic();
				run(`$tlogParseExe $(string(readpath,fileList[i])) $outFileName $VERBOSE $(string(readpath,fieldMatchFile))`)

				#  Now that we have a text file with our contents, read them in using Julia functions
				if !isfile(outFileName)
					display("***Error, no tlog output file created for $(outFileName)")
				else
					statFile = stat(outFileName)
					if statFile.size==0
						display("Processed tlog file is size zero: $(fileList[i])")
					else
						tempFrame = readtable(outFileName, separator=',', header=false, names=[:Key,:Val])
						# Put the data into table format, one row per time stamp (where the time variable is specified in timeKey):
						# ud is a data array of strings, each of which can be converted to a symbol with symbol(ud[i]) to index tlogDat parameters.
						tlogDat, ud = convertList2Array(tempFrame, i, filterParams=filterParams)

						if isempty(tlogDat)
							display("Did not find a time key in $(fileList[i]).  Skipping this file.")

						else
							# Remove the spurrious data points (zero or NaN/missing values, outliers)
							tlogDat=filterRawData(tlogDat, filterParams=filterParams)

							# Convert units to aviation units:
							tlogDat = convertUnits(tlogDat, unitDict)

							# Smooth data using a gaussian kernel:
							tlogDat=smoothHistories(tlogDat, filterParams=filterParams)

							# Interpolate the data to consistent intervals - 0.5 or 1.0 sec? Piecewise cubic hermite interpolation?
							tlogDatInterp = interpolateHistories(tlogDat, filterParams=filterParams)

							if !isempty(tlogDatInterp)
								# Fetch additional data, if possible:
								#   1. AGL data
								#   2. Airspace class data
								#   3. Country location
								if length(fileList)<=maxTlogsinArray
									# If there are more than maxTlogsinArray files, don't store everything in one giant data array.  Will
									# need to plot and analyze trajectories individually through their jld files, I think.
									push!(tlogDatArray,tlogDatInterp)
									push!(udArray,ud)
								end

								# The jld files are quite large, four tlog files totaled 2.4 MB in .jld, so I'll save them individually.
								jldSaveName = string(jldSaveNameBase, i, ".jld")
								@save string(writepath,jldSaveName) tlogDatInterp filterParams
							end
						end
					end
				end
				
				toc()
		 	end
		end
	end

	# When we're done processing tlog files, if we're not to create text logs then delete the templog file:
	if !createTextLogs
		run(`rm $outFileName`)
	end


	return tlogDatArray, udArray, timeKey

end

function convertList2Array(datList::DataFrame, acNum::Int64; filterParams::FilterParams=FilterParams())
# Potential options for time stamps (there may be others): 
#  time_unix_usec_._mavlink_system_time_t, 1445226177730000
#  time_boot_ms_._mavlink_system_time_t, 241450
#  time_boot_ms_._mavlink_scaled_pressure_t, 241951
#  time_boot_ms_._mavlink_attitude_t, 241952
#  time_boot_ms_._mavlink_global_position_int_t, 254938
#  time_boot_ms_._mavlink_rc_channels_raw_t, 254941
#
# Other variables/keys I'll use (and representative values from one tlog):
 # xacc_._mavlink_raw_imu_t, -8
 # yacc_._mavlink_raw_imu_t, -11
 # zacc_._mavlink_raw_imu_t, -984
 # roll_._mavlink_attitude_t, -0.01629377
 # pitch_._mavlink_attitude_t, -0.03017771
 # yaw_._mavlink_attitude_t, 1.848189
 # rollspeed_._mavlink_attitude_t, -0.005241281
 # pitchspeed_._mavlink_attitude_t, -0.0002181679
 # yawspeed_._mavlink_attitude_t, -0.001133392
 # alt_._mavlink_vfr_hud_t, 0.88
 # lat_._mavlink_gps_raw_int_t, 54566201
 # lon_._mavlink_gps_raw_int_t, 1004433554
 # alt_._mavlink_gps_raw_int_t, 14660
 # eph_._mavlink_gps_raw_int_t, 180
 # epv_._mavlink_gps_raw_int_t, 65535
 # vel_._mavlink_gps_raw_int_t, 1
 # lat_._mavlink_global_position_int_t, 54566163
 # lon_._mavlink_global_position_int_t, 1004433462
 # alt_._mavlink_global_position_int_t, 11070
 # vx_._mavlink_global_position_int_t, 1
 # vy_._mavlink_global_position_int_t, 1
 # vz_._mavlink_global_position_int_t, -1

 	timeKey = filterParams.timeStr
	datArray = DataFrame()
	timeKeyFound = false
	ud=unique(datList[:Key])
	ud = [ud; "acNum"]
	numTimeElems = sum(datList[:,:Key].==timeKey)
 	for i=1:length(ud) 
 		datArray[symbol(ud[i])]=DataArray(Float64,numTimeElems)
 		#datArray[:,symbol(ud[i])] = NA
 	end
	datArray[symbol("acNum")]=acNum*ones(Float64,numTimeElems)

	timeInd = 0
	for i=1:size(datList,1)
		if datList[i,:Key]==timeKey
			timeKeyFound=true
			timeInd+=1
		end		
		
		if timeKeyFound
			# Now we can start putting data elements in the right locations
			datArray[timeInd, symbol(datList[i,:Key])] = datList[i,:Val]
		end
	end

	# Need to remove last rows if we didn't populate them?
	datArray = datArray[1:timeInd,:]

	return datArray, ud

end

function  convertUnits(datArray::DataFrame, unitDict::Dict{Symbol,Float64})
# Converts from the units in Mission Planner .tlogs into standard aviation terms.  The entries 
# in unitDict are multiplied by the default tlog values to get aviation units.

	for sym in names(datArray)
		datArray[:,sym] = datArray[:,sym].*get(unitDict, sym, 1)  	# If we don't have the conversion, leave it unchanged
	end

	return datArray

end

function filterRawData(datArray::DataFrame; filterParams::FilterParams=FilterParams())
# This function does all the processing and filtering on the raw data to get it into a usable format:
#	1. Remove the time stamps that are zero (there may be multiple, which make the data unusable)
#	2. Remove position locations that greatly increase the path length (these are probably spurrious)

	# Remove the NaNs from the key parameters, the zeros from the timeKey, and (0,0) entries from lat/longs
	timeKey=filterParams.timeStr
	goodInds = collect(1:size(datArray,1))
	zeroInds = find(datArray[:,symbol(timeKey)].==0)
	zeroLLInds = find((datArray[:,symbol(filterParams.keyParams[filterParams.latParam])].==0) & (datArray[:,symbol(filterParams.keyParams[filterParams.lonParam])].==0))
	naInds=Int64[]
	for prm in filterParams.keyParams
		if sum(symbol(prm).==names(datArray))>0    # Check whether this parameter is actually in the data set
			naInds = [naInds; find(isna(datArray[:,symbol(prm)]))]
		end
	end
	deleteat!(goodInds,sort(unique([naInds ; zeroInds; zeroLLInds])))
	datArray = datArray[goodInds, :]

	# Convert values into appropriate units?  Would be nice no to worry about those here.

	# skipping this check for now: check for positions that greatly increase path distance (i.e. remove outliers):

	# Remove velocity outliers based on position changes (not the actual velocities, but perhaps I should check wether those are inconsistent?)
	datArray = removeVelOutliers(datArray, filterParams=filterParams)

	# Remove missing altitudes, excessive vertical rates

	# Remove altitudes or positions that come before positions or altitudes?  This may already be taken care of

	return datArray
end

# Start this on 12/11
function removeVelOutliers(datArray::DataFrame; filterParams::FilterParams=FilterParams())
	# May not need to do this, don't see very many points that have giant velocities for one step, perhaps
	# because we're directly recording velocity and not estimating it from positions and times.

	# Mykel removed outliers for both the horizontal and vertical velocities.  Will skip this for now.
	return datArray

end

function smoothHistories(datArray::DataFrame; filterParams::FilterParams=FilterParams())
# Throw away the points that don't have enough smoothing points before or after them?
# Don't go through every point with the filter, limit it to +-4 standard deviations from current point

	timeKey = filterParams.timeStr
	timeHist = datArray[:,symbol(timeKey)]
	meandt = mean(diff(timeHist))
	#w = zeros(Float64,length(timeHist))

	for prmInd = 1:length(filterParams.params2Smooth)
		if filterParams.smoothingSigma[prmInd]!=0.  		# Don't smooth if the filter sigma is zero
			prm = filterParams.params2Smooth[prmInd]
			rawParam = datArray[:,symbol(prm)]
			deltaInd = ceil(Int64, filterParams.maxSmoothSigma*filterParams.smoothingSigma[prmInd]/meandt)  # This is the number of indicies we will smooth over (otherwise we go over a TON of useless indicies)
			
			#filtParam = Array(Float64,length(rawParam))
			for i=1:length(rawParam)
				#fill!(w, 0.)
				# wSum = 0.
				# dotProd = 0.
				minIndSmooth = maximum([1, i-deltaInd])
				maxIndSmooth = minimum([length(rawParam), i+deltaInd])
				
				wSum, dotProd = getSmoothed(rawParam, timeHist, timeHist[i], minIndSmooth, maxIndSmooth, filterParams.smoothingSigma[prmInd])
				datArray[i,symbol(prm)] = dotProd/wSum
			end   
			#datArray[:,symbol(prm)] = filtParam
		end
	end

	return datArray

end

# Original version:
# function smoothHistories(datArray::DataFrame; filterParams::FilterParams=FilterParams())
# # Throw away the points that don't have enough smoothing points before or after them?
# # Don't go through every point with the filter, limit it to +-4 standard deviations from current point

# 	timeKey = filterParams.timeStr
# 	timeHist = datArray[:,symbol(timeKey)]
# 	meandt = mean(diff(timeHist))

# 	for prmInd = 1:length(filterParams.params2Smooth)
# 		if filterParams.smoothingSigma[prmInd]!=0.  		# Don't smooth if the filter sigma is zero
# 			prm = filterParams.params2Smooth[prmInd]
# 			rawParam = datArray[:,symbol(prm)]
# 			deltaInd = ceil(Int64, filterParams.maxSmoothSigma*filterParams.smoothingSigma[prmInd]/meandt)  # This is the number of indicies we will smooth over (otherwise we go over a TON of useless indicies)
			
# 			#filtParam = Array(Float64,length(rawParam))
# 			for i=1:length(rawParam)
# 				minIndSmooth = maximum([1, i-deltaInd])
# 				maxIndSmooth = minimum([length(rawParam), i+deltaInd])
# 				w = zeros(Float64,1+maxIndSmooth-minIndSmooth)
# 				getFiltWeights!(w, timeHist[minIndSmooth:maxIndSmooth], timeHist[i], filterParams.smoothingSigma[prmInd])
# 				# filtParam[i] = dot(w, rawParam)/sum(w)
# 				datArray[i,symbol(prm)] = dot(w, rawParam[minIndSmooth:maxIndSmooth])/sum(w)
# 			end   
# 			#datArray[:,symbol(prm)] = filtParam
# 		end
# 	end

# 	return datArray

# end

function getSmoothed(rawParam::DataArray{Float64}, timeHist::DataArray{Float64}, t::Float64, minInd::Int64, maxInd::Int64, sig::Float64)
	# Calculates the value of a parameter after having been passed through a gaussian smoothing filter:
	coef = 1/(sig*sqrt(2*pi))
	den = -1/(2*sig^2)
	wSum = 0.
	dotProd = 0.
	for j=minInd:maxInd
		w = coef*exp(den*(timeHist[j]-t)^2)
		wSum += w
		dotProd += w*rawParam[j]
	end

	return wSum, dotProd

end

# function getFiltWeights!(w::Array{Float64}, timeHist::DataArray{Float64}, t::Float64, minInd::Int64, 
# 						 maxInd::Int64, sig::Float64)

# 	# Calculates the weights of a gaussian smoothing filter:
# 	coef = 1/(sig*sqrt(2*pi))
# 	den = -1/(2*sig^2)
# 	for j=minInd:maxInd
# 		w[j] = coef*exp(den*(timeHist[j]-t)^2)
# 	end

# end

function interpolateHistories(datArray::DataFrame; filterParams::FilterParams=FilterParams())
#  Makes sure we have data at consistent intervals.  Use something like:
# itp=interpolate(A,BSpline(Quadratic(Flat())),OnGrid())
# See: https://github.com/tlycken/Interpolations.jl
# Unfortunately, to use the BSpline methods (which look pretty nice, at least on a sin() approx) the
# "knots" must be evenly spaced.  I definitely can't do that since my grid points/knots are based on
# the actual time stamps received.  Instead I can specify knots, but then I can only use linear interpolation,
# not the spline-based.  This doesn't look very good, but perhaps as a first pass to start trying to train 
# the bayes nets I can use linear?  In the meantime I can develop or find an interpolation scheme that works
# with irregularly space grid points.

	# Pre allocate space, if possible, using dt and time history data.
	timeHist = convert(Array, datArray[:,symbol(filterParams.timeStr)])
	timeHistInterp = Float64[minimum(timeHist):filterParams.tInterval:maximum(timeHist);]
	datArrayInterp = DataFrame()
	numTimeElems = length(timeHistInterp)

#	floor(Int64, (maximum(timeHist)-minimum(timeHist))/filterParams.tInterval)
 	for prm in filterParams.params2Smooth
 		datArrayInterp[symbol(prm)]=DataArray(Float64,numTimeElems)
 		#datArray[:,symbol(ud[i])] = NA
 	end
 	datArrayInterp[symbol(filterParams.timeStr)]=timeHistInterp

 	try
		for prm in filterParams.params2Smooth
			rawParam = convert(Array, datArray[:,symbol(prm)])

			if filterParams.useSplineFit
				spl = Spline1D(timeHist, rawParam, w=ones(length(timeHist)), k=filterParams.degSpline, bc=filterParams.bcSpline, s=filterParams.smthFactorSpline)
				datArrayInterp[symbol(prm)] = spl([timeHistInterp])
			else
				itp=interpolate((timeHist,), rawParam, Gridded(Linear())) 
				datArrayInterp[symbol(prm)] = itp[timeHistInterp]
			end
		end
	catch
		display("Error on spline interpolation, not interpolating this file.")
		if sort(timeHist)==timeHist
			display("Time hist is in sorted order")
		else
			display("Time hist not sorted correcty")
		end

		return DataFrame[]
	end

	return datArrayInterp

end

function plotTLogData(tlogDatArray::DataFrame; filterParams::FilterParams=FilterParams())
	plotTLogData([tlogDatArray], filterParams=filterParams)
end

function plotTLogData(tlogDatArray::Array{DataFrame}; filterParams::FilterParams=FilterParams())

	# filterParams.
	# timeStr = "time_unix_usec_._mavlink_system_time_t"
	# latStr = "lat_._mavlink_global_position_int_t"
	# lonStr = "lon_._mavlink_global_position_int_t"
	# altStr = "alt_._mavlink_global_position_int_t"
	# vxStr = "vx_._mavlink_global_position_int_t"
	# vyStr = "vy_._mavlink_global_position_int_t"

	for i=1:length(tlogDatArray)

		# Get the key variables:
		timeHist = convert(Array, tlogDatArray[i][:,symbol(filterParams.timeStr)])
		lat = convert(Array, tlogDatArray[i][:,symbol(filterParams.latStr)])
		lon = convert(Array, tlogDatArray[i][:,symbol(filterParams.lonStr)])
		alt = convert(Array, tlogDatArray[i][:,symbol(filterParams.altStr)])
		vel = convert(Array,(tlogDatArray[i][:,symbol(filterParams.vxStr)].^2+tlogDatArray[i][:,symbol(filterParams.vyStr)].^2).^0.5)

		# Plot the x/y locations from the lat/long
		x,y = convertLL2XY(lat,lon)
		figure()
		plot(x, y,"g")
		xlabel("X (ft)")
		ylabel("Y (ft)")

		# Plot alt vs. time		
		figure()
		plot(timeHist-timeHist[1], alt, "r")
		xlabel("Time (s)")
		ylabel("Altitude (ft)")

		# Plot velocity vs. time
		#timeHist = convert(Array, tlogDatArray[i][:,symbol(filterParams.timeStr)])
		
		figure()
		plot(timeHist-timeHist[1], vel, "b")
		xlabel("Time (s)")
		ylabel("Velocity (kts)")

		# Plot vertical velocity (dh/dt) vs. time
		#timeHist = convert(Array, tlogDatArray[i][:,symbol(filterParams.timeStr)])
		#alt = convert(Array, tlogDatArray[i][:,symbol(filterParams.altStr)])
		dh = diff(alt)
		dt = diff(timeHist)
		dhdt = dh./dt .* 60.  # Convert from ft/s to ft/min
		figure()
		plot(timeHist[2:end]-timeHist[1], dhdt, "g")
		xlabel("Time (s)")
		ylabel("Vertical Velocity, dh/dt (ft/min)")

		# Plot time steps:
		#timeHist = tlogDatArray[i][:,symbol(filterParams.timeStr)]
		#dt = diff(timeHist)
		figure()
		plot(timeHist[2:end]-timeHist[1], dt)
		xlabel("Time (s)")
		ylabel("Time between consecutive time stamps (s)")
		ylim([0, 2*maximum(dt)])
	end

	return 0

end	

# The following are deprecated, use the version that passes in FilterParams rather than timeKey
function plotTLogData(tlogDatArray::DataFrame, timeKey::AbstractString)
	plotTLogData([tlogDatArray], timeKey)
end

function plotTLogData(tlogDatArray::Array{DataFrame}, timeKey::AbstractString)

	latrawStr = "lat_._mavlink_gps_raw_int_t"
    lonrawStr = "lon_._mavlink_gps_raw_int_t"
	altrawStr = "alt_._mavlink_gps_raw_int_t"
	velrawStr = "vel_._mavlink_gps_raw_int_t"

	latglbStr = "lat_._mavlink_global_position_int_t"
    longlbStr = "lon_._mavlink_global_position_int_t"
	altglbStr = "alt_._mavlink_global_position_int_t"

	vxStr = "vx_._mavlink_global_position_int_t"
	vyStr = "vy_._mavlink_global_position_int_t"
	vzStr = "vz_._mavlink_global_position_int_t"
	hdgStr = "hdg_._mavlink_global_position_int_t"

	aspdStr = "airspeed_._mavlink_vfr_hud_t"
	climbStr = "climb_._mavlink_vfr_hud_t"


	for i=1:length(tlogDatArray)

		# # Plot the raw lat/lon traces:
		# goodInds = collect(1:size(tlogDatArray[i],1))
		# latInds = find(isna(tlogDatArray[i][:,symbol(latrawStr)]))
		# lonInds = find(isna(tlogDatArray[i][:,symbol(lonrawStr)]))
		# naInds = unique([latInds,lonInds])
		# #display("i=$i, $naInds")
		# deleteat!(goodInds,naInds)

		# lon = convert(Array, tlogDatArray[i][goodInds,symbol(lonrawStr)])./10^7
		# lat = convert(Array, tlogDatArray[i][goodInds,symbol(latrawStr)])./10^7
		# figure()
		# # plot(lon-mean(lon), lat-mean(lat),"r")
		# plot(lon, lat,"r")
		# xlabel("Longitude (??)")
		# ylabel("Latitude (??)")

		# Now overplot the gps lat/lon traces
		goodInds = collect(1:size(tlogDatArray[i],1))
		latInds = find(isna(tlogDatArray[i][:,symbol(latglbStr)]))
		lonInds = find(isna(tlogDatArray[i][:,symbol(longlbStr)]))
		naInds = unique([latInds,lonInds])
		deleteat!(goodInds,naInds)
		# lon = convert(Array, tlogDatArray[i][goodInds,symbol(longlbStr)])./10^7
		# lat = convert(Array, tlogDatArray[i][goodInds,symbol(latglbStr)])./10^7
		lon = convert(Array, tlogDatArray[i][goodInds,symbol(longlbStr)])
		lat = convert(Array, tlogDatArray[i][goodInds,symbol(latglbStr)])
		#plot(lon, lat,"b")

		# Plot x and y:
		# Use x and y converted from lat/long global positon, lat_._mavlink_global_position_int_t
		x,y = convertLL2XY(lat,lon)
		figure()
		# plot(lon-mean(lon), lat-mean(lat),"r")
		plot(x, y,"g")
		xlabel("X (ft)")
		ylabel("Y (ft)")

		# Plot alt vs. time
		# Not sure what to do about altitude.  GPS altitude seems a bit better, but frequently
		# we've got altitudes of something like 6000 m.  And we get altitude changes that wolud
		# require climb rates of 100 m/s. Might there be a different scaling factor for some files?
		goodInds = collect(1:size(tlogDatArray[i],1))
		altInds = find(isna(tlogDatArray[i][:,symbol(altrawStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([altInds, timeInds])
		deleteat!(goodInds,naInds)

		# alt = convert(Array, tlogDatArray[i][goodInds,symbol(altrawStr)])./10^2
		# timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		alt = convert(Array, tlogDatArray[i][goodInds,symbol(altrawStr)])
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])
		figure()
		plot(timeHist-timeHist[1], alt, "r")
		xlabel("Time (s)")
		ylabel("Altitude (ft)")

		# Now overplot the gps altitude:
		# Gps altitude better?
		goodInds = collect(1:size(tlogDatArray[i],1))
		altInds = find(isna(tlogDatArray[i][:,symbol(altglbStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([altInds, timeInds])
		deleteat!(goodInds,naInds)
		# alt = convert(Array, tlogDatArray[i][goodInds,symbol(altglbStr)])./10^2
		# timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		alt = convert(Array, tlogDatArray[i][goodInds,symbol(altglbStr)])
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])
		plot(timeHist-timeHist[1], alt, "b")

		# Plot velocity vs. time
		# use vx and vy components to get net velocity
		goodInds = collect(1:size(tlogDatArray[i],1))
		velInds = find(isna(tlogDatArray[i][:,symbol(velrawStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, velInds])
		deleteat!(goodInds,naInds)
		# timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		# vel = convert(Array, tlogDatArray[i][goodInds,symbol(velrawStr)])./10^2
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])
		vel = convert(Array, tlogDatArray[i][goodInds,symbol(velrawStr)])
		figure()
		plot(timeHist-timeHist[1], vel, "r")
		xlabel("Time (s)")
		ylabel("Velocity (kts)")

		# Plot magnitude of components of velocity (vx,vy) 
		goodInds = collect(1:size(tlogDatArray[i],1))
		vxInds = find(isna(tlogDatArray[i][:,symbol(vxStr)]))
		vyInds = find(isna(tlogDatArray[i][:,symbol(vyStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, vxInds, vyInds])
		deleteat!(goodInds,naInds)
		# timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])
		velGlobal = (tlogDatArray[i][goodInds,symbol(vxStr)].^2+tlogDatArray[i][goodInds,symbol(vyStr)].^2).^0.5
		# vel = convert(Array, velGlobal)./10^2
		vel = convert(Array, velGlobal)
		plot(timeHist-timeHist[1], vel, "b")

		# Plot airspeed vs. time 
		goodInds = collect(1:size(tlogDatArray[i],1))
		aspdInds = find(isna(tlogDatArray[i][:,symbol(aspdStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, aspdInds])
		deleteat!(goodInds,naInds)
		# timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])
		vel = convert(Array, tlogDatArray[i][goodInds,symbol(aspdStr)])
		plot(timeHist-timeHist[1], vel, "g")
		legend(("Raw vel","Global vel","Airspeed"))

		# Plot vertical gps velocity vs. time (probably won't be good)
		goodInds = collect(1:size(tlogDatArray[i],1))
		vvelInds = find(isna(tlogDatArray[i][:,symbol(vzStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, vvelInds])
		deleteat!(goodInds,naInds)
		# timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		# vel = convert(Array, tlogDatArray[i][goodInds,symbol(vzStr)])./10^2
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])
		vel = convert(Array, tlogDatArray[i][goodInds,symbol(vzStr)])
		figure()
		plot(timeHist-timeHist[1], vel, "b")
		xlabel("Time (s)")
		ylabel("Vertical Velocity (ft/min)")

		# Plot Climb vs. time 
		goodInds = collect(1:size(tlogDatArray[i],1))
		climbInds = find(isna(tlogDatArray[i][:,symbol(climbStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, climbInds])
		deleteat!(goodInds,naInds)
		# timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])
		vel = convert(Array, tlogDatArray[i][goodInds,symbol(climbStr)])
		plot(timeHist-timeHist[1], vel, "r")

		# Plot dh/dt vs. time 
		goodInds = collect(1:size(tlogDatArray[i],1))
		altInds = find(isna(tlogDatArray[i][:,symbol(altglbStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, altInds])
		deleteat!(goodInds,naInds)
		# timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		# alt = convert(Array, tlogDatArray[i][goodInds,symbol(altglbStr)])./10^2
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])
		alt = convert(Array, tlogDatArray[i][goodInds,symbol(altglbStr)])
		dh = diff(alt)
		dt = diff(timeHist)
		dhdt = dh./dt .*1./60.  # Convert from ft/s to ft/min
		plot(timeHist[2:end]-timeHist[1], dhdt, "g")
		legend(("GPS vz", "Climb rate", "dh/dt"))

		# Plot time steps:
		# timeHist = tlogDatArray[i][:,symbol(timeKey)]./10^6
		timeHist = tlogDatArray[i][:,symbol(timeKey)]
		dt = diff(timeHist)
		figure()
		plot(timeHist[2:end]-timeHist[1], dt)
		xlabel("Time (s)")
		ylabel("Time between consecutive time stamps (s)")
	end


	latStr = "lat_._mavlink_global_position_int_t"
	lonStr = "lon_._mavlink_global_position_int_t"
	altStr = "alt_._mavlink_global_position_int_t"

	return 0

end	

function convertLL2XY(lat,lon)
# Uses the first point as the origin, then uses simple curvilinear approx to get x,y
	Re = 6371000*3.28
	initLat = lat[1]
	initLon = lon[1]

	lat2ft = 2*pi*Re/360
	lon2ft = 2*pi*Re*cosd(initLat)/360

	latShift = lat.-initLat
	lonShift = lon.-initLon

	x = lonShift.*lon2ft
	y = latShift.*lat2ft

	return x,y
end	

# Found new key: time_unix_usec_._mavlink_system_time_t
# Found new key: time_boot_ms_._mavlink_system_time_t
# Found new key: custom_mode_._mavlink_heartbeat_t
# Found new key: type_._mavlink_heartbeat_t
# Found new key: autopilot_._mavlink_heartbeat_t
# Found new key: base_mode_._mavlink_heartbeat_t
# Found new key: system_status_._mavlink_heartbeat_t
# Found new key: mavlink_version_._mavlink_heartbeat_t
# Found new key: req_message_rate_._mavlink_request_data_stream_t
# Found new key: target_system_._mavlink_request_data_stream_t
# Found new key: target_component_._mavlink_request_data_stream_t
# Found new key: req_stream_id_._mavlink_request_data_stream_t
# Found new key: start_stop_._mavlink_request_data_stream_t
# Found new key: target_system_._mavlink_param_request_list_t
# Found new key: target_component_._mavlink_param_request_list_t
# Found new key: rxerrors_._mavlink_radio_t
# Found new key: fixed_._mavlink_radio_t
# Found new key: rssi_._mavlink_radio_t
# Found new key: remrssi_._mavlink_radio_t
# Found new key: txbuf_._mavlink_radio_t
# Found new key: noise_._mavlink_radio_t
# Found new key: remnoise_._mavlink_radio_t
# Found new key: rxerrors_._mavlink_radio_status_t
# Found new key: fixed_._mavlink_radio_status_t
# Found new key: rssi_._mavlink_radio_status_t
# Found new key: remrssi_._mavlink_radio_status_t
# Found new key: txbuf_._mavlink_radio_status_t
# Found new key: noise_._mavlink_radio_status_t
# Found new key: remnoise_._mavlink_radio_status_t
# Found new key: time_usec_._mavlink_raw_imu_t
# Found new key: xacc_._mavlink_raw_imu_t
# Found new key: yacc_._mavlink_raw_imu_t
# Found new key: zacc_._mavlink_raw_imu_t
# Found new key: xgyro_._mavlink_raw_imu_t
# Found new key: ygyro_._mavlink_raw_imu_t
# Found new key: zgyro_._mavlink_raw_imu_t
# Found new key: xmag_._mavlink_raw_imu_t
# Found new key: ymag_._mavlink_raw_imu_t
# Found new key: zmag_._mavlink_raw_imu_t
# Found new key: time_boot_ms_._mavlink_scaled_pressure_t
# Found new key: press_abs_._mavlink_scaled_pressure_t
# Found new key: press_diff_._mavlink_scaled_pressure_t
# Found new key: temperature_._mavlink_scaled_pressure_t
# Found new key: time_boot_ms_._mavlink_attitude_t
# Found new key: roll_._mavlink_attitude_t
# Found new key: pitch_._mavlink_attitude_t
# Found new key: yaw_._mavlink_attitude_t
# Found new key: rollspeed_._mavlink_attitude_t
# Found new key: pitchspeed_._mavlink_attitude_t
# Found new key: yawspeed_._mavlink_attitude_t
# Found new key: airspeed_._mavlink_vfr_hud_t
# Found new key: groundspeed_._mavlink_vfr_hud_t
# Found new key: alt_._mavlink_vfr_hud_t
# Found new key: climb_._mavlink_vfr_hud_t
# Found new key: heading_._mavlink_vfr_hud_t
# Found new key: throttle_._mavlink_vfr_hud_t
# Found new key: omegaIx_._mavlink_ahrs_t
# Found new key: omegaIy_._mavlink_ahrs_t
# Found new key: omegaIz_._mavlink_ahrs_t
# Found new key: accel_weight_._mavlink_ahrs_t
# Found new key: renorm_val_._mavlink_ahrs_t
# Found new key: error_rp_._mavlink_ahrs_t
# Found new key: error_yaw_._mavlink_ahrs_t
# Found new key: Vcc_._mavlink_hwstatus_t
# Found new key: I2Cerr_._mavlink_hwstatus_t
# Found new key: onboard_control_sensors_present_._mavlink_sys_status_t
# Found new key: onboard_control_sensors_enabled_._mavlink_sys_status_t
# Found new key: onboard_control_sensors_health_._mavlink_sys_status_t
# Found new key: load_._mavlink_sys_status_t
# Found new key: voltage_battery_._mavlink_sys_status_t
# Found new key: current_battery_._mavlink_sys_status_t
# Found new key: drop_rate_comm_._mavlink_sys_status_t
# Found new key: errors_comm_._mavlink_sys_status_t
# Found new key: errors_count1_._mavlink_sys_status_t
# Found new key: errors_count2_._mavlink_sys_status_t
# Found new key: errors_count3_._mavlink_sys_status_t
# Found new key: errors_count4_._mavlink_sys_status_t
# Found new key: battery_remaining_._mavlink_sys_status_t
# Found new key: brkval_._mavlink_meminfo_t
# Found new key: freemem_._mavlink_meminfo_t
# Found new key: seq_._mavlink_mission_current_t
# Found new key: time_usec_._mavlink_gps_raw_int_t
# Found new key: lat_._mavlink_gps_raw_int_t
# Found new key: lon_._mavlink_gps_raw_int_t
# Found new key: alt_._mavlink_gps_raw_int_t
# Found new key: eph_._mavlink_gps_raw_int_t
# Found new key: epv_._mavlink_gps_raw_int_t
# Found new key: vel_._mavlink_gps_raw_int_t
# Found new key: cog_._mavlink_gps_raw_int_t
# Found new key: fix_type_._mavlink_gps_raw_int_t
# Found new key: satellites_visible_._mavlink_gps_raw_int_t
# Found new key: nav_roll_._mavlink_nav_controller_output_t
# Found new key: nav_pitch_._mavlink_nav_controller_output_t
# Found new key: alt_error_._mavlink_nav_controller_output_t
# Found new key: aspd_error_._mavlink_nav_controller_output_t
# Found new key: xtrack_error_._mavlink_nav_controller_output_t
# Found new key: nav_bearing_._mavlink_nav_controller_output_t
# Found new key: target_bearing_._mavlink_nav_controller_output_t
# Found new key: wp_dist_._mavlink_nav_controller_output_t
# Found new key: breach_time_._mavlink_fence_status_t
# Found new key: breach_count_._mavlink_fence_status_t
# Found new key: breach_status_._mavlink_fence_status_t
# Found new key: breach_type_._mavlink_fence_status_t
# Found new key: severity_._mavlink_statustext_t
# Found new key: param_value_._mavlink_param_value_t
# Found new key: param_count_._mavlink_param_value_t
# Found new key: param_index_._mavlink_param_value_t
# Found new key: param_type_._mavlink_param_value_t
# Found new key: time_boot_ms_._mavlink_global_position_int_t
# Found new key: lat_._mavlink_global_position_int_t
# Found new key: lon_._mavlink_global_position_int_t
# Found new key: alt_._mavlink_global_position_int_t
# Found new key: relative_alt_._mavlink_global_position_int_t
# Found new key: vx_._mavlink_global_position_int_t
# Found new key: vy_._mavlink_global_position_int_t
# Found new key: vz_._mavlink_global_position_int_t
# Found new key: hdg_._mavlink_global_position_int_t
# Found new key: time_usec_._mavlink_servo_output_raw_t
# Found new key: servo1_raw_._mavlink_servo_output_raw_t
# Found new key: servo2_raw_._mavlink_servo_output_raw_t
# Found new key: servo3_raw_._mavlink_servo_output_raw_t
# Found new key: servo4_raw_._mavlink_servo_output_raw_t
# Found new key: servo5_raw_._mavlink_servo_output_raw_t
# Found new key: servo6_raw_._mavlink_servo_output_raw_t
# Found new key: servo7_raw_._mavlink_servo_output_raw_t
# Found new key: servo8_raw_._mavlink_servo_output_raw_t
# Found new key: port_._mavlink_servo_output_raw_t
# Found new key: time_boot_ms_._mavlink_rc_channels_raw_t
# Found new key: chan1_raw_._mavlink_rc_channels_raw_t
# Found new key: chan2_raw_._mavlink_rc_channels_raw_t
# Found new key: chan3_raw_._mavlink_rc_channels_raw_t
# Found new key: chan4_raw_._mavlink_rc_channels_raw_t
# Found new key: chan5_raw_._mavlink_rc_channels_raw_t
# Found new key: chan6_raw_._mavlink_rc_channels_raw_t
# Found new key: chan7_raw_._mavlink_rc_channels_raw_t
# Found new key: chan8_raw_._mavlink_rc_channels_raw_t
# Found new key: port_._mavlink_rc_channels_raw_t
# Found new key: rssi_._mavlink_rc_channels_raw_t
# Found new key: param_index_._mavlink_param_request_read_t
# Found new key: target_system_._mavlink_param_request_read_t
# Found new key: target_component_._mavlink_param_request_read_t
# Found new key: mag_declination_._mavlink_sensor_offsets_t
# Found new key: raw_press_._mavlink_sensor_offsets_t
# Found new key: raw_temp_._mavlink_sensor_offsets_t
# Found new key: gyro_cal_x_._mavlink_sensor_offsets_t
# Found new key: gyro_cal_y_._mavlink_sensor_offsets_t
# Found new key: gyro_cal_z_._mavlink_sensor_offsets_t
# Found new key: accel_cal_x_._mavlink_sensor_offsets_t
# Found new key: accel_cal_y_._mavlink_sensor_offsets_t
# Found new key: accel_cal_z_._mavlink_sensor_offsets_t
# Found new key: mag_ofs_x_._mavlink_sensor_offsets_t
# Found new key: mag_ofs_y_._mavlink_sensor_offsets_t
# Found new key: mag_ofs_z_._mavlink_sensor_offsets_t
# Found new key: param1_._mavlink_command_long_t
# Found new key: param2_._mavlink_command_long_t
# Found new key: param3_._mavlink_command_long_t
# Found new key: param4_._mavlink_command_long_t
# Found new key: param5_._mavlink_command_long_t
# Found new key: param6_._mavlink_command_long_t
# Found new key: param7_._mavlink_command_long_t
# Found new key: command_._mavlink_command_long_t
# Found new key: target_system_._mavlink_command_long_t
# Found new key: target_component_._mavlink_command_long_t
# Found new key: confirmation_._mavlink_command_long_t