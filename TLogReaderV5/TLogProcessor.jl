using HDF5, JLD, DataFrames, DataArrays, PyPlot, Interpolations, Dierckx, BayesNets
import Requests: get, readall, save


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

# Planned upgrades:
# 1. Sort timeHist if it's not in sorted order?  OR would files with this problem have many other problems too?
# 2. Figure out the appropriate time sigma for filtering, curretly all 2 seconds.  That may be too short for velocity.
# 3. Implement a derivative function that can return arbitrary derivative orders using arbitrary number of points left/right of current point.  See https://commons.wikimedia.org/wiki/File:FDnotes.djvu
# 4. Remove the now-unnecessary parameters from the filterParams() structure.  keyParaams and params2Smooth
# 5. Add capability to individually generate features from one jld at a time, not compile everything into a huge array.  Will have to write the features out to a log file.
# 6. Remove outlier positions (calculated the way Mykel did in his paper, d_distance/dt), though velocity will not be directly used.  Thought that wasn't an issue, but have seen it at least once.
# 7. Switch to numerical derivatives of position to get velocity rather than reported vx and vy? There seem to be important discrepancies between those variables, and it would be best to be self-consistent.
# 8. Might need a maximum dropout rate.  IF two time stamps are too far apart (after removing positons, for example), we might need to break into two tracks  to avoid having a super-linear stretch.
# 9. Should make the number of standard deviations at which to remove a time stamp a parameter in filterParams. But I don't want to lose the ability to load the older jld files (yet).  Make sure I've fixed every problem then re-parse everything.
# 10. Double check the position filtering file, perhaps with 11335.tlog?  That seemed to have a lot of problems in it.  Why does it remove points when they have zero net velocity change?


# Work flow:
# 0. Writing a single function (convertTLogs2FeatureCSV) to read in all the tlog files, process them and output to a giant csv file. That should cover the two following steps.
# 0.1 processTlog() takes in a .tlog file/folder and writes the tlog data (not features) and filter parameters to a .jld file. If the number of input files is smaller than a user-selectable parameter (20), then it also returns an array of that data.
# 0.2 featureFolder2csv() will read in a set of .jld files and write to a single, giant csv file the features contained in the .jld tlog file.
# 3. plotTlogData() takes in a tlogDat data frame or array of data frames and plots the most interesting state variable histories.
# 4. The output csv can be loaded into GeNIe, which allows for Bayes Net structure searches.  It also allows easy binning of the data without losing it, 
         # and that's a lot easier than doing what I did here in julia.  The data should be binned appropriately and then a network learned from the 
         # parameters. Output that network as an xdsl file so it can be read by BayesNets.jl


# 4. You can manually store the features you want by loading the whole csv feature file into julia as a data frame and copying only those features you want into a new frame (see featureTable20151221.jld)
# 5. Look at the distribution of features (perhaps smoothing them or removing some, though I don't currently do this) and select appropriate bins.  I've done this for all but the t+1 variables in getHardCodedBin...  You can investigate the distributions with binFeatures()
# 6. Convert the features into bins with convertFeatures2BinMat().  This will store the features as the Int8 representation of which index in the histogram they are, which will be much smaller that the original feature file (for 550 flights, 82 MB rather than 1.02 GB)
# 7. Haven't yet written out the binned features to a csv, need to look into GeNie to see what it wants.  I think it can work directly on the feature values.
# 8. If you've got a bunch of jld files without terrain elevations, you can run addTerrain2JLD to lookup the elevations and add them to the jld files. You can then proceed with creation of the csv file (featureFolder2csv())
# 9. Make an option in featureFolder2CSV to output each jld as a csv of features rather than a single csv.

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

	maxDrDtThreshold::Float64 	# Maximum horizontal velocity between position reports (given dt) that is allowable.  Expressed in ft/s
	maxDhDtThreshold::Float64   # Maximum horizontal velocity between position reports (given dt) that is allowable.  Expressed in ft/s


	FilterParams(distanceOutlier, velocityOutlier, keyParams, latParam, lonParam, params2Smooth, smoothingSigma, tInterval, maxSmoothSigma, timeStr, latStr, lonStr, altStr, vxStr, vyStr, useSplineFit, degSpline, bcSpline, smthFactorSpline, maxDrDtThreshold, maxDhDtThreshold) = 
			 new(distanceOutlier, velocityOutlier, keyParams, latParam, lonParam, params2Smooth, smoothingSigma, tInterval, maxSmoothSigma, timeStr, latStr, lonStr, altStr, vxStr, vyStr, useSplineFit, degSpline, bcSpline, smthFactorSpline, maxDrDtThreshold, maxDhDtThreshold)
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

	maxDrDtThreshold = 300. * 6071./3600. 	# Maximum horizontal velocity between position reports (given dt) that is allowable.  Expressed in ft/s
	maxDhDtThreshold = 3000. * 1./60. 	    # Maximum horizontal velocity between position reports (given dt) that is allowable.  Expressed in ft/s

	return FilterParams(distanceOutlier, velocityOutlier, keyParams, latParam, lonParam, params2Smooth, smoothingSigma, tInterval, maxSmoothSigma, timeStr, latStr, lonStr, altStr, vxStr, vyStr, useSplineFit, degSpline, bcSpline, smthFactorSpline, maxDrDtThreshold, maxDhDtThreshold)
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

	maxDrDtThreshold = 200. * 6071./3600. 	# Maximum horizontal velocity between position reports (given dt) that is allowable.  Expressed in ft/s
	maxDhDtThreshold = 3000. * 1./60. 	    # Maximum horizontal velocity between position reports (given dt) that is allowable.  Expressed in ft/s

	return FilterParams(distanceOutlier, velocityOutlier, keyParams, latParam, lonParam, params2Smooth, smoothingSigma, tInterval, maxSmoothSigma, timeStr, latStr, lonStr, altStr, vxStr, vyStr, useSplineFit, degSpline, bcSpline, smthFactorSpline, maxDrDtThreshold, maxDhDtThreshold)
end

type BNParams
	# Holds the discretized bin sizes for the Bayes net.  It would be nice to be able to read this in directly from the xdsl file,
	# but it doesn't appear to be stored there.

	binDisc::Dict

	BNParams(binDisc) = new(binDisc)

end

function BNParams()
# This discretization serves two functions: allows us to bin the continuous data before using Kyle's program (if we use that), and
# do continuous sampling within a bin once a set of discrete bins has been selected from the BN.  To do the latter, we need outer limits
# on the bin sizes, which adds two values to each discretization.  During the former binning, the first and last elements of each of
# these arrays is ignored and -Inf and +Inf are used instead (so no data is left out). 

# A limitation of GeNIe is that discretization has to happen from zero, it cannot "wrap around".  This is a problem for states that
# represent angles because, for example, we'd like to bin everything from -22.5 to 22.5 in one bin, 22.5 to 67.5 in the second, etc.
# I should be able to add this capability myself if I do the binning.

	binDisc = Dict(
		:range     => [0., 11., 29., 101., 822., 10000.],
		:rrHeading => [0., 45., 90., 135., 180., 225., 270., 315., 360],
		:rrHeaddot =>  [-50., -8.698, -0.0324, 0.032, 8.079, 50.],
		:altAGL    => [0., 10., 50., 100., 400., 5000.],
		:altdot    => [-5, -0.3921, -0.060, 0.061, 0.41447, 5],
		:dxdt      =>  [0., 0.03, 0.10, 0.30, 1.75, 40.],
		:d2xdt2    => [-5., -0.46, -5.11e-6, 4.906e-5, 0.464, 5.],  # This may need to be narrower for the 2nd and 5th bins.
		:d2xdt2_tp1    => [-5., -0.46, -5.11e-6, 4.906e-5, 0.464, 5.],
		:altdot_tp1    => [-5, -0.3921, -0.060, 0.061, 0.41447, 5],
		:rrHeaddot_tp1 =>  [-50., -8.698, -0.0324, 0.032, 8.079, 50.])

		return BNParams(binDisc)
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

# Units of features are as follows.  These features are used to create the Bayes Nets and then sample from them, so it is
# critical to know what units they are in.  It is advised that one convert from these aviation units to other (metric or basic English)
# units before proceeding with any calculations.
# :range = ft
# :vel = kts
# :bearing = degrees
# :heading = degrees
# :rrHeading = degrees
# :rrHeaddot = degrees/s
# :vdot  = kts/s
# :alt = ft
# :altAGL = ft
# :altdot = ft/s
# :dxdt = ft/s
# :d2xdt2 = ft/s^2
# :pathDist = ft
# :altdot_tp1 = ft/s
# :d2xdt2_tp1 = ft/s^2
# :rrHeaddot_tp1 = degrees/s


function convertTLogs2FeatureCSV(;readpathTLog::AbstractString="./", writepathJLD::AbstractString="./", createTextLogs::Bool=false, 
					  jldSaveNameBase::AbstractString="tlogData", fieldMatchFile::AbstractString="FieldMatchFile.txt", 
					  VERBOSE::Bool=false, timeKey::AbstractString="time_unix_usec_._mavlink_system_time_t",
					  maxTlogsinArray::Int64=20, tlogParseExe::AbstractString="./TLogReader.exe", 
					  filterParams::FilterParams=FilterParams(), successPathTLog::AbstractString="", failPathTLog::AbstractString="",
					  writeFileCSV::AbstractString="./featureHistory", outputType::AbstractString=".csv", redoElevations::Bool=false,
					  elevMethod::AbstractString="centroid", prependElevJLD::AbstractString="")
	# Single function that batch processes tlog files, moving them if desired in the process, then extracts the features and 
	# writes to a CSV file that is suitable for GeNie.  Basically lumps in two functions, processTLog (inputs tlog files,
	# outputs .jld files with basic track info) and featureFolder2CSV (inputs tracks, outputs a CSV of features).  If you decide you need
	# to call the elevation database again (perhaps your jld files didn't originally have elevation information), then set redoElevations 
	# to true.  In this case you may also specify the elevation method to use and what to prepend to the new files to distinguish them
	# from the old JLD files.  If you specify "" then the old files will be over-written.
	
	# Basic operation:
	#  tlogDatArray, successParses, failParses = convertTLogs2FeatureCSV(readpath="./", writepath="./jldfiles/", createTextLogs=false, jldSaveNameBase="tlogData", fieldMatchFile="FieldMatchFile.txt", VERBOSE=false, timeKey="time_unix_usec_._mavlink_system_time_t",successPath="./convertedTLogs/", failPath="./failedTLogs/",VERBOSE=true, tlogParseExe="../TLogReader.exe");

	# Eric's version for batch processing:
	# readpathTLog="/home/emueller/ACAS/EncounterModel/Data/tlog/"
	# writepathJLD="/home/emueller/ACAS/EncounterModel/Data/jldData/"
	# fieldMatchFile="FieldMatchFile.txt"    # Remember, this gets readpathTLog prepended to it (probably shouldn't but it does)
	# tlogParseExe="/home/emueller/.julia/v0.3/CA2DRP/src/TLogReader.exe"
	# successPathTLog="/home/emueller/ACAS/EncounterModel/Data/tlog/successTLogConvert/"
	# failPathTLog="/home/emueller/ACAS/EncounterModel/Data/tlog/failTLogConvert/"
	# writeFileCSV="/home/emueller/ACAS/EncounterModel/Data/FeatureHistory20151228"
	# tlogDatArray, successParses, failParses, successAltitudes = convertTLogs2FeatureCSV(readpathTLog=readpathTLog, writepathJLD=writepathJLD, fieldMatchFile=fieldMatchFile, VERBOSE=true, tlogParseExe=tlogParseExe,successPathTLog=successPathTLog,failPathTLog=failPathTLog, writeFileCSV=writeFileCSV, redoElevations=false);


	tlogDatArray, successParses, failParses = processTLog(readpath=readpathTLog, writepath=writepathJLD, createTextLogs=createTextLogs, 
												  jldSaveNameBase=jldSaveNameBase, fieldMatchFile=fieldMatchFile, 
												  VERBOSE=VERBOSE, timeKey=timeKey, maxTlogsinArray=maxTlogsinArray, tlogParseExe=tlogParseExe, 
												  filterParams=filterParams, successPath=successPathTLog, failPath=failPathTLog)

	if VERBOSE
		display("Completed conversion of tlogs to tracks in jld files.")
		display("Out of $(successParses+failParses) files, $(successParses) were successfully parsed and $failParses were not.")
	end

	if redoElevations
		addTerrain2JLD(readPath=writepathJLD, writePath=writepathJLD, elevMethod=elevMethod, jldSaveNameBase=prependElevJLD, VERBOSE=VERBOSE)
	end

	successAltitudes = featureFolder2csv(readPath=writepathJLD, writeFile=writeFileCSV, outputType=outputType, VERBOSE=VERBOSE)

	return tlogDatArray, successParses, failParses, successAltitudes
end

function extractFeatures(tlogDatArray::DataFrame; filterParams::FilterParams=FilterParams(), terrainElev::Float64=NaN)
	return extractFeatures([tlogDatArray], filterParams=filterParams, terrainElev=(DataArray([terrainElev]),))[1]
end

function extractFeatures(tlogDatArray::DataFrame; filterParams::FilterParams=FilterParams(), terrainElev::Array{Float64}=Float64[])
	return extractFeatures([tlogDatArray], filterParams=filterParams, terrainElev=(DataArray(terrainElev),))[1]
end

function extractFeatures(tlogDatArray::DataFrame; filterParams::FilterParams=FilterParams(), terrainElev::DataArray=DataArray())
	return extractFeatures([tlogDatArray], filterParams=filterParams, terrainElev=(terrainElev,))[1]
end


function extractFeatures(tlogDatArray::Array{DataFrame}; filterParams::FilterParams=FilterParams(), terrainElev::Tuple{DataArray}=Tuple(DataArray[],))
	# This function extracts a set of "features" from the raw tlog data.  The set is currently large because
	# I don't know which features I'l lend up using.

	featureArray = Array(DataFrame,length(tlogDatArray))
	# for i=1:length(ud) 
 # 		datArray[symbol(ud[i])]=DataArray(Float64,numTimeElems)
 # 		#datArray[:,symbol(ud[i])] = NA
 # 	end

	for i=1:length(tlogDatArray)
		featureMat = DataFrame()

		timeHist = convert(Array, tlogDatArray[i][:,symbol(filterParams.timeStr)])
		lat = convert(Array, tlogDatArray[i][:,symbol(filterParams.latStr)])
		lon = convert(Array, tlogDatArray[i][:,symbol(filterParams.lonStr)])
		alt = convert(Array, tlogDatArray[i][:,symbol(filterParams.altStr)])
		vel = convert(Array,(tlogDatArray[i][:,symbol(filterParams.vxStr)].^2+tlogDatArray[i][:,symbol(filterParams.vyStr)].^2).^0.5)
		vx = convert(Array,(tlogDatArray[i][:,symbol(filterParams.vxStr)]))
		vy = convert(Array,(tlogDatArray[i][:,symbol(filterParams.vyStr)]))

		# Create range feature:
		x,y = convertLL2XY(lat,lon)
		#rangeFeature = sqrt(x.^2+y.^2)
		featureMat[:range] = DataArray(sqrt(x.^2+y.^2))[1:end-1]

		# Create velocity feature:
		featureMat[:vel] = DataArray(vel)[1:end-1]

		# Create bearing from origin feature (CW from North in degrees)
		bearingFeature = 180/pi * DataArray(atan2(x,y))
		featureMat[:bearing] = bearingFeature[1:end-1]

		# Create radial-relative heading (directly away from origin is zero, directly to origin is 180, perpendicular to radius is +-90 deg)
		headingFeature = 180/pi * DataArray(atan2(vx,vy))
		featureMat[:heading] = headingFeature[1:end-1]
		rrHeadingFeature = mod(headingFeature-bearingFeature + 360, 360)
		featureMat[:rrHeading] = rrHeadingFeature[1:end-1]

		# Create acceleration feature using central difference:
		featureMat[:vdot] = DataArray(getDeriv(vel, timeHist, n=1, k=1))[1:end-1]

		# Create radial-relative heading rate feature vector:
		rrHeaddotFeature = DataArray(getDeriv(convert(Array,rrHeadingFeature), timeHist, n=1, k=1))
		featureMat[:rrHeaddot] = rrHeaddotFeature[1:end-1]

		# Create altitude feature (currently MSL altitude, which isn't good)
		featureMat[:alt] = DataArray(alt)[1:end-1]

		# Create altitude above ground level feature 
		featureMat[:altAGL] = featureMat[:alt] .- terrainElev[i][1]

		# Create altitude rate feature (currently MSL altitude, which isn't good)
		altdotFeature = DataArray(getDeriv(alt, timeHist, n=1, k=1))
		featureMat[:altdot] = altdotFeature[1:end-1]

		# Create a velocity feature based on the positions and time stamps, not the reported velocity (I fear the reported velocity may be inconsistent with the position updates...)
		dx = diff(x)
		dy = diff(y)
		dxy = cumsum([0; sqrt(dx.^2+dy.^2)])
		dxdtFeature =  DataArray(getDeriv(dxy, timeHist, n=1, k=1) * 3600/6071)
		featureMat[:dxdt] = dxdtFeature[1:end-1]
		
		# Get the acceleration based on position estimates:
		d2xdt2Feature = DataArray(getDeriv(convert(Array,dxdtFeature), timeHist, n=1, k=1)) 
		featureMat[:d2xdt2] = d2xdt2Feature[1:end-1]   # Acceleration will be in knots per second (nmi/hour/sec)

		# Since I'm calculating the path distance to get velocity anyway, might as well record it (unless it takes up too much space)
		featureMat[:pathDist] = DataArray(dxy)[1:end-1]

		# Now get the three next time step features, altdot_t+1, d2xdt2_t+1, rrHeadDot_t+1
		featureMat[:altdot_tp1] = altdotFeature[2:end]
		featureMat[:d2xdt2_tp1] = d2xdt2Feature[2:end] 
		featureMat[:rrHeaddot_tp1] = rrHeaddotFeature[2:end]


		featureArray[i] = featureMat
	end

	return featureArray

end

function addAccel2Features!(featureArray::DataFrame; filterParams::FilterParams=FilterParams())
	# This operates directly on the featureArray data frame.  Terrain does'nt work this way because you need the track information to get elevations, which isn't in the features.

	#if !in(:d2xdt2,names(featureArray))
		featureArray[:d2xdt2] = zeros(Float64,size(featureArray,1))
		flIds = unique(featureArray[:FlightNum])

		for i=1:length(flIds)
			inds = find(featureArray[:,:FlightNum].==flIds[i])
			dxdt = convert(Array,featureArray[inds,:dxdt])
			featureArray[inds,:d2xdt2] = DataArray(getDeriv(dxdt, [0.; cumsum(filterParams.tInterval*ones(Float64,length(dxdt)))], n=1, k=1))
		end
	#end

	# return featureArray
end

function addTerrain2JLD(; readPath::AbstractString="./", writePath::AbstractString=readPath, elevMethod::AbstractString="centroid", 
						  jldSaveNameBase::AbstractString="agl", VERBOSE::Bool=false)
	# This function is necessary to add terrain data to jld files generated without it.  After about 12/29/15, TLogProcess() should
	# automatically add terrain elevation to the jld files as a new column in tlogDatInterp.

	fileList = readdir(readPath)
	inputFileName = AbstractString{}
	n = length(fileList)

	for i=1:n
		if length(fileList[i])>4
	 		if fileList[i][end-2:end] == "jld"	 			
	 			inputFileName = fileList[i][1:end-4]
	 			jldSaveName = string(jldSaveNameBase, inputFileName, ".jld")
	 			tlogDatInterp, filterParams, terrainElev = loadTLog(string(readPath,fileList[i]), VERBOSE=VERBOSE)

	 			terrainElev = getTerrainElevation(tlogDatInterp[:,symbol(filterParams.latStr)], tlogDatInterp[:,symbol(filterParams.lonStr)], elevMethod="centroid")

	 			JLD.save(string(writePath,jldSaveName), "tlogDatInterp", tlogDatInterp, "filterParams", filterParams, "terrainElev", terrainElev)
	 			if VERBOSE
		 			display("Added terrain elevation to jld file $i/$n: $(fileList[i]). Wrote to $(string(writePath,jldSaveName)), elevation = $terrainElev")
		 		end
	 		end
	 	end
	 end	


end

function tlogFolder2csv(;readPath::AbstractString="./", writeFile::AbstractString="./tlogHistory", outputType::AbstractString=".csv", VERBOSE::Bool=false)
	# Will save all jld tlog data contained in readPath in the csv file specified by writefile

	tlogDatArray = DataFrame[]
	fParamArray = FilterParams[]
	fileList = readdir(readPath)
	firstJLD = true
	firstNames = Symbol[]
	ind = 1

	for i=1:length(fileList)
		if length(fileList[i])>5
	 		if (fileList[i][end-2:end] == "jld")
	 			if VERBOSE
		 			display("Loading jld file $(fileList[i]) and writing to $(string(writeFile,outputType))")
				end
				tlogDatInterp, filterParams, terrainElev, features = loadTLog(string(readPath,fileList[i]), VERBOSE=VERBOSE)
				if !isempty(terrainElev)
					if length(terrainElev)>1
						tlogDatInterp[:,symbol("terrainElev")] = terrainElev
					else
						tlogDatInterp[:,symbol("terrainElev")] = terrainElev*ones(Float64, size(tlogDatInterp,1))
					end
				end
				tlogDatInterp[:,symbol("FlightNum")] = ind*ones(Int64, size(tlogDatInterp,1))
				ind += 1
				namestlog = names(tlogDatInterp)

				# Need to include a header only in the first call to write table.  Should also check to make sure the names are all the same:
				if firstJLD
					firstJLD = false
					firstNames = namestlog
					writetable(string(writeFile,outputType),tlogDatInterp,append=false,header=true)
				else
					# Check whether the names match the first names (will continue, however):
					if any(namestlog.!=firstNames)
						display("Warning, the names in file $(fileList[i]) don't match the names in the original output file:")
						display("Names in original file: $firstNames")
						display("Names in $(fileList[i]): $namestlog")
					end
					writetable(string(writeFile,outputType),tlogDatInterp,append=true,header=false)
				end

			end
		end
	end

	return 0
end

function featureFolder2csv(;readPath::AbstractString="./", writeFile::AbstractString="./featureHistory", outputType::AbstractString=".csv", 
		  					VERBOSE::Bool=false, saveFeaturesToJLD::Bool=true)
	# Will save all jld tlog data contained in readPath in the csv file specified by writefile

	tlogDatArray = DataFrame[]
	fParamArray = FilterParams[]
	fileList = readdir(readPath)
	firstJLD = true
	firstNames = Symbol[]
	ind = 1
	n = length(fileList)
	successAltitudes = 0

	for i=1:n
		if length(fileList[i])>5
	 		if (fileList[i][end-2:end] == "jld")
	 			if VERBOSE
		 			display("Loading jld file $i/$n: $(string(readPath,fileList[i])). Writing to $(string(writeFile,outputType))")
				end
				tlogDatInterp, filterParams, terrainElev, features = loadTLog(string(readPath,fileList[i]), VERBOSE=VERBOSE)
				# The following isn't necessary because the tlogDatINterp variable will never be written out.  So we can just pass terrainElev directly to extract features
				# if !isempty(terrainElev)
				# 	if length(terrainElev)>1
				# 		tlogDatInterp[:,symbol("terrainElev")] = terrainElev
				# 	else
				# 		tlogDatInterp[:,symbol("terrainElev")] = terrainElev*ones(Float64, size(tlogDatInterp,1))
				# 	end
				# end
				# display("$(size(tlogDatInterp)) is the size of tlogDatInterp")
				features = extractFeatures(tlogDatInterp, filterParams=filterParams, terrainElev=terrainElev)
				features[:,symbol("FlightNum")] = ind*ones(Int64, size(features,1))
				ind += 1
				namestlog = names(features)

				# Check to make sure we're not going to overwrite a CSV file that may have been started before (i.e. it already exists):
				if firstJLD && isfile(string(writeFile,outputType))
					firstJLD=false
					firstNames=namestlog
				end

				if !in(true,isnan(features[:,:altAGL]))
					# There are no NaN altitude AGL values in the feature, so we were  successful
					successAltitudes += 1
				end

				# Need to include a header only in the first call to write table.  Should also check to make sure the names are all the same:
				if firstJLD
					firstJLD = false
					firstNames = namestlog
					if isfile(string(writeFile,outputType))
						writetable(string(writeFile,outputType),features,append=true,header=false)
					else
						writetable(string(writeFile,outputType),features,append=false,header=true)
					end
				else
					# Check whether the names match the first names (will continue, however):
					if any(namestlog.!=firstNames)
						display("Warning, the names in file $(fileList[i]) don't match the names in the original output file:")
						display("Names in original file: $firstNames")
						display("Names in $(fileList[i]): $namestlog")
					end
					writetable(string(writeFile,outputType),features,append=true,header=false)
				end

				if saveFeaturesToJLD
					JLD.save(string(readPath,fileList[i]), "tlogDatInterp", tlogDatInterp, "filterParams", filterParams, "terrainElev", terrainElev, "features", features)
				end

			end
		end
	end

	if VERBOSE
		display("Found $successAltitudes elevations out of $n track queries.")
	end

	return successAltitudes
end

function getTerrainElevation(lat::Float64, lon::Float64; elevMethod::AbstractString="centroid")

	return getTerrainElevation([lat], [lon], elevMethod="first")[1]   # Since there's only one point, just use that rather than whatever the user entered.

end

function getTerrainElevation(lat::Array{Float64}, lon::Array{Float64}; elevMethod::AbstractString="centroid")

	return convert(Array, getTerrainElevation(DataArray(lat), DataArray(lon), elevMethod=elevMethod))

end

function getTerrainElevation(latArr::DataArray{Float64}, lonArr::DataArray{Float64}; elevMethod::AbstractString="centroid")

	lat=0.
	lon=0.
	terrainElev = Float64[]

	if (elevMethod=="first") | (elevMethod=="centroid")
		terrainElev = Array(Float64,1)
	end

	if elevMethod=="all"
		terrainElev = Array(Float64,length(lonArr))
	end

	for i=1:length(terrainElev)
		if elevMethod=="first"
			lat = latArr[1]
			lon = lonArr[1]
		end

		if elevMethod=="centroid"
			lat = mean(latArr)
			lon = mean(lonArr)
		end

		if elevMethod=="all"
			lat = latArr[i]
			lon = lonArr[i]
		end

		query=Dict("x" => string(lon), "y" => string(lat), "units" => "Feet", "output" => "xml")
		urlResp = get("http://ned.usgs.gov/epqs/pqs.php", query=query)

		dataPacket = readall(urlResp)
		startPoint = search(dataPacket, "<Elevation>")[1]+length("<Elevation>")
		endPoint = search(dataPacket, "</Elevation>")[1]-1
		terrainElev[i] = parse(Float64, dataPacket[startPoint:endPoint])
		if terrainElev[i]==-1.0e6
			terrainElev[i]= NaN
		end 
	end

	return DataArray(terrainElev)

end

# function featureFolderMSL2AGL(;readPath::AbstractString="./", writeFile::AbstractString="./featureHistory", outputType::AbstractString=".csv", 
# 							   VERBOSE::Bool=false, elevMethod::AbstractString="centroid")
# 	# Will convert the altitudes in the jld files to AGL and write out a CSV file with all the data. Same as featureFolder2csv, but includes the MSL->AGL conversion

# 	tlogDatArray = DataFrame[]
# 	fParamArray = FilterParams[]
# 	fileList = readdir(readPath)
# 	firstJLD = true
# 	firstNames = Symbol[]
# 	ind = 1
# 	n = length(fileList)

# 	for i=1:n
# 		if length(fileList[i])>5
# 	 		if (fileList[i][end-2:end] == "jld")
# 	 			if VERBOSE
# 		 			display("Loading jld file $i/$n: $(fileList[i]). Writing to $(string(writeFile,outputType))")
# 				end
# 				tlogDatInterp, filterParams = loadTLog(string(readPath,fileList[i]), VERBOSE=VERBOSE)

				

# 				features = extractFeatures(tlogDatInterp, filterParams=filterParams)
# 				features[:,symbol("FlightNum")] = ind*ones(Int64, size(features,1))
# 				ind += 1
# 				namestlog = names(features)

# 				# Need to include a header only in the first call to write table.  Should also check to make sure the names are all the same:
# 				if firstJLD
# 					firstJLD = false
# 					firstNames = namestlog
# 					writetable(string(writeFile,outputType),features,append=false,header=true)
# 				else
# 					# Check whether the names match the first names (will continue, however):
# 					if any(namestlog.!=firstNames)
# 						display("Warning, the names in file $(fileList[i]) don't match the names in the original output file:")
# 						display("Names in original file: $firstNames")
# 						display("Names in $(fileList[i]): $namestlog")
# 					end
# 					writetable(string(writeFile,outputType),features,append=true,header=false)
# 				end

# 			end
# 		end
# 	end

# 	return 0
# end

# function binFeatures(dataFile::AbstractString)
# 	tf = readtable(dataFile, header=true)
# 	outbins = binFeatures(tf)

# 	# Do something with the bins?

# end

function getAutoBinEdges(datArray::DataArray; numBins::Int64=6)
# Returns a set of bin edges based on the bin count and the same number of entries being in each bin.

	datSort = sort(datArray)
	datLen = length(datSort)
	indArr = Int64[]
	stepPrct = 1/numBins
	binEdges = Array(Float64,numBins+1)

	binEdges[1] = datSort[1]
	for i=1:numBins
		prctVal = i*stepPrct
		prctInd = floor(Int,prctVal*datLen)
		binEdges[i+1] = datSort[prctInd]
	end

	return binEdges

end	

# Here are our features, with bin edges selected to make roughly equal numbers of data points in each bin (so they are not uniformly spaced):
#  :range (ft) [0.0, 6.62105, 20.2713, 57.4401, 191.642, 677.942, 1981.8]
#  :vel   (kts) [0.,0.0317517, 0.0979747, 0.339625, 2.59219, 17.3287, 29.0623, 46.0387]
#  :bearing  (deg) [-180.0, -127.544, -83.4834, -45.5739, -1.00285, 45.3191, 92.3315, 140.121]
#  :heading  (deg) [-180.0, -124.378,-78.9133, -25.1267, 14.5364, 66.4746, 93.8072, 128.981]
#  :rrHeading (deg) [-360, -177.5, -96.4994, -41.989, 8.41766, 60.1614, 116.999, 187.001]    # before rectification
#  :rrHeading (deg) [0.0, 46.3583, 93.0749, 132.754, 176.582, 225.874, 265.326, 311.901]     # after rectification
#  :vdot     (kts/s) [-60.2262, -0.188663, -0.0437868, -0.0103595, 0.0, 0.012615, 0.0494847, 0.223948, 28.2656]
#  :rrHeaddot (deg/s) [-2354.47, -15.3275, -3.50304, -0.325726, 0.000748911, 0.198675, 2.4845, 13.8329]
#  :alt      (ft) [-4190.56, 12.2128, 89.7661, 211.148, 424.309, 687.007, 969.719, 1915.82]
#  :altdot  (ft/s) [-87.6822, -0.994523, -0.296876, -0.0844747, 1.7053e-12, 0.0793538, 0.305454, 1.01627]
#  :dxdt     (kts) [0., 0.0259615, 0.0674304, 0.152337, 0.405003, 2.6992, 19.0364, 31.2167] 
#  :pathDist (ft)  [0., 184.071, 1131.49, 3356.55, 7708.31, 12944.6, 37394.2, 89585.0]
#  :FlightNum (Int)

# Now creating bins to minimize the number but also capture typical performance:
#  :rangeBin = [0.0, 10., 20., 60., 200., 500., 2000., 5000., Inf]
#  :velBin = [0., 0.1, 1., 2.5, 10., 25., 50., Inf]
#  :bearingBin = [-180.0, -90., -45., 0., 45., 90., 180.]
#  :headingBin = [-180.0, -90., -45., 0., 45., 90., 180.]
#  :rrHeadingBin = [0., 45, 90., 135., 180., 225., 270., 315., 360.]
#  :vdotBin = [-60, -0.2, -0.05, -0.01, 0.01, 0.05, 0.2, 30.]
#  :rrHeaddotBin = [-2400., -15., -3.5, -0.35, 0., 0.35, 3.5, 15.]
#  :altBin  = [-4200, -10., 15., 100., 200, 400, 700., 1000., 2000., Inf]
#  :altdotBin = [-90, -1., -0.3, -0.05, 0.05, 0.30, 1., Inf]
#  :dxdtBin = [0., 0.05, 0.25, 0.75, 5., 20., 30., Inf] 
#  :pathDistBin =  [0., 200., 1000., 3500., 7500., 13000., 35000., 90000., Inf]
#  :FlightNum (Int)

function getHardCodedEdges(featSym::Symbol)
	# I think our main features (to start) should be range, dxdt, rrHeading, vdot, rrHeadDot, alt, altdot
	# With the following discretization, that come sout to 1,185,408 possible state combinations.  We have 10,422,608 data points.

	bins = Float64[]

	if featSym==:range
		bins = [0.0, 10., 20., 60., 200., 500., 2000., 5000., Inf]
	end
	if featSym==:vel
		bins = [0., 0.1, 1., 2.5, 10., 25., 50., Inf]
	end
	if featSym==:bearing
		bins = [-180.0, -90., -45., 0., 45., 90., 180.]
	end
	if featSym==:heading
	 	bins = [-180.0, -90., -45., 0., 45., 90., 180.]
	 end
 	if featSym==:rrHeading
	 	bins = [0., 45, 90., 135., 180., 225., 270., 315., 360.]
	 end
 	if featSym==:vdot
	 	bins = [-60, -0.2, -0.05, -0.01, 0.01, 0.05, 0.2, 30.]
	 end
 	if featSym==:rrHeaddot
	 	bins = [-2400., -15., -3.5, -0.20, 0.20, 3.5, 15.]
	 end
 	if featSym==:alt
	 	bins  = [-4200, -10., 15., 100., 200, 400, 700., 1000., 2000., Inf]
	 end
 	if featSym==:altdot
	 	bins = [-90, -1., -0.3, -0.05, 0.05, 0.30, 1., Inf]
	 end
 	if featSym==:dxdt
	 	bins = [0., 0.05, 0.25, 0.75, 5., 20., 30., Inf] 
	 end
 	if featSym==:pathDist
	 	bins =  [0., 200., 1000., 3500., 7500., 13000., 35000., 90000., Inf]
	end

	if isempty(bins)
		display("Warning, no bin sizes returned")
	end

	return bins
end

function convertFeature2Bin!(feat::DataFrame, indMat::DataFrame; binSymArray::Array{Symbol}=[:range, :dxdt, :rrHeading, :vdot, :rrHeaddot, :alt, :altdot])
	# Receives a single array of values and returns the indicies of the bins it would be in.  Currently hard coded for
	# the bins, with the following features:
	# range, dxdt, rrHeading, vdot, rrHeadDot, alt, altdot
	
	#indMat = DataFrame(Int8, length(binSymArray))

	for symind = 1:length(binSymArray)
		bins=getHardCodedEdges(binSymArray[symind])
		j=2
		while (convert(Array{Float64},feat[binSymArray[symind]])[1]>bins[j]) & (j<length(bins))
			j+=1
		end
		indMat[binSymArray[symind]] = j-1
	end

	return 0
end

function convertFeatures2BinMat(datArray::DataFrame; binSymArray::Array{Symbol}=[:range, :dxdt, :rrHeading, :vdot, :rrHeaddot, :alt, :altdot])

	numDatPoints = size(datArray,1)
	numFeatures = length(binSymArray)
	indMat = DataFrame()
	binMat = DataFrame()
	for i=1:length(binSymArray)
		binMat[binSymArray[i]]=Array(Int8,numDatPoints)
		indMat[binSymArray[i]]=Array(Int8,1)
	end

	for i=1:numDatPoints
		convertFeature2Bin!(datArray[i,:], indMat, binSymArray=binSymArray)
		binMat[i,:] = indMat
	end

	return binMat

end


function binFeatures(datArray::DataFrame; datNames::Array{Symbol}=names(datArray), bins::Tuple=(), counts::Array{Float64}=Float64[], defBinCount::Int64=20)
	# This version of the function would take a data frame of processed data.  This could be read directly from a file (a giant file, perhaps)
	# datNames is an array of symbols indicating which variables to plot.  If not specified, all columns of datArray will be plotted.
	# bins is an array of bin edge arrays.  The default value will be used if not specified.
	# counts is an array of the number of bins to use for each symbol.  If both bins and counts are specified, then bins will be used.
	# In the case of both bins and counts, the array location should correspond to the symbols specified in datNames
	# binMat,countMat=binFeatures(tf);

	# The range relative heading, :rrHeading, for some reason has a domain=[-360:360]. So you need to run the following command to rectify it:
	# rrHead=mod(tf[:,:rrHeading]+360,360);

	#datNames = names(datArray)
	binMat = ()
	countMat = Array(Array{Float64},length(datNames))

	for i=1:length(datNames)-1   		# Since the last name is FlightNum, which isn't a data point
		medVal = median(datArray[:,datNames[i]])
		minVal = minimum(datArray[:,datNames[i]])
		maxVal = maximum(datArray[:,datNames[i]])

		figure()
		title(string(datNames[i]))
		if !isempty(bins)	
			b,c = hist(datArray[:,datNames[i]], bins[i])
		elseif !isempty(counts)
			b,c = hist(datArray[:,datNames[i]], counts[i])
		else
			#binsAuto = minVal:(maxVal-minVal)/(defBinCount+1):maxVal
			#binsAuto = getAutoBinEdges(datArray[:,datNames[i]], numBins=defBinCount)
			binsAuto = getHardCodedEdges(datNames[i])
			b,c = hist(datArray[:,datNames[i]], binsAuto)
		end
		bar(b[1:end-1],c)

		binMat = (binMat..., b)
		countMat[i] = c

	end

	return binMat, countMat

end

# function binFeatures()
# # Eventually will need to bin the features (a la histograms) to predict probabilities of transitions, but not sure how this will work.
# # This could read in .jld files and build the bins one-by-one.

# 	tlogDatArray = DataFrame[]
# 	fParamArray = FilterParams[]
# 	fileList = readdir(readPath)
# 	firstJLD = true
# 	firstNames = Symbol[]
# 	ind = 1
# 	n = length(fileList)

# 	for i=1:n
# 		if length(fileList[i])>5
# 	 		if (fileList[i][end-2:end] == "jld")
# 	 			if VERBOSE
# 		 			display("Loading jld file $i/$n: $(fileList[i]). Writing to $(string(writeFile,outputType))")
# 				end
# 				tlogDatInterp, filterParams = loadTLog(string(readPath,fileList[i]), VERBOSE=VERBOSE)
# 				features = extractFeatures(tlogDatInterp, filterParams=filterParams)
# 				features[:,symbol("FlightNum")] = ind*ones(Int64, size(features,1))
# 				ind += 1
# 				namestlog = names(features)

# 				# Need to include a header only in the first call to write table.  Should also check to make sure the names are all the same:
# 				if firstJLD
# 					firstJLD = false
# 					firstNames = namestlog
# 					writetable(string(writeFile,outputType),features,append=false,header=true)
# 				else
# 					# Check whether the names match the first names (will continue, however):
# 					if any(namestlog.!=firstNames)
# 						display("Warning, the names in file $(fileList[i]) don't match the names in the original output file:")
# 						display("Names in original file: $firstNames")
# 						display("Names in $(fileList[i]): $namestlog")
# 					end
# 					writetable(string(writeFile,outputType),features,append=true,header=false)
# 				end

# 			end
# 		end
# 	end

# 	return 0
# end

function getDeriv(vals::Array{Float64}, t::Array{Float64}; n::Int64=1, k::Int64=1)
# Returns the kth derivative of vals using n points left and right of the current point
# Currently just supporting n=1 and k=1.  Would be nice to generalize this to arbitrary derivatives and points.
	
	deriv = Array(Float64,length(vals))

	if (k==1) & (n==1)
		deriv[1] = (vals[2]-vals[1])/(t[2]-t[1])    # Forward difference
		for i=2:length(vals)-1
			deriv[i] = (vals[i+1] - vals[i-1])/(t[i+1]-t[i-1])   # Central difference
		end
		deriv[end] = (vals[end]-vals[end-1])/(t[end]-t[end-1])   # backward difference
	else
		display("Derivative calculation not supported, returning nothing.")
		deriv = []
	end

	return deriv

end

function loadTLog(readFile::AbstractString; VERBOSE::Bool=false)
# This function loads a single tlog jld file specified in the input parameter and returns it as a data frame
# tlogDatInterp, filterParams = loadTLog("templog.jld");

	tlogDatInterp = DataFrame()
	filterParams = FilterParams()
	terrainElev = 0.
	features = DataFrame()

	if length(readFile)>5
 		if (readFile[end-2:end] == "jld")
 			try
	 			tlogDict = load(readFile)

	 			if length(tlogDict)==2
	 				# This is an older version with only tlogDatInterp and filterParams:
		 			tlogDatInterp = tlogDict["tlogDatInterp"]
		 			filterParams = tlogDict["filterParams"]
		 			if VERBOSE
		 				display("Received only tlog data and filter parameters, no terrain elevation data, or features")
		 			end
		 		elseif length(tlogDict)==3
		 			tlogDatInterp = tlogDict["tlogDatInterp"]
		 			filterParams = tlogDict["filterParams"]
		 			terrainElev = tlogDict["terrainElev"]
		 			if VERBOSE
		 				display("Received tlog data, filter parameters and terrain elevation data, no features")
		 			end
	 			elseif length(tlogDict)==4
		 			tlogDatInterp = tlogDict["tlogDatInterp"]
		 			filterParams = tlogDict["filterParams"]
		 			terrainElev = tlogDict["terrainElev"]
		 			features = tlogDict["features"]
		 			if VERBOSE
		 				display("Received tlog data, filter parameters, terrain elevation data and features")
		 			end
		 		end
	 		catch
	 			if VERBOSE
	 				display("Error reading file $readFile.")
	 			end
	 			# do nothing if error
	 		end
	 	else
	 		if VERBOSE
	 			display("Error reading file $readFile, not a jld file.")
	 		end
	 	end
	 end

	 return tlogDatInterp, filterParams, terrainElev, features

end

function loadTLogs(;readpath::AbstractString="./", VERBOSE::Bool=false)
# This function loads all tlog jld files from the specified read path and returns them  in a tlog data array.
# tlogDatArray, fParamArray = loadTLog(readpath="./")

	tlogDatArray = DataFrame[]
	fParamArray = FilterParams[]
	fileList = readdir(readpath)

	for i=1:length(fileList)
		if length(fileList[i])>5
	 		if (fileList[i][end-2:end] == "jld")
	 			if VERBOSE
		 			display("Found jld file: $(fileList[i])")
				end
	 			tlogDatInterp, FilterParams = loadTLog(string(readpath,fileList[i]), VERBOSE=VERBOSE)
	 			tlogDatArray = [tlogDatArray; tlogDatInterp]
	 			fParamArray = [fParamArray; filterParams]
	 		end
	 	end
	 end

	 return tlogDatArray, fParamArray

end

function processTLog(;readpath::AbstractString="./", writepath::AbstractString="./", createTextLogs::Bool=false, 
					  jldSaveNameBase::AbstractString="tlogData", fieldMatchFile::AbstractString="FieldMatchFile.txt", 
					  VERBOSE::Bool=false, timeKey::AbstractString="time_unix_usec_._mavlink_system_time_t",
					  maxTlogsinArray::Int64=20, tlogParseExe::AbstractString="./TLogReader.exe", 
					  filterParams::FilterParams=FilterParams(), successPath::AbstractString="", failPath::AbstractString="")
# This function can either convert tlog files into text files, or convert the data into
# Julia data structures and leave off the large text files.  Set createTextLogs to true
# to retain the text files. If false, they will be deleted.  If not empty, successPath will
# be the locations the .tlog files are moved to when they have been processed into .jld files.
# If empty then the tlog files will not be moved.  Similarly with failPath, if not empty the
# files that failed to convert will be moved to this directory.
# tlogDatArray, successParses, failParses = processTLog();
# tlogDatArray, successParses, failParses = processTLog(readpath="./", writepath="./", createTextLogs=false, jldSaveNameBase="tlogData", fieldMatchFile="FieldMatchFile.txt", VERBOSE=false, timeKey="time_unix_usec_._mavlink_system_time_t",successPath="", failPath="");

	outFileName = string(writepath,"templog.txt")
	tlogDatArray = DataFrame[]
	udArray = DataArray[]
	fileList = readdir(readpath)
	unitDict = getUnitConversions()
	inputFileName = AbstractString{}
	successParses = 0
	failParses = 0

	for i=1:length(fileList)
		if length(fileList[i])>5
	 		if fileList[i][end-3:end] == "tlog"
	 			inputFileName = fileList[i][1:end-5]
	 			parseSuccess = false
	 			if VERBOSE
			 		display("Found tlog file $i/$(length(fileList)): $(fileList[i])")
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
						tlogDat, ud = convertList2Array(tempFrame, inputFileName, filterParams=filterParams)

						if isempty(tlogDat)
							display("Did not find a time key in $(fileList[i]).  Skipping this file.")

						else
							# Remove the spurrious data points (zero or NaN/missing values, outliers)
							if VERBOSE
								display("Number of track hits before filtering: $(size(tlogDat,1))")
							end

							if  (in(symbol(filterParams.timeStr), names(tlogDat))) & (size(tlogDat,1)!=0)  		# This shouldn't be necessary, but once the program quit in filterRawData because it didn't have a timeKey...
								tlogDat=filterRawData(tlogDat, filterParams=filterParams, VERBOSE=VERBOSE)
								if VERBOSE
									display("Number of track hits after filtering: $(size(tlogDat,1))")
								end

								#display("$(size(tlogDat))")
								if  size(tlogDat,1)!=0  	# It's possible that after removing NaNs and zero time stamps we have no data left (that happened with 136076.tlog)
									# Convert units to aviation units:
									tlogDat = convertUnits(tlogDat, unitDict)

									# Remove any outliers in time:
									tlogDat = removeTimeOutliers(tlogDat, filterParams=filterParams, VERBOSE=VERBOSE)
								
									# Remove velocity outliers based on position changes (not the actual velocities, 
									# but perhaps I should check wether those are inconsistent?)  Also removes altitude
									# outliers based on similar criteria and identical methodology.
									tlogDat = removeVelOutliers(tlogDat, filterParams=filterParams, VERBOSE=VERBOSE)

									# Smooth data using a gaussian kernel:
									tlogDat = smoothHistories(tlogDat, filterParams=filterParams)

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

										# This is being stored as a separate variable because it may be NaN for a significant proportion 
									    # of the flights (60% are outside U.S.), and I don't want that to create a data frame in tlogDatInterp
									    # that has a lot of NaNs.
										terrainElev = getTerrainElevation(tlogDatInterp[:,symbol(filterParams.latStr)], tlogDatInterp[:,symbol(filterParams.lonStr)], elevMethod="centroid")

										# The jld files are quite large, four tlog files totaled 2.4 MB in .jld, so I'll save them individually.
										jldSaveName = string(jldSaveNameBase, inputFileName, ".jld")
										@save string(writepath,jldSaveName) tlogDatInterp filterParams terrainElev
										parseSuccess = true
										if !isempty(successPath)
											if VERBOSE
												display("Successfully processed file $(fileList[i]). Moving file to $(string(successPath, fileList[i])) and saving in $(string(writepath,jldSaveName)).")
											end
											mv(string(readpath, fileList[i]), string(successPath, fileList[i]), remove_destination=true)
										end
									end
								end
							end
						end
					end
				end

				if parseSuccess
					successParses += 1
				else
					failParses += 1
				end

				if (!parseSuccess) & (!isempty(failPath))
					if VERBOSE
						display("Failed to process file $(fileList[i]). Moving file to $(string(failPath, fileList[i])).")
					end
					mv(string(readpath, fileList[i]), string(failPath, fileList[i]), remove_destination=true)
				end
				toc()
		 	end
		end
	end

	# When we're done processing tlog files, if we're not to create text logs then delete the templog file:
	if (!createTextLogs) && (isfile(outFileName))
		run(`rm $outFileName`)
	end


	return tlogDatArray, successParses, failParses

end

function convertList2Array(datList::DataFrame, acStr::AbstractString; filterParams::FilterParams=FilterParams())


 	timeKey = filterParams.timeStr
	datArray = DataFrame()
	timeKeyFound = false
	ud=unique(datList[:Key])
	ud = [ud; "acNum"]
	numTimeElems = sum(datList[:,:Key].==timeKey)
 	for i=1:length(ud) 
 		datArray[symbol(ud[i])]=DataArray(Float64,numTimeElems)
 	end
	datArray[symbol("acNum")]=DataArray(repmat([acStr], numTimeElems))

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
		datArray[:,sym] = getConversion(datArray[:,sym], get(unitDict, sym, 1))  	# If we don't have the conversion, leave it unchanged
	end

	return datArray

end

function getConversion(datArray::DataArray{Float64,1},  convFactor::Int64)
	return datArray.*convFactor
end

function getConversion(datArray::DataArray{Float64,1},  convFactor::Float64)
	return datArray.*convFactor
end

function getConversion(datArray::DataArray{ASCIIString,1},  convFactor::Int64)
	# If the input is an array of strings (i.e. the ac string identifier), then don't multiply it, just return the array.
	return datArray
end

function filterRawData(datArray::DataFrame; filterParams::FilterParams=FilterParams(), VERBOSE::Bool=false)
# This function does all the processing and filtering on the raw data to get it into a usable format:
#	1. Remove the time stamps that are zero (there may be multiple, which make the data unusable)
#	2. Remove position locations that greatly increase the path length (these are probably spurrious) based on the velocity that track hit would imply
#	3. Remove altitude hits that have very lange altitude rates (these are probably spurrious)
# 	4. Remove time stamps that are excessively large by looking at their z-scores (maximum z-score for a sequential list starting at 0 should be about 1.79)
#	5. Remove combinations of lat/long that are (0,0).  These are probably dropouts.

	# Remove the NaNs from the key parameters, the zeros from the timeKey, and (0,0) entries from lat/longs
	timeKey=filterParams.timeStr
	goodInds = collect(1:size(datArray,1))
	zeroInds = find(datArray[:,symbol(timeKey)].==0)
	zeroLLInds = find((datArray[:,symbol(filterParams.params2Smooth[filterParams.latParam])].==0) & (datArray[:,symbol(filterParams.params2Smooth[filterParams.lonParam])].==0))
	naInds=Int64[]
	for prm in filterParams.params2Smooth
		if sum(symbol(prm).==names(datArray))>0    # Check whether this parameter is actually in the data set
			naInds = [naInds; find(isna(datArray[:,symbol(prm)]))]
		end
	end

	deleteat!(goodInds,sort(unique([naInds ; zeroInds; zeroLLInds])))
	datArray = datArray[goodInds, :]

	# skipping this check for now: check for positions that greatly increase path distance (i.e. remove outliers):

	# Remove missing altitudes, excessive vertical rates

	# Remove altitudes or positions that come before positions or altitudes?  This may already be taken care of

	return datArray
end

function removeTimeOutliers(datArray::DataFrame; filterParams::FilterParams=FilterParams(), VERBOSE::Bool=false)
	# Check for outliers in the time stamps.  Need to do this after removing nans so we don't get nan as the answer here:
	ts = datArray[:,symbol(filterParams.timeStr)].-datArray[1,symbol(filterParams.timeStr)]

	# We sometimes get wild discrepancies in the time stamps, like a time thats 1e9 seconds from the starting time.  So I'll remove those 
	# time stamps that are wildly different from the first, like 1e6 away (no flight is going to be 11+ days long on drone share...).  
	# This is necessary because the standard deviation check could be fooled if we have these data points (because the standard deviation 
	# will be so large)
	goodInds = collect(1:size(datArray,1))
	outlierInds = find(abs(ts).>1e6)
	deleteat!(goodInds,outlierInds)
	datArray = datArray[goodInds, :] 
	ts = datArray[:,symbol(filterParams.timeStr)].-datArray[1,symbol(filterParams.timeStr)]

	# Now do the check for single outliers:
	zts, maxInd = findmax((ts.-mean(ts))/std(ts))
	while zts > 5.0
		# In a roughly sequential list of numbers starting at 0, the largest element will have a z-value of about 1.79
		# Take out any elements with a greater z-value than this.  Don't know whether I should check for negative values too.
		if VERBOSE
			display("Removing a time stamp that was excessively large: $(ts[maxInd]) has a z-score of $zts.")
		end
		goodInds=collect(1:size(datArray,1))
		deleteat!(goodInds,maxInd)
		datArray = datArray[goodInds, :]
		ts = datArray[:,symbol(filterParams.timeStr)].-datArray[1,symbol(filterParams.timeStr)]
		zts, maxInd = findmax((ts.-mean(ts))/std(ts))
	end

	return datArray
end

function removeVelOutliers(datArray::DataFrame; filterParams::FilterParams=FilterParams(), VERBOSE::Bool=false)
	# May not need to do this, don't see very many points that have giant velocities for one step, perhaps
	# because we're directly recording velocity and not estimating it from positions and times. 
	# Uses the following thresholds in filterParams:
	# maxDrDtThreshold = 200. * 6071/3600 	# Maximum horizontal velocity between position reports (given dt) that is allowable.
	# maxDrDtThreshold = 3000. * 1/60 	# Maximum horizontal velocity between position reports (given dt) that is allowable.
	timeHist = convert(Array, datArray[:,symbol(filterParams.timeStr)])
	lat = convert(Array, datArray[:,symbol(filterParams.latStr)])
	lon = convert(Array, datArray[:,symbol(filterParams.lonStr)])

	# Remove the position outliers, based on observed distance between track hits and time stamps
	x,y = convertLL2XY(lat,lon)
	dx = diff(x)
	dy = diff(y)
	dR = sqrt(dx.^2+dy.^2)
	dt = diff(timeHist)
	dRdT = dR./dt     	# This is an estimate of the velocity at each time step.

	# highInds will be an array of points on either side of the high velocity segments.
	highInds = find(dRdT.>filterParams.maxDrDtThreshold)   # Get indicies of those steps that have greater velocity than indicated threshold
	highInds = unique([highInds; highInds+1])     # Now also include the "next" data point becaues it also contributes to the high velocity.  Remove duplicates

	while !isempty(highInds)
		speedRedux = Array(Float64,length(highInds))
		for i=1:length(highInds)
			speedRedux[i] = getSpeedReduction(x,y,timeHist, highInds[i], filterParams.maxDrDtThreshold)
		end
		maxRedux, maxInd = findmax(speedRedux)

		if VERBOSE
			display("Removing element $(highInds[maxInd]), resulting in a net speed reduction of $maxRedux ft/s.  Candidate elements = $(length(highInds))")
		end

		# Remove the worst offender
		goodInds = collect(1:size(datArray,1))
		deleteat!(goodInds,highInds[maxInd])
		datArray = datArray[goodInds, :]

		if VERBOSE
			display("New number of track hits is $(size(datArray,1))")
		end

		# Re-calculate the path distances (most will not have changed, can I do this more efficiently? )
		timeHist = convert(Array, datArray[:,symbol(filterParams.timeStr)])
		lat = convert(Array, datArray[:,symbol(filterParams.latStr)])
		lon = convert(Array, datArray[:,symbol(filterParams.lonStr)])
		x,y = convertLL2XY(lat,lon)
		dx = diff(x)
		dy = diff(y)
		dR = sqrt(dx.^2+dy.^2)
		dt = diff(timeHist)
		dRdT = dR./dt     	# This is an estimate of the velocity at each time step.

		# highInds will be an array of points on either side of the high velocity segments.
		highInds = find(dRdT.>filterParams.maxDrDtThreshold)   # Get indicies of those steps that have greater velocity than indicated threshold
		highInds = unique([highInds; highInds+1]) 
	end

	# Now remove the altitude outliers, based on observed altitudes between track hits and time stamps
	alt = convert(Array, datArray[:,symbol(filterParams.altStr)])
	timeHist = convert(Array, datArray[:,symbol(filterParams.timeStr)])

	dh = diff(alt)
	dt = diff(timeHist)
	dHdT = dh./dt     	# This is an estimate of the altitude rate at each time step.

	# highInds will be an array of points on either side of the high altitude rate segments.
	highInds = find(abs(dHdT).>filterParams.maxDhDtThreshold)   # Get indicies of those steps that have greater velocity than indicated threshold
	highInds = unique([highInds; highInds+1])     # Now also include the "next" data point becaues it also contributes to the high velocity.  Remove duplicates

	while !isempty(highInds)
		speedRedux = Array(Float64,length(highInds))
		for i=1:length(highInds)
			speedRedux[i] = getAltRateReduction(alt,timeHist, highInds[i], filterParams.maxDhDtThreshold)
		end
		maxRedux, maxInd = findmax(speedRedux)

		if VERBOSE
			display("Removing element $(highInds[maxInd]), resulting in a net altitude rate reduction of $maxRedux ft/s.  Candidate elements = $(length(highInds))")
		end

		# Remove the worst offender
		goodInds = collect(1:size(datArray,1))
		deleteat!(goodInds,highInds[maxInd])
		datArray = datArray[goodInds, :]

		if VERBOSE
			display("New number of track hits is $(size(datArray,1))")
		end

		# Re-calculate the altitudes distances (most will not have changed, can I do this more efficiently? )
		alt = convert(Array, datArray[:,symbol(filterParams.altStr)])
		timeHist = convert(Array, datArray[:,symbol(filterParams.timeStr)])

		dh = diff(alt)
		dt = diff(timeHist)
		dHdT = dh./dt     	# This is an estimate of the altitude rate at each time step.

		# highInds will be an array of points on either side of the high altitude rate segments.
		highInds = find(abs(dHdT).>filterParams.maxDhDtThreshold)   # Get indicies of those steps that have greater velocity than indicated threshold
		highInds = unique([highInds; highInds+1])     # Now also include the "next" data point becaues it also contributes to the high velocity.  Remove duplicates

	end


	return datArray

end

function getSpeedReduction(x::Array{Float64}, y::Array{Float64}, t::Array{Float64}, remInd::Int64, speedThreshold::Float64)
# Return the reduction in speed, over the threshold, that comes from removing the given index.  Can do this by looking only at the
# values at remInd and its immediate neighbors.

	speedRedux = 0.  
	if remInd==1
		# Removing the first point, easy calculation:
		speedRedux = max(0., sqrt((x[2]-x[1])^2+(y[2]-y[1])^2)/(t[2]-t[1]) - speedThreshold)
	elseif (remInd<length(x))
		speedRedux = max(0., sqrt((x[remInd]-x[remInd-1])^2+(y[remInd]-y[remInd-1])^2)/(t[remInd]-t[remInd-1]) +
		   			 		 sqrt((x[remInd]-x[remInd+1])^2+(y[remInd]-y[remInd+1])^2)/(t[remInd+1]-t[remInd]) -
					 		 sqrt((x[remInd-1]-x[remInd+1])^2+(y[remInd-1]-y[remInd+1])^2)/(t[remInd+1]-t[remInd-1]) - 
					 		 speedThreshold)
	else
		speedRedux = max(0., sqrt((x[remInd]-x[remInd-1])^2+(y[remInd]-y[remInd-1])^2)/(t[remInd]-t[remInd-1]) - speedThreshold)
	end

	return speedRedux
end	

function getAltRateReduction(h::Array{Float64}, t::Array{Float64}, remInd::Int64, speedThreshold::Float64)
# Return the reduction in speed, over the threshold, that comes from removing the given index.  Can do this by looking only at the
# values at remInd and its immediate neighbors.

	speedRedux = 0.  
	if remInd==1
		# Removing the first point, easy calculation:
		speedRedux = max(0., abs(h[2]-h[1])/(t[2]-t[1]) - speedThreshold)
	elseif (remInd<length(h))
		speedRedux = max(0., abs(h[remInd]-h[remInd-1])/(t[remInd]-t[remInd-1]) +
		   			 		 abs(h[remInd+1]-h[remInd])/(t[remInd+1]-t[remInd]) -
					 		 abs(h[remInd+1]-h[remInd-1])/(t[remInd+1]-t[remInd-1]) - 
					 		 speedThreshold)
	else
		speedRedux = max(0., abs(h[remInd]-h[remInd-1])/(t[remInd]-t[remInd-1]) - speedThreshold)
	end

	return speedRedux
end	

function smoothHistories(datArray::DataFrame; filterParams::FilterParams=FilterParams())
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

# Should I sort timeHist if it's not sorted, would that work ok?


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
		PyPlot.plot(x, y,"g")
		xlabel("X (ft)")
		ylabel("Y (ft)")

		# Plot alt vs. time		
		figure()
		PyPlot.plot(timeHist-timeHist[1], alt, "r")
		xlabel("Time (s)")
		ylabel("Altitude (ft)")

		# Plot velocity vs. time
		#timeHist = convert(Array, tlogDatArray[i][:,symbol(filterParams.timeStr)])
		
		figure()
		PyPlot.plot(timeHist-timeHist[1], vel, "b")
		xlabel("Time (s)")
		ylabel("Velocity (kts)")

		# Plot vertical velocity (dh/dt) vs. time
		#timeHist = convert(Array, tlogDatArray[i][:,symbol(filterParams.timeStr)])
		#alt = convert(Array, tlogDatArray[i][:,symbol(filterParams.altStr)])
		dh = diff(alt)
		dt = diff(timeHist)
		dhdt = dh./dt .* 60.  # Convert from ft/s to ft/min
		figure()
		PyPlot.plot(timeHist[2:end]-timeHist[1], dhdt, "g")
		xlabel("Time (s)")
		ylabel("Vertical Velocity, dh/dt (ft/min)")

		# Plot time steps:
		#timeHist = tlogDatArray[i][:,symbol(filterParams.timeStr)]
		#dt = diff(timeHist)
		figure()
		PyPlot.plot(timeHist[2:end]-timeHist[1], dt)
		xlabel("Time (s)")
		ylabel("Time between consecutive time stamps (s)")
		ylim([0, 2*maximum(dt)])
	end

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

function scrapeDroneShare(;minIndex::Int64=4734, maxIndex::Int64=168329, writePath::AbstractString="./", 
						   reportInterval::Int64=100, VERBOSE::Bool=false)
	# Max mission index on 12/30/15 at 12:06 pm was 168329.  Couldn't view any of the logs, though.
	# respArr = scrapeDroneShare(minIndex=168000, maxIndex=168329);

	respArr=Array(HttpCommon.Response,maxIndex-minIndex+1)

	startTime = convert(Float64,time_ns())/1e9

	ind = 1
	successes = 0
	for i=minIndex:maxIndex
		respArr[ind]=get("http://api.droneshare.com/api/v1/mission/$i/messages.tlog")
		
		#display("Seeking file $i.  Response was $(respArr[ind].status)")
		if respArr[ind].status!=404
			if VERBOSE
				display("*********************** Did not get a 404!!!!***********************")
			end

			if respArr[ind].status==200
				# Request succeeded
				try
					Requests.save(respArr[ind], string(writePath, i, ".tlog") )
					successes+=1
					if VERBOSE
						display("Saved file $(string(writePath, i, ".tlog"))")
					end

				catch
					display("Error, failed to save file $(string(writePath, i, ".tlog")) even though we received a 200 response from the server")
				end
			end

		end

		if reportInterval != 0
			if mod(ind,reportInterval)==0
				currTime = convert(Float64,time_ns())/1e9
				display("Sought $ind files in $(currTime-startTime) s.  Estimated remaining time for $(maxIndex-minIndex-ind) files is $(((maxIndex-minIndex-ind)/ind)*(currTime-startTime)).  Found $successes tlog files so far.")
			end
		end

		ind += 1
	end

	return respArr

end

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