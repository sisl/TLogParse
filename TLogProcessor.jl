using HDF5, JLD, DataFrames, DataArrays, PyPlot

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

# Note: don't put leading . on your paths.  That will confuse my .tlog file finder and cause it to skip that folder entirely.

function processTLog(;readpath::AbstractString="./", writepath::AbstractString="./", createTextLogs::Bool=false, 
					  jldSaveNameBase::AbstractString="tlogData", fieldMatchFile::AbstractString="FieldMatchFile.txt", 
					  VERBOSE::Bool=false, timeKey::AbstractString="time_unix_usec_._mavlink_system_time_t",
					  maxTlogsinArray::Int64=100, tlogParseExe::AbstractString="./TLogReader.exe")
# This function can either convert tlog files into text files, or convert the data into
# Julia data structures and leave off the large text files.  Set createTextLogs to true
# to retain the text files. If false, they will be deleted.
# tlogDatArray, udArray, timeKey = processTLog();
# tlogDatArray, udArray, timeKey = processTLog(readpath="./", writepath="./", createTextLogs=false, jldSaveNameBase="tlogData.jld", fieldMatchFile="FieldMatchFile.txt", VERBOSE=false, timeKey="time_unix_usec_._mavlink_system_time_t");

	outFileName = string(writepath,"templog.txt")
	tlogDatArray = DataFrame[]
	udArray = DataArray[]
	fileList = readdir(readpath)

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
					display("Error, no tlog output file created for $(outFileName)")
				else
					tempFrame = readtable(outFileName, separator=',', header=false, names=[:Key,:Val])
					# Put the data into table format, one row per time stamp (where the time variable is specified in timeKey):
					# ud is a data array of strings, each of which can be converted to a symbol with symbol(ud[i]) to index tlogDat parameters.
					tlogDat, ud = convertList2Array(tempFrame, i, timeKey=timeKey)

					# Fetch additional data, if possible:
					#   1. AGL data
					#   2. Airspace class data
					#   3. Country location
					if length(fileList)<=maxTlogsinArray
						# If there are more than maxTlogsinArray files, don't store everything in one giant data array.  Will
						# need to plot and analyze trajectories individually through their jld files, I think.
						push!(tlogDatArray,tlogDat)
						push!(udArray,ud)
					end

					# The jld files are quite large, four tlog files totaled 2.4 MB in .jld, so I'll save them individually.
					jldSaveName = string(jldSaveNameBase, i, ".jld")
					@save string(writepath,jldSaveName) tlogDatArray udArray timeKey
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

function convertList2Array(datList::DataFrame, acNum::Int64; timeKey::AbstractString="time_unix_usec_._mavlink_system_time_t")
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

	datArray = DataFrame()
	timeKeyFound = false
	ud=unique(datList[:Key])
	numTimeElems = sum(datList[:,:Key].==timeKey)
 	for i=1:length(ud) 
 		datArray[symbol(ud[i])]=DataArray(Float64,numTimeElems)
 		#datArray[:,symbol(ud[i])] = NA
 	end
	datArray[:acNum]=acNum*ones(Float64,numTimeElems)

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

function plotTLogData(tlogDatArray::Array{DataFrame}, timeKey)

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

		# Plot the raw lat/lon traces:
		goodInds = collect(1:size(tlogDatArray[i],1))
		latInds = find(isna(tlogDatArray[i][:,symbol(latrawStr)]))
		lonInds = find(isna(tlogDatArray[i][:,symbol(lonrawStr)]))
		naInds = unique([latInds,lonInds])
		#display("i=$i, $naInds")
		deleteat!(goodInds,naInds)

		lon = convert(Array, tlogDatArray[i][goodInds,symbol(lonrawStr)])./10^7
		lat = convert(Array, tlogDatArray[i][goodInds,symbol(latrawStr)])./10^7
		figure()
		# plot(lon-mean(lon), lat-mean(lat),"r")
		plot(lon, lat,"r")
		xlabel("Longitude (??)")
		ylabel("Latitude (??)")

		# Plot x and y:
		x,y = convertLL2XY(lat,lon)
		figure()
		# plot(lon-mean(lon), lat-mean(lat),"r")
		plot(x, y,"g")
		xlabel("X (m)")
		ylabel("Y (m)")

		# Now overplot the gps lat/lon traces
		goodInds = collect(1:size(tlogDatArray[i],1))
		latInds = find(isna(tlogDatArray[i][:,symbol(latglbStr)]))
		lonInds = find(isna(tlogDatArray[i][:,symbol(longlbStr)]))
		naInds = unique([latInds,lonInds])
		#display("i=$i, $naInds")
		deleteat!(goodInds,naInds)
		lon = convert(Array, tlogDatArray[i][goodInds,symbol(longlbStr)])
		lat = convert(Array, tlogDatArray[i][goodInds,symbol(latglbStr)])
		# plot(lon-mean(lon), lat-mean(lat),"b")
		plot(lon, lat,"b")

		# Plot alt vs. time
		goodInds = collect(1:size(tlogDatArray[i],1))
		altInds = find(isna(tlogDatArray[i][:,symbol(altrawStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([altInds, timeInds])
		deleteat!(goodInds,naInds)

		alt = convert(Array, tlogDatArray[i][goodInds,symbol(altrawStr)])./10^2
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		figure()
		plot(timeHist-timeHist[1], alt, "r")
		xlabel("Time (s)")
		ylabel("Altitude (m)")

		# Now overplot the gps altitude:
		goodInds = collect(1:size(tlogDatArray[i],1))
		altInds = find(isna(tlogDatArray[i][:,symbol(altglbStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([altInds, timeInds])
		deleteat!(goodInds,naInds)
		alt = convert(Array, tlogDatArray[i][goodInds,symbol(altglbStr)])./10^2
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		plot(timeHist-timeHist[1], alt, "b")

		# Plot velocity vs. time
		goodInds = collect(1:size(tlogDatArray[i],1))
		velInds = find(isna(tlogDatArray[i][:,symbol(velrawStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, velInds])
		deleteat!(goodInds,naInds)
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		vel = convert(Array, tlogDatArray[i][goodInds,symbol(velrawStr)])./10^2
		figure()
		plot(timeHist-timeHist[1], vel, "r")
		xlabel("Time (s)")
		ylabel("Velocity (m/s)")

		goodInds = collect(1:size(tlogDatArray[i],1))
		vxInds = find(isna(tlogDatArray[i][:,symbol(vxStr)]))
		vyInds = find(isna(tlogDatArray[i][:,symbol(vyStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, vxInds, vyInds])
		# display("$(length(timeInds)) $(length(vxInds)) $(length(vyInds))")
		# display("NA inds for vx, vy and time: $naInds")
		# display("NA inds before removal: $(find(isna(tlogDatArray[i][:,symbol(vxStr)])))")
		# display("NA inds before removal: $(find(isna(tlogDatArray[i][:,symbol(vyStr)])))")
		deleteat!(goodInds,naInds)
		# display("NA inds after removal: $(find(isna(tlogDatArray[i][goodInds,symbol(vxStr)])))")
		# display("NA inds after removal: $(find(isna(tlogDatArray[i][goodInds,symbol(vyStr)])))")
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		velGlobal = (tlogDatArray[i][goodInds,symbol(vxStr)].^2+tlogDatArray[i][goodInds,symbol(vyStr)].^2).^0.5
		vel = convert(Array, velGlobal)./10^2
		plot(timeHist-timeHist[1], vel, "b")

		# Plot vertical gps velocity vs. time (probably won't be good)
		goodInds = collect(1:size(tlogDatArray[i],1))
		vvelInds = find(isna(tlogDatArray[i][:,symbol(vzStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, vvelInds])
		deleteat!(goodInds,naInds)
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		vel = convert(Array, tlogDatArray[i][goodInds,symbol(vzStr)])./10^2
		figure()
		plot(timeHist-timeHist[1], vel, "r")
		xlabel("Time (s)")
		ylabel("Vertical Velocity (m/s)")

		# Plot airspeed vs. time 
		goodInds = collect(1:size(tlogDatArray[i],1))
		aspdInds = find(isna(tlogDatArray[i][:,symbol(aspdStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, aspdInds])
		deleteat!(goodInds,naInds)
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		vel = convert(Array, tlogDatArray[i][goodInds,symbol(aspdStr)])
		figure()
		plot(timeHist-timeHist[1], vel, "r")
		xlabel("Time (s)")
		ylabel("Airspeed (m/s)")

		# Plot Climb vs. time 
		goodInds = collect(1:size(tlogDatArray[i],1))
		climbInds = find(isna(tlogDatArray[i][:,symbol(climbStr)]))
		timeInds = find(isna(tlogDatArray[i][:,symbol(timeKey)]))
		naInds = unique([timeInds, climbInds])
		deleteat!(goodInds,naInds)
		timeHist = convert(Array, tlogDatArray[i][goodInds,symbol(timeKey)])./10^6
		vel = convert(Array, tlogDatArray[i][goodInds,symbol(climbStr)])
		figure()
		plot(timeHist-timeHist[1], vel, "r")
		xlabel("Time (s)")
		ylabel("Climb (?)")
	end


	latStr = "lat_._mavlink_global_position_int_t"
	lonStr = "lon_._mavlink_global_position_int_t"
	altStr = "alt_._mavlink_global_position_int_t"

	return 0

end	

function convertLL2XY(lat,lon)
# Uses the first point as the origin, then uses simple curvilinear approx to get x,y
	Re = 6371000
	initLat = lat[1]
	initLon = lon[1]

	lat2m = 2*pi*Re/360
	lon2m = 2*pi*Re*cosd(initLat)/360

	latShift = lat.-initLat
	lonShift = lon.-initLon

	x = lonShift.*lon2m
	y = latShift.*lat2m

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