# TLogParse
Parser for telemetry log (tlog) files from mission planner software.

TLogProcessor.jl is designed to process Mission Planner .tlog files into Julia data structures.  Here's how to run it:

julia\> tlogDatArray, udArray, timeKey = processTLog(readpath="/", writepath="/", createTextLogs=false, jldSaveNameBase="tlogData", fieldMatchFile="FieldMatchFile.txt", VERBOSE=false, timeKey="time_unix_usec_._mavlink_system_time_t");

readpath and writepath are simply the places you'd like to read tlogs from and where you'd like to store the output files (text versions of the tlog files and/or julia data structures of the data).  Note: don't put leading . on your paths.  That will confuse my .tlog file finder and cause it to skip that folder entirely. The boolean createTextLogs indicates whether to actually write the tlog parameters to a text file; if false, only the julia data structures will be saved. jldSaveNameBase indicates the .jld filename in which to store the julia data structures.  Note that currently if there are more than 100 files to parse it does not create the tlogDatArray data structure out of fear that it will be too large.  Instead, it saves the data structures as individual .jld files which can be read in sequentially and processed individually in julia. fieldMatchFile is a list of parameters to convert, if omitted it will convert all parameters (see the following paragraph).  timeKey indicates which tlog parameter to use as the timestamp when collecting sequential parameters into discrete time bins.

The outputs of TLogProcessor are the array of data tables (one entry per .tlog file, as long as there are fewer than 100 files, otherwise it will be empty), an array of the unique identifiers pulled out of each tlog file (this can be used to index into the data tables without worrying about figuring out the column numbers), and the timekey used to align all the data into bins.

The batch processor relys on calls to the C# programe TLogReader.exe, which converts the tlog file into a text file. The usage for that program is:

\>\> TLogReader.exe inputfile.tlog [outputfile.txt] [verbose: true/false] [FieldMatchFile.txt]

For the verbose flag, simply type "true" or "false", there are no - characters.  The output file is optional, if
not specified the output will be the same name as the .tlog file but with a .txt extension.  The last parameter
(they must be specified in this order) is a list of field names to convert into the text file (1 per line).  If omitted, every
field will be converted (resulting in a file about 10x as large as the .tlog).  To reduce the output size, which
is recommended becaues there are well over a hundred parameters in the .tlog files and most won't be useful, specify
the parameter names you'd like to keep..  See the following list of parameters, which are separated from their MAVLink-specified 
data types by the three characters "\_.\_".  The first entry indicates that the parameter name is "time_unix_usec" 
while its type is "mavlink_system_time_t".  The type was retained because there are several parameters, like "alt" 
that are written several times as different types.  I wanted to preserve the ability to differentiate between 
them.  Note that if you specify "alt" in the FieldMatchFile you'll get all three different flavors of that variable.

Note also that users specify which telemetry parameters to pass over MAVLink and at what rate, and that there is no
guarantee that every parameter will be received/stored at each time step.  That's bandwidth limited.  So the presence
of a parameter in one .tlog file is no guarantee that it will be in another.  See the Mission Planner documentation
for an explanation of these parameters (http://planner.ardupilot.com/wiki/mission-planner-overview/)

## Recommended TLog Parameters to Parse
vx

vy

vz

lat

lon

alt

eph

epv

vel

time_unix_usec

time_boot_ms

xacc

yacc

zacc

roll

pitch

yaw

rollspeed

pitchspeed

yawspeed

climb

airspeed


## All TLog Parameters
time_unix_usec_._mavlink_system_time_t

time_boot_ms_._mavlink_system_time_t

custom_mode_._mavlink_heartbeat_t

type_._mavlink_heartbeat_t

autopilot_._mavlink_heartbeat_t

base_mode_._mavlink_heartbeat_t

system_status_._mavlink_heartbeat_t

mavlink_version_._mavlink_heartbeat_t

req_message_rate_._mavlink_request_data_stream_t

target_system_._mavlink_request_data_stream_t

target_component_._mavlink_request_data_stream_t

req_stream_id_._mavlink_request_data_stream_t

start_stop_._mavlink_request_data_stream_t

target_system_._mavlink_param_request_list_t

target_component_._mavlink_param_request_list_t

rxerrors_._mavlink_radio_t

fixed_._mavlink_radio_t

rssi_._mavlink_radio_t

remrssi_._mavlink_radio_t

txbuf_._mavlink_radio_t

noise_._mavlink_radio_t

remnoise_._mavlink_radio_t

rxerrors_._mavlink_radio_status_t

fixed_._mavlink_radio_status_t

rssi_._mavlink_radio_status_t

remrssi_._mavlink_radio_status_t

txbuf_._mavlink_radio_status_t

noise_._mavlink_radio_status_t

remnoise_._mavlink_radio_status_t

time_usec_._mavlink_raw_imu_t

xacc_._mavlink_raw_imu_t

yacc_._mavlink_raw_imu_t

zacc_._mavlink_raw_imu_t

xgyro_._mavlink_raw_imu_t

ygyro_._mavlink_raw_imu_t

zgyro_._mavlink_raw_imu_t

xmag_._mavlink_raw_imu_t

ymag_._mavlink_raw_imu_t

zmag_._mavlink_raw_imu_t

time_boot_ms_._mavlink_scaled_pressure_t

press_abs_._mavlink_scaled_pressure_t

press_diff_._mavlink_scaled_pressure_t

temperature_._mavlink_scaled_pressure_t

time_boot_ms_._mavlink_attitude_t

roll_._mavlink_attitude_t

pitch_._mavlink_attitude_t

yaw_._mavlink_attitude_t

rollspeed_._mavlink_attitude_t

pitchspeed_._mavlink_attitude_t

yawspeed_._mavlink_attitude_t

airspeed_._mavlink_vfr_hud_t

groundspeed_._mavlink_vfr_hud_t

alt_._mavlink_vfr_hud_t

climb_._mavlink_vfr_hud_t

heading_._mavlink_vfr_hud_t

throttle_._mavlink_vfr_hud_t

omegaIx_._mavlink_ahrs_t

omegaIy_._mavlink_ahrs_t

omegaIz_._mavlink_ahrs_t

accel_weight_._mavlink_ahrs_t

renorm_val_._mavlink_ahrs_t

error_rp_._mavlink_ahrs_t

error_yaw_._mavlink_ahrs_t

Vcc_._mavlink_hwstatus_t

I2Cerr_._mavlink_hwstatus_t

onboard_control_sensors_present_._mavlink_sys_status_t

onboard_control_sensors_enabled_._mavlink_sys_status_t

onboard_control_sensors_health_._mavlink_sys_status_t

load_._mavlink_sys_status_t

voltage_battery_._mavlink_sys_status_t

current_battery_._mavlink_sys_status_t

drop_rate_comm_._mavlink_sys_status_t

errors_comm_._mavlink_sys_status_t

errors_count1_._mavlink_sys_status_t

errors_count2_._mavlink_sys_status_t

errors_count3_._mavlink_sys_status_t

errors_count4_._mavlink_sys_status_t

battery_remaining_._mavlink_sys_status_t

brkval_._mavlink_meminfo_t

freemem_._mavlink_meminfo_t

seq_._mavlink_mission_current_t

time_usec_._mavlink_gps_raw_int_t

lat_._mavlink_gps_raw_int_t

lon_._mavlink_gps_raw_int_t

alt_._mavlink_gps_raw_int_t

eph_._mavlink_gps_raw_int_t

epv_._mavlink_gps_raw_int_t

vel_._mavlink_gps_raw_int_t

cog_._mavlink_gps_raw_int_t

fix_type_._mavlink_gps_raw_int_t

satellites_visible_._mavlink_gps_raw_int_t

nav_roll_._mavlink_nav_controller_output_t

nav_pitch_._mavlink_nav_controller_output_t

alt_error_._mavlink_nav_controller_output_t

aspd_error_._mavlink_nav_controller_output_t

xtrack_error_._mavlink_nav_controller_output_t

nav_bearing_._mavlink_nav_controller_output_t

target_bearing_._mavlink_nav_controller_output_t

wp_dist_._mavlink_nav_controller_output_t

breach_time_._mavlink_fence_status_t

breach_count_._mavlink_fence_status_t

breach_status_._mavlink_fence_status_t

breach_type_._mavlink_fence_status_t

severity_._mavlink_statustext_t

param_value_._mavlink_param_value_t

param_count_._mavlink_param_value_t

param_index_._mavlink_param_value_t

param_type_._mavlink_param_value_t

time_boot_ms_._mavlink_global_position_int_t

lat_._mavlink_global_position_int_t

lon_._mavlink_global_position_int_t

alt_._mavlink_global_position_int_t

relative_alt_._mavlink_global_position_int_t

vx_._mavlink_global_position_int_t

vy_._mavlink_global_position_int_t

vz_._mavlink_global_position_int_t

hdg_._mavlink_global_position_int_t

time_usec_._mavlink_servo_output_raw_t

servo1_raw_._mavlink_servo_output_raw_t

servo2_raw_._mavlink_servo_output_raw_t

servo3_raw_._mavlink_servo_output_raw_t

servo4_raw_._mavlink_servo_output_raw_t

servo5_raw_._mavlink_servo_output_raw_t

servo6_raw_._mavlink_servo_output_raw_t

servo7_raw_._mavlink_servo_output_raw_t

servo8_raw_._mavlink_servo_output_raw_t

port_._mavlink_servo_output_raw_t

time_boot_ms_._mavlink_rc_channels_raw_t

chan1_raw_._mavlink_rc_channels_raw_t

chan2_raw_._mavlink_rc_channels_raw_t

chan3_raw_._mavlink_rc_channels_raw_t

chan4_raw_._mavlink_rc_channels_raw_t

chan5_raw_._mavlink_rc_channels_raw_t

chan6_raw_._mavlink_rc_channels_raw_t

chan7_raw_._mavlink_rc_channels_raw_t

chan8_raw_._mavlink_rc_channels_raw_t

port_._mavlink_rc_channels_raw_t

rssi_._mavlink_rc_channels_raw_t

param_index_._mavlink_param_request_read_t

target_system_._mavlink_param_request_read_t

target_component_._mavlink_param_request_read_t

mag_declination_._mavlink_sensor_offsets_t

raw_press_._mavlink_sensor_offsets_t

raw_temp_._mavlink_sensor_offsets_t

gyro_cal_x_._mavlink_sensor_offsets_t

gyro_cal_y_._mavlink_sensor_offsets_t

gyro_cal_z_._mavlink_sensor_offsets_t

accel_cal_x_._mavlink_sensor_offsets_t

accel_cal_y_._mavlink_sensor_offsets_t

accel_cal_z_._mavlink_sensor_offsets_t

mag_ofs_x_._mavlink_sensor_offsets_t

mag_ofs_y_._mavlink_sensor_offsets_t

mag_ofs_z_._mavlink_sensor_offsets_t

param1_._mavlink_command_long_t

param2_._mavlink_command_long_t

param3_._mavlink_command_long_t

param4_._mavlink_command_long_t

param5_._mavlink_command_long_t

param6_._mavlink_command_long_t

param7_._mavlink_command_long_t

command_._mavlink_command_long_t

target_system_._mavlink_command_long_t

target_component_._mavlink_command_long_t

confirmation_._mavlink_command_long_t
