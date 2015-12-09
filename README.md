# TLogParse
Parser for telemetry log (tlog) files from mission planner software.

TLogProcessor.jl is desigen to process Mission Planner .tlog files into Julia data structures.  It relys on calls
to the C# programe TLogReader.exe, which converts the tlog file into a text file.  The usage for that program is:
>> TLogReader.exe inputfile.tlog [outputfile.txt] [verbose: true/false] [FieldMatchFile.txt]

For the verbose flag, simply type "true" or "false", there are no - characters.  The output file is optional, if
not specified the output will be the same name as the .tlog file but with a .txt extension.  The last parameter
(they must be specified in this order) is a list of field names to convert into the text file (1 per line).  If omitted, every
field will be converted (resulting in a file about 10x as large as the .tlog).  To reduce the output size, which
is recommended becaues there are well over a hundred parameters in the .tlog file and most won't be useful, specify
the parameter name only.  See the following list of parameters, which are separated from their MAVLink-specified 
data types by the three characters "_._".  The first entry indicates that the parameter name is "time_unix_usec" 
while its type is "mavlink_system_time_t".  The type was retained because there are several parameters, like "alt" 
that are written several times as different types.  I wanted to preserve the ability to differentiate between 
them.  Note that if you specify "alt" in the FieldMatchFile you'll get all three different flavors of that variable.

Note also that users specify which telemetry parameters to pass over MAVLink and at what rate, and that there is no
guarantee that every parameter will be received/stored at each time step.  That's bandwidth limited.  So the presence
of a parameter in one .tlog file is no guarantee that it will be in another.  See the Mission Planner documentation
for an explanation of these parameters (http://planner.ardupilot.com/wiki/mission-planner-overview/)

Note: don't put leading . on your paths.  That will confuse my .tlog file finder and cause it to skip that folder entirely.
