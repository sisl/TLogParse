using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
//using System.Windows.Forms;
using MissionPlanner;
//using csmatio.io;
//using csmatio.types;
//using csmatio.dll;
using System.Globalization;
//using log4net;
using System.Reflection;
using System.Runtime.InteropServices;
using MissionPlanner.Mavlink;

namespace MissionPlanner.Log
{
    public class TLogUtils
    {
		public static string[] getFields (string fieldMatchFile)
		{
			string[] fields = null;
			try {
				fields = System.IO.File.ReadAllLines(@fieldMatchFile);
			} catch {
				Console.WriteLine ("Could not open field matching file, will match all fields.");
				fields = null;
			}
			return fields;
		}

        public static void tlog(string logfile, string outfile, Boolean VERBOSE, string[] fieldMatch)
        {
            Hashtable datappl = new Hashtable();
			int numEntries = 0;

            using (MAVLinkInterface mine = new MAVLinkInterface())
            {
                try
                {
					if (VERBOSE) {
						Console.WriteLine("Attempting to open logfile {0}", logfile);
					}
                    mine.logplaybackfile =
                        new BinaryReader(File.Open(logfile, FileMode.Open, FileAccess.Read, FileShare.Read));
                }
                catch
                {
					if (VERBOSE) {
						Console.Write ("Log Can not be opened. Are you still connected?");
					}
					return;
                }
                mine.logreadmode = true;

                mine.MAV.packets.Initialize(); // clear     

				using (System.IO.StreamWriter file = new System.IO.StreamWriter(@outfile)) {

					while (mine.logplaybackfile.BaseStream.Position < mine.logplaybackfile.BaseStream.Length) {

						byte[] packet = mine.readPacket ();
						object data = mine.GetPacket (packet);
						if (data == null)
							continue;

						if (data is MAVLink.mavlink_heartbeat_t) {
							if (((MAVLink.mavlink_heartbeat_t)data).type == (byte)MAVLink.MAV_TYPE.GCS)
								continue;
						}

						Type test = data.GetType ();

						DateTime time = mine.lastlogread;

						//double matlabtime = GetMatLabSerialDate(time);

						try {
							foreach (var field in test.GetFields()) {
								// field.Name has the field's name.

								// Only log those fields that match strings in fieldMatch.  If fieldMatch
								// is empty, then log every field.
								bool foundMatch = false;
								if (fieldMatch==null) {
									foundMatch=true;
								} else {
									foreach (string fm in fieldMatch) {
										bool result = fm.Equals(field.Name, StringComparison.Ordinal);
										if (result==true) {
											foundMatch=true;
										}
									}
								}

								if (foundMatch) {
									object fieldValue = field.GetValue (data); // Get value
									if (field.FieldType.IsArray) {
										//Console.Write ("*************** Found array: " + field.Name + "_._" + field.DeclaringType.Name + "\n");
									} else {
										if (!datappl.ContainsKey (field.Name + "_" + field.DeclaringType.Name)) {
											datappl [field.Name + "_" + field.DeclaringType.Name] = new List<double[]> ();
											numEntries++;
											if (VERBOSE) {
												Console.WriteLine ("Found new key ({0}): {1}_._{2} ", numEntries, field.Name, field.DeclaringType.Name);
											}
										}

										List<double[]> list =
	                                    ((List<double[]>)datappl [field.Name + "_" + field.DeclaringType.Name]);

										object value = fieldValue;

										if (value.GetType () == typeof(Single)) {
											list.Add (new double[] { (double) (Single) field.GetValue(data) });
												file.WriteLine("{0}_._{1}, {2}", field.Name, field.DeclaringType.Name, (Single) field.GetValue(data)); 
											//Console.WriteLine("Newest value = " + (Single) field.GetValue(data));
										} else if (value.GetType () == typeof(short)) {
											list.Add (new double[] { (double) (short) field.GetValue(data) });
												file.WriteLine("{0}_._{1}, {2}", field.Name, field.DeclaringType.Name, (short) field.GetValue(data));
										
											//Console.WriteLine("Newest value = " + (short) field.GetValue(data));
										} else if (value.GetType () == typeof(ushort)) {
											list.Add (new double[] { (double) (ushort) field.GetValue(data) });
												file.WriteLine("{0}_._{1}, {2}", field.Name, field.DeclaringType.Name, (ushort) field.GetValue(data));

											//Console.WriteLine("Newest value = " + field.GetValue(data));
										} else if (value.GetType () == typeof(byte)) {
											list.Add (new double[] { (double) (byte) field.GetValue(data) });
												file.WriteLine("{0}_._{1}, {2}", field.Name, field.DeclaringType.Name, (byte) field.GetValue(data));
												
											//Console.WriteLine("Newest value = " + field.GetValue(data));
										} else if (value.GetType () == typeof(sbyte)) {
											list.Add (new double[] { (double) (sbyte) field.GetValue(data) });
												file.WriteLine("{0}_._{1}, {2}", field.Name, field.DeclaringType.Name, (sbyte) field.GetValue(data));
													
											//Console.WriteLine("Newest value = " + field.GetValue(data));
										} else if (value.GetType () == typeof(Int32)) {
											list.Add (new double[] { (double) (Int32) field.GetValue(data) });
												file.WriteLine("{0}_._{1}, {2}", field.Name, field.DeclaringType.Name, (Int32) field.GetValue(data));
													
											//Console.WriteLine("Newest value = " + field.GetValue(data));
										} else if (value.GetType () == typeof(UInt32)) {
											list.Add (new double[] { (double) (UInt32) field.GetValue(data) });
												file.WriteLine("{0}_._{1}, {2}", field.Name, field.DeclaringType.Name, (UInt32) field.GetValue(data));
														
											//Console.WriteLine("Newest value = " + field.GetValue(data));
										} else if (value.GetType () == typeof(ulong)) {
											list.Add (new double[] { (double) (ulong) field.GetValue(data) });
												file.WriteLine("{0}_._{1}, {2}", field.Name, field.DeclaringType.Name, (ulong) field.GetValue(data));
															
											//Console.WriteLine("Newest value = " + field.GetValue(data));
										} else if (value.GetType () == typeof(long)) {
											list.Add (new double[] { (double) (long) field.GetValue(data) });
												file.WriteLine("{0}_._{1}, {2}", field.Name, field.DeclaringType.Name, (long) field.GetValue(data));
																
											//Console.WriteLine("Newest value = " + field.GetValue(data));
										} else if (value.GetType () == typeof(double)) {
											list.Add (new double[] { (double) (double) field.GetValue(data) });
												file.WriteLine("{0}_._{1}, {2}", field.Name, field.DeclaringType.Name, (double) field.GetValue(data));
																
											//Console.WriteLine("Newest value = " + field.GetValue(data));
										} else {
											Console.WriteLine ("Unknown data type");
										}

									}
								} 		// end if foundMatch
							}       // end foreach (var field in test.GetFields())
						} catch {
						}
					}

					mine.logreadmode = false;
					mine.logplaybackfile.Close ();
					mine.logplaybackfile = null;

				}
            }

			if (VERBOSE) {
				Console.WriteLine ("Num items in hashtable: " + datappl.Count);
			}
        }

        /// <summary>
        /// http://www.mathworks.com.au/help/matlab/matlab_prog/represent-date-and-times-in-MATLAB.html#bth57t1-1
        /// MATLAB also uses serial time to represent fractions of days beginning at midnight; for example, 6 p.m. equals 0.75 serial days.
        /// So the string '31-Oct-2003, 6:00 PM' in MATLAB is date number 731885.75.
        /// </summary>
        /// <param name="dt"></param>
        /// <returns></returns>
        public static double GetMatLabSerialDate(DateTime dt)
        {
            // in c# i cant represent year 0000, so we add one year and the leap year
            DateTime timebase = new DateTime(1, 1, 1); // = 1

            double answer = (dt.AddYears(1).AddDays(1) - timebase).TotalDays;

            return answer;
        }

        public class DoubleList : IDisposable
        {
            Stream file;

            string filename;

            Stream offsetfile;

            string offsetfilename;

            const int offsetsize = sizeof (long);

            public int Count
            {
                get { return (int) (offsetfile.Length/offsetsize); }
            }

            public DoubleList()
            {
                filename = Path.GetTempFileName();
                file = File.Open(filename, FileMode.Create);

                offsetfilename = Path.GetTempFileName();
                offsetfile = File.Open(offsetfilename, FileMode.Create);
            }

            void setoffset(int index, long offset)
            {
                byte[] data = BitConverter.GetBytes(offset);
                offsetfile.Seek(offsetsize*index, SeekOrigin.Begin);
                offsetfile.Write(data, 0, offsetsize);
            }

            long getoffset(int index)
            {
                byte[] data = new byte[offsetsize];
                offsetfile.Seek(offsetsize*index, SeekOrigin.Begin);
                offsetfile.Read(data, 0, offsetsize);
                return BitConverter.ToInt64(data, 0);
            }

            ~DoubleList()
            {
                Dispose();
            }

            public void Dispose()
            {
                offsetfile.Close();
                offsetfile = null;

                file.Close();
                file = null;

                File.Delete(filename);
                File.Delete(offsetfilename);
            }

            public double[] this[int index]
            {
                get
                {
                    // init a buffer
                    byte[] buffer = new byte[sizeof (double)];
                    // seek to the offset of this index we want
                    file.Seek(getoffset(index), SeekOrigin.Begin);
                    // read the number of elements following
                    file.Read(buffer, 0, sizeof (int));
                    int elements = BitConverter.ToInt32(buffer, 0);

                    // read the elements
                    List<double> data = new List<double>();
                    for (int a = 0; a < elements; a++)
                    {
                        file.Read(buffer, 0, buffer.Length);
                        data.Add(BitConverter.ToDouble(buffer, 0));
                    }
                    // return the data
                    return data.ToArray();
                }
            }

            public int Add(double[] items)
            {
                // goto the end of the file
                file.Seek(0, SeekOrigin.End);
                // save the position of the data following
                setoffset(Count, file.Position);
                // save the number of elements following
                file.Write(BitConverter.GetBytes(items.Length), 0, sizeof (int));
                // save the elements
                foreach (var item in items)
                {
                    file.Write(BitConverter.GetBytes(item), 0, sizeof (double));
                }

                // return the index
                return Count;
            }

            public double[][] ToArray()
            {
                double[][] data = new double[Count][];

                for (int a = 0; a < Count; a++)
                {
                    data[a] = this[a];
                }

                return data;
            }
        }
    }
}