using System;
using MissionPlanner;

namespace TLogReaderV5
{
	class MainClass
	{
		public static void Main (string[] args)
		{
			int nargs;
			string inputFileName = null;
			string outputFileName = null;
			Boolean VERBOSE = false;
			string[] fieldMatch = null;
			string fieldMatchFile = null;

			nargs = args.Length;
			if (nargs == 0) {
				Console.WriteLine ("Error, no tlog file specified.");
				return;
			} else if (nargs >= 1) { 
				inputFileName = args [0];
				string[] stringSeparators = new string[] {"."};
				string[] tmpStr = inputFileName.Split(stringSeparators, StringSplitOptions.None);
				int inLen = inputFileName.Length;
				int tmpStrLen = tmpStr.Length;
				if (String.Equals("tlog",tmpStr[tmpStrLen-1])) {
//				if ((String.Equals (inputFileName [inLen - 5], '.')) & 
//					(String.Equals (inputFileName [inLen - 4], 't')) & 
//					(String.Equals (inputFileName [inLen - 3], 'l')) & 
//					(String.Equals (inputFileName [inLen - 2], 'o')) & 
//					(String.Equals (inputFileName [inLen - 1], 'g'))) {
					// We have a tlog file, proceed with parsing: args

					if (nargs == 1) {
						// Only have an input filename, so create an output filename.  Leave verbose false and no fieldMatch

						//inputFileName.CopyTo (0, tempStr, 0, inputFileName.Length - 4);
						outputFileName = String.Concat (tmpStr[0], ".txt");
					} else if (nargs == 2) {
						outputFileName = args [1];
					} else if (nargs == 3) { 
						outputFileName = args [1];
						if (args [2].Equals ("true")) {
							VERBOSE = true;
						} else {
							VERBOSE = false;
						}
					} else if (nargs == 4) {
						outputFileName = args [1];
						if (args [2].Equals ("true")) {
							VERBOSE = true;
						} else {
							VERBOSE = false;
						}
						fieldMatchFile = args [3];
						fieldMatch = MissionPlanner.Log.TLogUtils.getFields (fieldMatchFile);
					}
				} else {
					// The input file name did not have a .tlog extension, so we won't parse it
					Console.WriteLine ("Error, specified file does not have a .tlog extension: {0}", inputFileName);
					return;
				}
			}



			if (VERBOSE) {
				if (fieldMatchFile == null) {
					Console.WriteLine ("Parsing all fields in tlog file {0}, outputting to {1}.  ", inputFileName, outputFileName);
				} else {
					Console.WriteLine ("Parsing tlog file {0}, outputting to {1}. Matching fields from {2}", inputFileName, outputFileName, fieldMatchFile);
				}
			}

			MissionPlanner.Log.TLogUtils.tlog (inputFileName, outputFileName, VERBOSE, fieldMatch);
		}
	}
}
