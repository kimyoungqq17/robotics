package ca.mcgill.ecse211.sensor;

public class ColorDetection {
	private static final double[] BLUE_RGB= {0.2826, 0.6361, 0.7093};
	private static final double[] BLUE_STD= {0.0677, 0.0626, 0.0638};
	private static final double[] GREEN_RGB= {0.3406, 0.8425, 0.4008};
	private static final double[] GREEN_STD= {0.0792, 0.0411, 0.0764};
	private static final double[] YELLOW_RGB= {0.8591, 0.4727, 0.1767};
	private static final double[] YELLOW_STD= {0.0357, 0.0439, 0.0645};
	private static final double[] RED_RGB= {0.9547, 0.1990, 0.1648};
	private static final double[] RED_STD= {0.0471, 0.1083, 0.0919};

	/**
	 * Instead of using the distance formula in the lab 5 document, we decided to implement a more efficient approach.
	 * We check if the scanned sample is in a range between the mean plus or minus the standard deviation for every color.
	 * If it is, it assigns the sample to this color.
	 * @param RGBvalues This is the RGB value detected by the color sensor
	 * @return	The number of the corresponding color (0 for blue, 1 for green, 2 for yellow, 3 for red, 4 for other)
	 */
	public static int printColor(double[] RGBvalues) {
		
		if((RGBvalues[0]>(BLUE_RGB[0]-BLUE_STD[0])&&RGBvalues[0]<(BLUE_RGB[0]+BLUE_STD[0])) && (RGBvalues[1]>(BLUE_RGB[1]-BLUE_STD[1])&&RGBvalues[1]<(BLUE_RGB[1]+BLUE_STD[1])) && (RGBvalues[2]>(BLUE_RGB[2]-BLUE_STD[2])&&RGBvalues[2]<(BLUE_RGB[2]+BLUE_STD[2]))) {
			return 0;
		}else if((RGBvalues[0]>(GREEN_RGB[0]-GREEN_STD[0])&&RGBvalues[0]<(GREEN_RGB[0]+GREEN_STD[0])) && (RGBvalues[1]>(GREEN_RGB[1]-GREEN_STD[1])&&RGBvalues[1]<(GREEN_RGB[1]+GREEN_STD[1])) && (RGBvalues[2]>(GREEN_RGB[2]-GREEN_STD[2])&&RGBvalues[2]<(GREEN_RGB[2]+GREEN_STD[2]))) {
			return 1;
		}else if((RGBvalues[0]>(YELLOW_RGB[0]-YELLOW_STD[0])&&RGBvalues[0]<(YELLOW_RGB[0]+YELLOW_STD[0])) && (RGBvalues[1]>(YELLOW_RGB[1]-YELLOW_STD[1])&&RGBvalues[1]<(YELLOW_RGB[1]+YELLOW_STD[1])) && (RGBvalues[2]>(YELLOW_RGB[2]-YELLOW_STD[2])&&RGBvalues[2]<(YELLOW_RGB[2]+YELLOW_STD[2]))) {
			return 2;
		}else if((RGBvalues[0]>(RED_RGB[0]-RED_STD[0])&&RGBvalues[0]<(RED_RGB[0]+RED_STD[0])) && (RGBvalues[1]>(RED_RGB[1]-RED_STD[1])&&RGBvalues[1]<(RED_RGB[1]+RED_STD[1])) && (RGBvalues[2]>(RED_RGB[2]-RED_STD[2])&&RGBvalues[2]<(RED_RGB[2]+RED_STD[2]))) {
			return 3;
		}else {
			return 4;
		}
		
}
}
