package ca.mcgill.ecse211.sensor;

import lejos.robotics.SampleProvider;


public class ColorSensor extends Thread{

	double []RGBdata= new double[3];
	double [] canScan = new double[3];
	private float[] colorData;
	SampleProvider colorSensor;
	public double [] RGBVal;
	
	/**
	 * Method that sums up 10 samples of the Color Sensor's RGB values and computes the mean of R, G, and B values.
	 * 
	 * @return Array of mean R, G, B values
	 */
	
	 //get the sensor for color
	
	public ColorSensor(SampleProvider colorSensor ) {
		this.colorSensor = colorSensor;
	    this.colorData = new float[3];
	}
     
	/**
	 * This method gets the RGB values.
	 */
	public void run() {
		while(true) {
			try {
				RGBVal = getRGB();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
    
    /**
     * This method calculate the normalized RGB value and return them as a double array
     * @return A double array of normalized RGB values
     * @throws InterruptedException
     */
	public double[] getRGB() throws InterruptedException {
		
		colorSensor.fetchSample(colorData, 0);
		double normalized_R =colorData[0]/Math.sqrt(colorData[0]*colorData[0]+colorData[1]*colorData[1]+colorData[2]*colorData[2]);
		double nomorlized_G =colorData[1]/Math.sqrt(colorData[0]*colorData[0]+colorData[1]*colorData[1]+colorData[2]*colorData[2]);
		double normalized_B =colorData[2]/Math.sqrt(colorData[0]*colorData[0]+colorData[1]*colorData[1]+colorData[2]*colorData[2]);
		
		return new double[] {normalized_R, nomorlized_G, normalized_B};
   }
}
