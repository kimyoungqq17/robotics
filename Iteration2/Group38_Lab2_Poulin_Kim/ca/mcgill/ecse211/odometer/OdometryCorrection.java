package ca.mcgill.ecse211.odometer;
/*
 * OdometryCorrection.java
 */

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer; //setup the odometer to correct position from

  //setting up the port in which the ColorSensor is connected
  private static final Port csPort = LocalEV3.get().getPort("S1");
  
  //Creating a color sensor instance and a sample provider (Standard Sensor setup)
  private SensorModes ColorSampler = new EV3ColorSensor(csPort); 
  private SampleProvider sampleColor = ColorSampler.getMode("Red"); //we setup the sensor in RED mode to get the reflected light
  private float[] sensorData = new float[ColorSampler.sampleSize()]; 
  
  //Number of lines counted by the color sensor in x and y
  int XlineCount=0;
  int YlineCount=0;
  
  //environment-related variables: offset start position of the robot and width of a square tile
  public static final double XY_OFFSET=15.24;
  public static final double S_OFFSET=6;
  public static final double SQ_WIDTH=30.48;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    //OdometryCorrection.ColorSampler= new EV3ColorSensor(csPort);

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    double Xpos, Ypos, T;//variables used in the loop 

    //we set the starting position of the robot to be the center of the tile at south-west of (0,0)
    odometer.setXYT(-XY_OFFSET, -XY_OFFSET, 0);
    
    while (true) {
      correctionStart = System.currentTimeMillis();//take the time measurement
      
      //We get a light reflection measure from the ColorSensor
      sampleColor.fetchSample(sensorData,0);
      
      //We get the current odometer data from our odometer object and store in into an array that we will modify
      double[] odoData=odometer.getXYT();
      
      
      //Since the average light reflection value of the tiles is 0.7 and the average light reflection of the lines is 0.3,
      //we set 0.5 as a threshold for detecting a line.
      
      //if the sensor value is below 0.5, it means that the sensor was on a line
      if(((sensorData[0]))<0.5) {
    	  
    	  //We make the robot beep to let the user know that we met a line
    	  Sound.beep();
    	  
    	  //at this moment, we assign position variables to the array (easier to recognize which index is what)
    	  Xpos=odoData[0];//X position
    	  Ypos=odoData[1];//Y position
    	  T=odoData[2];//value of theta
    	  
    	  //If the robot has not encountered 3 lines in Y yet, it means we are still drawing the first edge of the square.
    	  if(YlineCount<3) { 
    		  
    		  //Xpos=(-(XY_OFFSET));//We know in the first edge, the X has not changed yet. We set it to the initial position
    		  Ypos=(YlineCount*SQ_WIDTH)+S_OFFSET;//Depending on the number of lines we encountered, we are at a different position. We add the offset of the distance between the robot and the sensor
    		  							//Line count is the number of tiles 
    		  T=0;//in the first edge, theta is still 0, because we havent rotated yet.
    		  YlineCount++; //increment YlineCount to adjust the distance a tile further next time
    	  }
    	  
    	  //If the robot has encountered 3 lines in Y, but not 3 lines in X, it means we are in the second edge of the square.
    	  else if (XlineCount<3) {
    		  
    		  Xpos=(XlineCount*SQ_WIDTH)+S_OFFSET;//Depending on the number of lines we encountered, we are at a different position.
    		  							//Line count is the number of tiles (t the first line, the count is still 0, 
    		  							//so the position will be 0, which is correct. We subtract the offset of the distance between the robot and the sensor
    		  T=90; //We rotated once, so theta is 90 degs
    		  XlineCount++; //increment XlineCount to adjust the distance a tile further next time 
    	  }
    	  
    	  //if we have crossed at least 3 lines in Y and 3 lines in X, we know we are in the 3rd edge of the square
    	  else if(YlineCount<6) {
    		  
    		 Ypos=(SQ_WIDTH*(5-YlineCount));//we correct the Y position using the sensor offset and the number of lines
    		 T=180; //we are at 180 degrees
    		 YlineCount++;//increment number of lines for y
    	  }
    	  else if(XlineCount<6) {//we are now in the fourth edge of the square
    		  Xpos=(SQ_WIDTH*(5-XlineCount))+S_OFFSET;//we correct the X position using the sensor offset and the number of lines
    		  T=270;// we are at 270 degrees
    		  XlineCount++;//increment number of lines for x
    	  }
    	  odometer.setXYT(Xpos, Ypos, T);//update position
    	  
      }
      
      
      
      

      // TODO Trigger correction (When do I have information to correct?) //knowing when there is a line
      // TODO Calculate new (accurate) robot position //line count

      // TODO Update odometer with new calculated (and more accurate) vales //update(odo)
      //odometer.setXYT(, 19.23, 5.0);
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
