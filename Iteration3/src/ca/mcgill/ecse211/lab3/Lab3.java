/*
 * basic skeleton from lab2.java was used
 * we define motors and sesnors
 * we start odo, odo display, us poller, nav thread 
 * we run the code with nav.run() 
 */

package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.ultrasonicpoll.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

  // Motor Objects, and Robot related parameters
  private static final Port usPort = LocalEV3.get().getPort("S1");
  static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  
  
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 12.25;
  
  //map 1
  public static final double [][] map1 = {
		  {0,2},
		  {1,1},
		  {2,2},
		  {2,1},
		  {1,0}
  };
  //map 2
  public static final double [][] map2 = {
		  {1,1},
		  {0,2},
		  {2,2},
		  {2,1},
		  {1,0}
  };
  //map3
  public static final double [][] map3 = {
		  {1,0},
		  {2,1},
		  {2,2},
		  {0,2},
		  {1,1}
  };
  //map4
  public static final double [][] map4 = {
		  {0,1},
		  {1,2},
		  {1,0},
		  {2,1},
		  {2,2}
  };
  

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    
    
    Display odometryDisplay = new Display(lcd); 

    @SuppressWarnings("resource")
    //get the sensor for distance
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); 
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];
    

    Thread odoThread = new Thread(odometer);
    odoThread.start();//starting the thread to run odo
    Thread odoDisplayThread = new Thread(odometryDisplay);
    odoDisplayThread.start();



    //create navigvation object to travel
    final Navigation nav = new Navigation(odometer, leftMotor, rightMotor, usDistance, usData);
    Thread usPoller = new UltrasonicPoller(usDistance, usData, nav);
    usPoller.start();


    // spawn a new Thread to avoid nav.drive() from blocking
    (new Thread() {
      public void run() {
        nav.run();
      }
    }).start();
    

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}