package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab4 {

  // Motor Objects, and Robot related parameters
private static final Port usPort = LocalEV3.get().getPort("S1");
private static final Port lightPort = LocalEV3.get().getPort("S4");
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.2;
  public static final double TRACK = 12.25;
  
  //public UltrasonicLocalizer myLocalizer;

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;
    int type=0;
    

    // Odometer related objects
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 

    Display odometryDisplay = new Display(lcd); // No need to change


    //get the sensor for distance
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); 
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];
    
    //get the senspr for color
    SensorModes lightSensor = new EV3ColorSensor(lightPort); 
    SampleProvider sampleColor = lightSensor.getMode("Red"); //we setup the sensor in RED mode to get the reflected light
    float[] colorData = new float[sampleColor.sampleSize()]; 
    
    do {
      // clear the display
      lcd.clear();

 
      lcd.drawString("< Left | Right >", 0, 0);
      lcd.drawString("       |        ", 0, 1);
      lcd.drawString("Falling| Rising  ", 0, 2);
      lcd.drawString(" edge  |  edge  ", 0, 3);
      lcd.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

   do {
    if (buttonChoice == Button.ID_LEFT) {
    	type = 1;
    }
    if (buttonChoice == Button.ID_RIGHT) {
    	type = 2;
    }
    
    //setup odometer
    	Thread odoThread = new Thread(odometer);
    	odoThread.start();
    	Thread odoDisplayThread = new Thread(odometryDisplay);
    	odoDisplayThread.start();
    	
    	//create us loc object
    	UltrasonicLocalizer USLoc = new UltrasonicLocalizer(odometer, leftMotor ,rightMotor , usDistance, usData, type);
 
    	USLoc.localizeUS(USLoc.edgeType);
      
      Button.waitForAnyPress();
      
      //new light object
      	LightLocalizer LightLoc = new LightLocalizer(odometer, leftMotor, rightMotor, sampleColor, colorData);
      	LightLoc.localize();

   }
    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
}