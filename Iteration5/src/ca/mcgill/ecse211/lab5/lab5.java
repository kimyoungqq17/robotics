package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.localizer.LightLocalizer;
import ca.mcgill.ecse211.localizer.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensor.ColorDetection;
import ca.mcgill.ecse211.sensor.ColorSensor;
import ca.mcgill.ecse211.sensor.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class lab5 {
	
	//INSTATIATION OF OUR MOTORS PORTS 
	public static final EV3LargeRegulatedMotor leftmotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightmotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3MediumRegulatedMotor sensormotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	//INSTATIATION OF OUR SENSOR PORTS
	private static Port usPort = LocalEV3.get().getPort("S1");
	private static Port lsPort = LocalEV3.get().getPort("S3");
	private static Port csPort = LocalEV3.get().getPort("S4");
	private static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(usPort);
	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(lsPort);
	private static final EV3ColorSensor colorSensor = new EV3ColorSensor(csPort);
	
	//Text display
	public static TextLCD lcd = LocalEV3.get().getTextLCD();
	
	//Buffer for the sensor
	static SampleProvider usValue = usSensor.getMode("Distance");
	static float[] usData = new float[usValue.sampleSize()];
	
	static SampleProvider lightValue = lightSensor.getMode("Red");
	static float[] lightData = new float[lightValue.sampleSize()];
	
	static SampleProvider colorValue = colorSensor.getMode("RGB");
	float[] colorData = new float[colorValue.sampleSize()];
	
	//Robot constants
	private static final double TILE_LENGTH = 30.48;
	private static final double TRACK = 12.92;
	private static final double WHEEL_RAD = 2.1;
	static Odometer odo;
	
	static int llangle = 210;
	static boolean target_detected = false;
	private static int detected_color = 4;
	static String[] color_base = {"BLUE", "GREEN", "YELLOW", "RED", "OTHER"};
	
	//Grid parameters
	
	static int LLx = 1;
	static int LLy = 2;
	static int URx = 4;
	static int URy = 4;
	static int TR = 2;
	static int SC = 0;
	
	/**This is the main method where we start the program and activate all the threads. 
	 * We show our display on the screen and start the program according to the user's input
	 */  
	
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {
		
		int buttonChoice;
		do {
		      // clear the display
		      lcd.clear();

		      // ask the user whether the motors should drive in a square or float
		      lcd.drawString("< Left | Right >", 0, 0);
		      lcd.drawString("       |        ", 0, 1);
		      lcd.drawString(" Color | Field  ", 0, 2);
		      lcd.drawString(" test  | test   ", 0, 3);
		      lcd.drawString("       |        ", 0, 4);

		      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		
		lcd.clear();
		
		//Color Testing Part of the demo
		if(buttonChoice == Button.ID_LEFT) {
			
			final UltrasonicPoller myusData = new UltrasonicPoller(usValue);
			myusData.start();
			ColorSensor mylsData = new ColorSensor(colorValue);
			mylsData.start();
			Thread.sleep(1000);
			while(true) {
				//if the ultrasonic sensor has a reading less than or equal to 5 cm, we consider that there is a can and the robot stops and scans it
				if(myusData.getDistance()<=5) {				
					lcd.clear();
					lcd.drawString("can detected", 0, 1);
					detect_color(mylsData);
					lcd.drawString("color: "+color_base[detected_color], 0, 2); //display the color scanned
					Button.waitForAnyPress();
				} 
			}
			
			//Field test 
		}else {
			odo = Odometer.getOdometer(leftmotor, rightmotor, TRACK, WHEEL_RAD); //initialization of the odometer parameters
			Thread odoThread = new Thread(odo);
			odoThread.start();
			
			//initialization of navigation
			Navigation nav = new Navigation(leftmotor,rightmotor);
			nav.start();
			
			leftmotor.setAcceleration(1000);
			rightmotor.setAcceleration(1000);
			
			lcd.drawString("Press to start", 0, 1);
			Button.waitForAnyPress();
			
			//Start the localization part: USsensor localization
			UltrasonicLocalizer usl = new UltrasonicLocalizer (odo, usValue, usData);
			usl.localizeUS();
			nav.turnTo(0);
			lcd.drawString("US finished", 0, 2);
			
			// Light localization
			turnTo(llangle);
			LightLocalizer lsl = new LightLocalizer (odo, lightValue, lightData);
			lsl.Localize();
			nav.travelTo(TILE_LENGTH,TILE_LENGTH,false);
			nav.turnTo(0);
			lcd.drawString("LL finished", 0, 3);
			
			Thread.sleep(1000);
			Button.waitForAnyPress();
					
			final UltrasonicPoller myusData = new UltrasonicPoller(usValue);
			myusData.start();
			ColorSensor mylsData = new ColorSensor(colorValue);
			mylsData.start();
			Thread.sleep(1000);
			
			travelTo_LL(SC, nav);
			 
			//initialization of the odometer data
			odo.setXYT(0, 0, odo.getXYT()[2]);
			 
			Thread.sleep(3000);
			 
			lcd.clear();
			lcd.drawString("can detection starts", 0, 3);
			
			
			for(int x = 0;x<=URx-LLx;x++) {								
		    	if(x%2==1) {
		    		//To make the robot go downwards on the Y axis if it is on a X coordinate that is odd (x=1, x=3,x=5...)
		    		for(int y=URy-LLy;y>=0;y--) {
		    			detect_can(myusData, mylsData, x, y, nav);  //checks if there is a can each time it crosses an intersection
		    			Thread.sleep(500);
		    			if(target_detected) break;
		    		}
		    		//To make the robot go upwards on the Y axis if it is on a X coordinate that is even (SC=0, x=2,x=4...)
		    	}else {
		    		for(int y=0;y<=URy-LLy;y++) {
		    			detect_can(myusData, mylsData,x, y, nav); //checks if there is a can each time it crosses an intersection
		    			Thread.sleep(500);
		    			if(target_detected) break;
		    		}
		    	}
		    	if(target_detected) {
		    		lcd.drawString("target found", 0, 4);
		    	    break;
		    	}
		    }
			//when target can is found, travel to the UpperRight corner
			double correction_x = odo.getXYT()[0] + LLx*TILE_LENGTH;
			double correction_y = odo.getXYT()[1] + LLy*TILE_LENGTH;
			double correction_theta = odo.getXYT()[2];
			odo.setXYT(correction_x, correction_y, correction_theta);
			
			travelTo_UR(nav);
		}
		
	}
	
	// if the starting center was not the same as the LowerLeft where
	/**
	 * This method travels to the LowerLeft Corner of the search 
	 * zone to start the navigation no matter which corner the
	 *  robot is placed in by the TA.
	 * 
	 * @param SC starting corner of the robot
	 * @param nav instance of navigation class
	 */
	static void travelTo_LL(int SC, Navigation nav) {
		if(SC == 0) {
			nav.travelTo(LLx*30.48, LLy*30.48, false);
		}else if(SC == 1) {
			double angle_correction = odo.getXYT()[2]+270;
			if(angle_correction > 360) {
				angle_correction = angle_correction - 360;
			}
			odo.setXYT(odo.getXYT()[0]+6*TILE_LENGTH, odo.getXYT()[1], angle_correction);
			nav.travelTo(odo.getXYT()[0], LLy*30.48-TILE_LENGTH/2, false);
			nav.travelTo(LLx*30.48, odo.getXYT()[1], false);
			nav.travelTo(LLx*30.48, LLy*30.48, false);
		}else if(SC == 2) {
			double angle_correction = odo.getXYT()[2]+180;
			if(angle_correction > 360) {
				angle_correction = angle_correction - 360;
			}
			odo.setXYT(odo.getXYT()[0]+6*TILE_LENGTH, odo.getXYT()[1]+6*TILE_LENGTH, angle_correction);
			nav.travelTo(odo.getXYT()[0]+TILE_LENGTH/2,odo.getXYT()[1], false);
			nav.travelTo(odo.getXYT()[0], LLy*30.48-TILE_LENGTH/2, false);
			nav.travelTo(LLx*30.48, odo.getXYT()[1], false);
			nav.travelTo(LLx*30.48, LLy*30.48, false);
		}else {
			double angle_correction = odo.getXYT()[2]+90;
			if(angle_correction > 360) {
				angle_correction = angle_correction - 360;
			}
			odo.setXYT(odo.getXYT()[0], odo.getXYT()[1]+6*TILE_LENGTH, angle_correction);
			nav.travelTo(LLx*30.48-TILE_LENGTH/2, odo.getXYT()[1], false);
			nav.travelTo(odo.getXYT()[0], LLy*30.48, false);
			nav.travelTo(LLx*30.48, LLy*30.48, false);
		}
	}
	
	
	/**
	 * This method gets the current location of the robot and makes it travel to the upper right corner of the search zone
	 * @param nav
	 */
	static void travelTo_UR(Navigation nav) {
		leftmotor.rotate(-convertDistance(WHEEL_RAD,5),true);
		rightmotor.rotate(-convertDistance(WHEEL_RAD,5),false);
		nav.travelTo(odo.getXYT()[0]+15,odo.getXYT()[1],false);
		nav.travelTo(odo.getXYT()[0],URy*30.48-15,false);
		nav.travelTo(URx*30.48-15,odo.getXYT()[1],false);
		nav.travelTo(URx*30.48,URy*30.48,false);
	}
	
	/**
	 * The following method goes around the can to avoid hitting it after it is done scanning it
	 */
	public static void avoid_can() {
		leftmotor.rotate(-convertDistance(WHEEL_RAD,5),true);
		rightmotor.rotate(-convertDistance(WHEEL_RAD,5),false);
		leftmotor.rotate(convertAngle(WHEEL_RAD,TRACK,90),true);
		rightmotor.rotate(-convertAngle(WHEEL_RAD,TRACK,90),false);
		leftmotor.rotate(convertDistance(WHEEL_RAD,15),true);
		rightmotor.rotate(convertDistance(WHEEL_RAD,15),false);
		leftmotor.rotate(-convertAngle(WHEEL_RAD,TRACK,90),true);
		rightmotor.rotate(convertAngle(WHEEL_RAD,TRACK,90),false);
		leftmotor.rotate(convertDistance(WHEEL_RAD,15),true);
		rightmotor.rotate(convertDistance(WHEEL_RAD,15),false);
	}
	
	
	
	public static void detect_can(UltrasonicPoller myData,ColorSensor mylsData, int x, int y, Navigation nav) throws InterruptedException {
		leftmotor.setSpeed(200);
		rightmotor.setSpeed(200);
		leftmotor.setAcceleration(1500);
		leftmotor.setAcceleration(1500);

		nav.travelTo(x*30.48,y*30.48,true);
		while(leftmotor.isMoving()) {
			if(myData.getDistance()<=5) {				//if the ultrasonic sensor has a reading less than or equal to 5 cm, wo consider that there is a can and the robot stops and scan it
				leftmotor.stop(true);
				rightmotor.stop();
				lcd.clear();
				lcd.drawString("can detected", 0, 1);
				detect_color(mylsData);
				//SensorData mylsData = new SensorData();
				if(detected_color == TR) {		//find out if this is the target can or not
					target_detected = true;
					Sound.beep();							//if yes, beep once
					lcd.drawString("color: "+color_base[detected_color], 0, 2);
					break;
				}else {										//if no, beep twice
					Sound.beep();
					Sound.beep();
					lcd.drawString("color: "+color_base[detected_color], 0, 2);
				}
				
				//go around the can
				avoid_can();
				
			} 
		}
		
		
	}
	
	public static int getLightData(SampleProvider cs, float[] cd) {
		cs.fetchSample(cd,0);
		int color = (int)cd[0] * 1000;
		
		return color;
	}

	/**
	 * This method is used to detect the color of the can, and update it to the detected_color.
	 * Each time the a sample scan is taking, a color is assigned to it.
	 * The color array is used to keep track of the number of occurrences of a color.
	 * The color that occurred the most times, thus the color having the highest value in the array is the color printed to the LCD screen.
	 * @param myData This is the light sensor data provider
	 */
	public static void detect_color(ColorSensor myData) {
		sensormotor.setSpeed(70);
		sensormotor.rotate(180,true);
		int[] color = {0,0,0,0,0};
		while(sensormotor.isMoving()) {
			int result = ColorDetection.printColor(myData.RGBVal);
			color[result]++;
			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
		//find out the color detected most times
		int index_number = 0;
		int largest = color[0];	
		for (int i = 0; i<color.length-1;i++) {
			if(largest<color[i]) {
				index_number = i;
				largest = color[i];
			}
		}
		//Bring back the rotating sensor to its initial position (back to 0after it turned to 180 degrees)
		sensormotor.rotate(-180);
		detected_color = index_number;	//set to detected_color
	}
	
	
	/**
	 * This method calculates the distance to be traveled by the robot to reach the new coordinate from its current position and travels to it without stopping the motors.
	 * @param x
	 * @param y
	 * @param immediate
	 */
	
	public static void travelTo(double x, double y, boolean immediate) {
		
		//navigating = true;
		
		// We compute the turn angle
		double dX = x - odo.getXYT()[0]; //remaining x distance
		double dY = y - odo.getXYT()[1]; //remaining y distance
		double turn_angle = Math.atan2(dX, dY);
		
		// We rotate
		leftmotor.setSpeed(200);
		rightmotor.setSpeed(200);
		turnTo(Math.toDegrees(turn_angle));
		
		double distance = Math.hypot(dX, dY);
		
		// We move to waypoint
		leftmotor.setSpeed(200);
		rightmotor.setSpeed(200);
		leftmotor.rotate(convertDistance(WHEEL_RAD,distance),true);
		rightmotor.rotate(convertDistance(WHEEL_RAD,distance),immediate);
	}
	
	/**
	 * This method calculates the distance to be traveled by the robot to reach the new coordinate from its current position and travels to it after stopping the motors.
	 * @param x
	 * @param y
	 */
	public static void travelTo(double x, double y) {
		
		//Reset motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftmotor, rightmotor}) {
			motor.stop();
			motor.setAcceleration(1500);
		}
		
		//navigating = true;
		
		// We compute the turn angle
		double dX = x - odo.getXYT()[0]; //remaining x distance
		double dY = y - odo.getXYT()[1]; //remaining y distance
		double turn_angle = Math.atan2(dX, dY);
		
		// We rotate
		leftmotor.setSpeed(200);
		rightmotor.setSpeed(200);
		turnTo(Math.toDegrees(turn_angle));
		
		double distance = Math.hypot(dX, dY);
		
		// We move to waypoint
		leftmotor.setSpeed(200);
		rightmotor.setSpeed(200);
		leftmotor.rotate(convertDistance(WHEEL_RAD,distance),true);
		rightmotor.rotate(convertDistance(WHEEL_RAD,distance));
	}
	
/** 
 * Takes the new heading as input and make the robot turn to it
 * @param: double theta that represents an angle in radians
 */
	public static void turnTo(double theta) {
	
		double angle = getMinAngle(theta-odo.getXYT()[2]);
	
		leftmotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle),true);
		rightmotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle),false);
	}

	/**
	 * Gets the smallest value (between 180 and -180) of an angle
	 * @param angle
	 * @return angle the minimum angle to turn
	 */
	public static double getMinAngle(double angle){
		if (angle > 180) {  //Pi = 180 degrees
			angle -= 2*180; 
		} else if (angle < -180) {
			angle = angle + 2*180;
		}
		return angle;
	}

	/**This method takes the radius of wheels and the distance that needs to be traveled, and calculate 
	 * the rotation of the wheels need to make to travel that distance
	 * @param radius is the wheel radius
	 * @param distance is the distance needs to be traveled
	 * @return rotation of the wheels
	 */
	
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**This method take the radius of wheels, width of the robot and the angle needs to be rotated by the robot, 
	 * to calculate the rotation needs to make by wheels to turn a certain angle to a certain direction
	 * 
	 * @param radius is the radius of wheels
	 * @param width is the width of the robot
	 * @param angle is the angle needs to be turned by the robot
	 * @return rotation of wheels
	 */
	private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}