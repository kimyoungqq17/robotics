package ca.mcgill.ecse211.lab4;
import ca.mcgill.ecse211.lab4.Lab4;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class UltrasonicLocalizer {
	
	private static final int ROTATE_SPEED = 75;
  	private EV3LargeRegulatedMotor leftMotor;
  	private EV3LargeRegulatedMotor rightMotor;
  	Odometer odo;
  	public int edgeType;
  	//private Navigation nav;
  	private float[] usData;
  	private SampleProvider us;
  	
  	private int filterControl = 0;
  	private int filterOut = 10;
  	private float lastDistance = 0;
  	
  	public static final double WHEEL_RAD=2.2;
  	public static final double TRACK = 12.25;
  	
	double angle1, angle2, origin;
  	
	/**
	 * This constructor creates an ultrasonic localization object that can travel to the origin
	 * @param odo Odometer used 
	 * @param leftMotor Left motor of the robot
	 * @param rightMotor Right motor of the robot
	 * @param us Ultrasonic sensor
	 * @param usData Buffer for ultrasonic sensor data
	 * @param edgeType Falling/Rising Edge (Falling=1, Rising=2)
	 * @throws OdometerExceptions
	 */
  	public UltrasonicLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor , EV3LargeRegulatedMotor rightMotor, SampleProvider us, float [] usData, int edgeType) throws OdometerExceptions {
  		
  		this.odo = Odometer.getOdometer(); //get current odo
  		this.leftMotor = leftMotor;//left, right motors
  		this.rightMotor = rightMotor;
  		this.us = us;//sensor
  		this.edgeType = edgeType;//falling or rising edge
  		this.usData = usData;//sensor buffer
  	}
  	
  	/**
  	 * The method localizeUS takes the position of the robot and sweeps for a change in the wall distance, depending on the edge type
  	 * @param type Falling or Rising Edge
  	 */
  	public void localizeUS(int type) {
  		
  		//setup motor speed
  		leftMotor.setSpeed(ROTATE_SPEED);
  		rightMotor.setSpeed(ROTATE_SPEED);
  		
  		if(type == 1) {//if the type is one
  			
  			fallingEdge();//do falling edge
  		}
  		else {
  			
  			risingEdge();//do rising edge
  		}
  	}
  	/**
  	 * The method falling Edge is used when the robot is NOT facing the wall. In that case, it will check for a reduction of distance and sweep for angle
  	 */
  	public void fallingEdge() {
  		
  		
  		while(getData() < 30) {//while distance is below 30cm (if the robot is not well located)
  			
  			rightMotor.backward();//rotate
  			leftMotor.forward();
  		}
  		
  		while(getData() > 30) {//then, while distance stays over 30cm
  			
  			rightMotor.backward();//still rotate
  			leftMotor.forward();
  			
  		}
  		
  		angle1 = (odo.getXYT())[2];//note angle
  		
  		while(getData() < 30) {//keep rotating in the other way
  			
  			rightMotor.forward();
  			leftMotor.backward();
  		}
  		
  		while(getData() > 30) {
  			
  			rightMotor.forward();			
  			leftMotor.backward();
  		}
  		
  		leftMotor.stop();
  		rightMotor.stop();
  		
  		angle2 = (odo.getXYT())[2];//get second angle
  		
  		if(angle1>angle2) {
  			angle1-=360;
  		}
  		
  		double avgAngle = (angle1 + angle2)/2;//calculate the average angle
		origin =  angle2 - avgAngle;//we turn to the right angle
		turnTo(origin);
  	}
  		
  	/**
  	 * The method falling Edge is used when the robot is facing the wall. In that case, it will check for an increase of distance and sweep for angle
  	 */
  	 public void risingEdge() {
  		 
  		 while(getData() > 30) {//while distance is over 30, just in case
  			
  			rightMotor.backward();//rotate
  			leftMotor.forward();
  		}
  		
  		while(getData() < 30) {//if distance is below 30 (we are facing a wall)
  			
  			rightMotor.backward();
  			leftMotor.forward();
  			
  		}
  		
  		angle1 = (odo.getXYT())[2];//get angle
  		
  		while(getData() > 30) {
  			
  			rightMotor.forward();//rotate until we see a wall
  			leftMotor.backward();
  		}
  		
  		while(getData() < 30) {//rotate until we do not see a wall anymore
  			
  			rightMotor.forward();
  			leftMotor.backward();
  		}
  		
  		leftMotor.stop();
  		rightMotor.stop();
  		
  		angle2 = (odo.getXYT())[2];//note angle
  		
  		if(angle2>angle1) {
  			angle2-=360;
  		}
  		
  		double avgAngle = (angle1 + angle2)/2;
		origin =  angle2 - avgAngle -45 ;//move to the origin
		turnTo(origin);
  		 
  	 }
  	 /**
  	  * The method getData gets the distance from ultrasonic sensor
  	  * @return
  	  */
  	 public float getData() {

  			us.fetchSample(usData, 0);
  			float distance = (int)(usData[0]*100.0);
  			float result = 0;
  			if (distance > 50 && filterControl < filterOut) {
  				// bad value, do not set the distance var, however increment the filter value
  				filterControl ++;
  				result = lastDistance;
  			} else if (distance > 255){
  				// true 255, therefore set distance to 255
  				result = 50; //clips it at 50
  			} else {
  				// distance went below 255, therefore reset everything.
  				filterControl = 0;
  				result = distance;
  			}
  			lastDistance = distance;
  			
  			return result;
  		}
  	 
  	 /**
  	  * the method turnTo turns to the right angle form a given odo angle
  	  * @param theta Angle we want to be turning of
  	  */
  	public void turnTo(double theta) {

			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK , theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		}
  	
  	/**Converts the angle to turn of to a distance so it is easier to use with the rotate mathod of the motors.*/
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	/**Converts the distance to turn of to a rotating distance so it is easier to use with the rotate mathod of the motors.*/
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
}