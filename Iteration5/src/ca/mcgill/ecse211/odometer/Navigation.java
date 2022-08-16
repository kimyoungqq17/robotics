package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class Navigation extends Thread{
	private Odometer odometer;								//odometer for the navigation process
	private EV3LargeRegulatedMotor leftMotor;				//left motor of the robot
	private EV3LargeRegulatedMotor rightMotor;				//right motor of the robot			
	
	static int distance; 
	public static final double WHEEL_RAD = 2.1; //wheel radius
	public static final double TRACK = 12.9;	//distance between two wheels
	private static boolean navigating = false;	//show navigation is running or not
	
	
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions{
		odometer = Odometer.getOdometer();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

	}
	

	/**
	 * The following method is used to make the robot go to a preset location
	 * @param x
	 * @param y
	 * @param immediate
	 */
	
	public void travelTo(double x, double y, boolean immediate) {
		
		//navigating = true;
		
		// We compute the turn angle
		double dX = x - odometer.getXYT()[0]; //remaining x distance
		double dY = y - odometer.getXYT()[1]; //remaining y distance
		double turn_angle = Math.atan2(dX, dY);
		
		// We rotate
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		turnTo(Math.toDegrees(turn_angle));
		
		double distance = Math.hypot(dX, dY);
		
		// We move to waypoint
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.rotate(convertDistance(WHEEL_RAD,distance),true);
		rightMotor.rotate(convertDistance(WHEEL_RAD,distance),immediate);
	}
	
	
	
	
	/**
	 * The following method is used to turn to the preset direction of the destination point
	 * @param theta
	 */
	
	
	 public void turnTo(double theta) {
		
		double angle = getMinAngle(theta-odometer.getXYT()[2]);
	
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle),true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle),false);
	}

	/**
	 * Gets the smallest value (between 180 and -180) of an angle
	 * @param angle
	 * @return angle
	 */
	 
	public static double getMinAngle(double angle){
		if (angle > 180) {  //Pi = 180 degrees
			angle -= 2*180; 
		} else if (angle < -180) {
			angle = angle + 2*180;
		}
		return angle;
		
	}
	
	/**
	 * The following method is used to determine whether or not the robot is rotating or not
	 * @return navigating state of the robot
	 */
	boolean isNavigation() {
		return navigating;
	}
	
	
	/**
	 * This method is used to convert distance to rotating angle
	 * @param radius
	 * @param distance
	 * @return rotation of the wheel for the robot to travel a specific distance
	 */
	
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	
	/**
	 * This method is used to convert turning angle to rotating angle
	 * @param radius
	 * @param width
	 * @param angle
	 * @return rotation of the wheel for the robot to turn a specific angle
	 */
	private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	
}
