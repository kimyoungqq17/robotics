package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.lab3.Lab3;
import ca.mcgill.ecse211.lab3.NavExceptions;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.ultrasonicpoll.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class Navigation {
	
	private static Navigation nav = null; // Returned as singleton
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	int bandCenter = 17;//width for bang bang
	int bandWidth = 3;
	int distance;//distance from us sensor
	private SampleProvider US;
	private float[] USData;
	
	public static final int MOTOR_HIGH = 220;
	public static final int MOTOR_LOW = 130;
	
	private boolean isNavigating = false;//to check if robot is navigating or wall following
	
	/**This is the constructor for the Navigation class. it takes an odometer, the two motors and the sample provider for the us sensor.*/
	public Navigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider US, float[] USData) {
	
	this.odometer= odometer;
	this.leftMotor = leftMotor;
	this.rightMotor = rightMotor;
	this.US = US;
	this.USData = USData;
	
}
	//map to be implemented
	private double [][] myMap = Lab3.map1;
	
	/** The run method implements travelTo and turnTo. It travels to the waypoints while avoiding obstacles.*/
	public void run() {

	for(double[] map : myMap) {//for every waypoint
		travelTo(map[0],map[1]);
		try {//sleep condition
            Thread.sleep(200);
        } catch (InterruptedException e) {
        }
	}
	}
	
	/**TravelTo makes the robot go from one waypoint to another using minimal angle. if the robot sees an obstacle, it calls avoid.*/
	public void travelTo(double targetx, double targety) {
		
		double x = 30.48* targetx; //coordinates to travelTo
		double y = 30.48* targety;
		isNavigating = true;//we are navigating to the waypoints
		double[] xyt = odometer.getXYT();//get current odo data
		double diffX = x - xyt[0];//x and y distances to travel
		double diffY = y - xyt[1];
		double toTravel = Math.hypot(diffX, diffY);//distance to travel (shortest)
		double theta = Math.toDegrees(Math.atan(diffX/diffY));//theta to turn of so we go directly to the waypoint
		
		
		//depending of the relative position of the waypoint, the theta is different, so we set it accordingly to x and y.
		if((xyt[0] > x) && (xyt[1] > y)) {
			theta += 180;	
		}
		if (( xyt[0] > x) && (xyt[1] < y)) {
			
			theta+=360;
			
		}
		if((xyt[0] < x) && (xyt[1] > y)) {
			
			theta += 180;
		}
		
		turnTo(theta);//we turn to 
		leftMotor.setSpeed(MOTOR_HIGH);
		rightMotor.setSpeed(MOTOR_HIGH);
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, toTravel), true);//go to th enext waypoint
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, toTravel), true);
		
		
		while(leftMotor.isMoving() || rightMotor.isMoving()) {//while the two motors are moving
			
			US.fetchSample(USData,0);//get distance from us
			this.distance=(int)(USData[0]*100.0);
			int distance = this.distance;
			
			if(distance<12) {//if we are close from a block
				
				isNavigating = false;//go into avoidance
				
			}
			
			if(!isNavigating) {
				avoidObstacle();//avoid block
				isNavigating = true;//avoidance finished
				travelTo(targetx,targety);//get back to travelling
			}
			try { Thread.sleep(50); } catch(Exception e){}
		}
			 
	}
	
	/**The method turnTo sets the robot to the minimal angle required to go from a certain point to a waypoint*/
	public void turnTo(double theta) {

		 //method from navigation program
			double minAngle;//minimal angle to turn
			double odoAngle = (odometer.getXYT()[2]);//angle from odometer
			
			if ((theta-odoAngle)>180) {//we change the angle if we are not at the minimal angle
				minAngle = theta - odoAngle - 360;
			}
			else if((theta - odoAngle)<-180) {//we change the angle if we are not at the minimal angle
				minAngle = theta - odoAngle;
			}
			
			else {//no changes to be made on angle
				minAngle = theta - odoAngle;
			}
			
			leftMotor.setSpeed(MOTOR_HIGH);
			rightMotor.setSpeed(MOTOR_HIGH);
			
			//rotate
			leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK , minAngle), true);
			rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, minAngle), false);
			
		}
	
		/** avoidObstacle implements the bang-bang controller to avoid the blocks on the trajectory*/
	public void avoidObstacle() {
		
		double theta = (odometer.getXYT())[2];
		turnTo(theta -90);//initial rotation
		int count=0; //count will be the number of times that the sensor sees a distance greater than 12

		while (true){
			US.fetchSample(USData,0);//get us sensor data				
			distance=(int)(USData[0]*100.0);
			
			if(distance > 12) {
				if(count == 25)break;//if we have recorder a large distance more than 25 times, we get out of the loop. obstacle is considered avoided.
				count++;//increment count
			}
			int error = bandCenter - distance;//calculate the error
			
			if (Math.abs(error)<= bandWidth){ //we are moving straight
				leftMotor.setSpeed(MOTOR_HIGH);
				rightMotor.setSpeed(MOTOR_HIGH);
				leftMotor.forward();
				rightMotor.forward();
			} else if (error > 0){ //too close to the wall
				leftMotor.setSpeed(MOTOR_HIGH);// outer wheel goes backwards
				rightMotor.setSpeed(MOTOR_HIGH); 
				leftMotor.backward();
				rightMotor.forward();
			} else if (error < 0){ // getting too far from the wall
				rightMotor.setSpeed(MOTOR_LOW);
				leftMotor.setSpeed(MOTOR_HIGH);// Setting the outer wheel to move faster to get back on track
				rightMotor.forward();
				leftMotor.forward();
			}
			try {//wait until fetching next sample
	            Thread.sleep(400);
	        } catch (InterruptedException e) {
	        }
		}
		
		leftMotor.stop();
		rightMotor.stop();
	}
	
	/** returns true if the robot is navigating, false if it is avoiding.*/
	public boolean isNavigating() {
		
		return isNavigating;
		
	}
	//reads distance from sensor
	  public int readUSDistance() {
		    return this.distance;
		  }

	public void processUSData(int distance) {
		// TODO Auto-generated method stub
		
	}
	/**Converts the angle to turn of to a distance so it is easier to use with the rotate mathod of the motors.*/
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	/**Converts the distance to turn of to a rotating distance so it is easier to use with the rotate mathod of the motors.*/
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	//exceptions
	 public synchronized static Navigation getNavigation() throws NavExceptions {

		    if (nav == null) {
		      throw new NavExceptions("No previous Navigation exits.");

		    }
		    return nav;
		  }
}

