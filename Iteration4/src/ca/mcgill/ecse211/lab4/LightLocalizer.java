package ca.mcgill.ecse211.lab4;


import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class LightLocalizer {

	//Constants and objects of lab
	private static final double WHEEL_RADIUS = 2.2;//radius of wheels
	private static final int ROTATE_SPEED = 85;//speed of rotation
	private static final double OFFSET = 9.7;//distance between center of rotation of the robot and light sensor
	private static final double TRACK = 12.25;//distance between the two wheels
	
	private EV3LargeRegulatedMotor leftMotor;//left and right motors
	private EV3LargeRegulatedMotor rightMotor;
	
	//odometer object
	private Odometer odo;
	
	//color sensor object
	private float[] colorData;
  	private SampleProvider sampleColor;
  	
  	//variables to determine the position of the axis
  	private double xNeg, xPos, yNeg, yPos;
  	
  	//value recorded at the beginning as the clear light
  	private float ambientLight;
  	
  	//last recorded value 
  	private float lastValue = 0;
  	
	/**
	 * The LightLocalizer constructor takes motors, the odometer and the Color Sensor and runs the localize method to get to the origin.
	 * @param odo The odometer used for the sensoring
	 * @param leftMotor left motor of the robot
	 * @param rightMotor Right motor of the robot
	 * @param sampleColor Color Sensor
	 * @param colorData Buffer for color sensor
	 * @throws OdometerExceptions If there is no odometer yet created, we throw an exception
	 */
	public LightLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider sampleColor, float[] colorData ) throws OdometerExceptions {
		
		this.odo = Odometer.getOdometer(); //odometer
		this.leftMotor = leftMotor;//left and right motors
		this.rightMotor = rightMotor;
		this.sampleColor = sampleColor;//sensor related objects
		this.colorData = colorData;
	}
	
	/**
	 * The method localize is the main routine of the class. It settles the robot close from the lines, calls the line sweeping and makes the robot travel to the origin.
	 */
	public void localize() {
		
		//set speed of motors
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		//get the value of the ambient light (when there is no line)
		ambientLight = (float) getData();
		
		//we reset the theta to 0 degrees since we localized with ultrasonic sensor before
		odo.setTheta(0);
		
		setToStart();//settle the robot for good sensoring (to make sure it will see the lines)
		getAngles();//rotates the robot to get the location of axis
		double thetaY = (360-yNeg) + yPos;//angle difference between positive and negative y axis
		double thetaX = xNeg-xPos;//angle difference between positive and negative x axis
		double nowY = calculateDistance(OFFSET, thetaX);//calculate the actual y position of the robot
		double nowX= calculateDistance(OFFSET, thetaY);//calculate the actual x position of the robot
		
		//override x and y positions to travel to origin
		odo.setX(nowX);
		odo.setY(nowY);
		
		//travel to origin
		travelTo(0,0);
		
		//turn to face origin
		turnTo(0);
		
		//the next steps are done to correct the position of the odo, that is falsified because of the sensor offset
		double [] position = odo.getXYT();
		odo.setY(position[1]-OFFSET);
		odo.setX(position[0]-OFFSET);

		
	}
	
	/**
	 * The method travelTo takes the robot from its actual odometer position to a wanted position 
	 * targetx, targety. It calls turnTo to correct its orientation towards the point and then travels 
	 * to the euclidean distance from the two points.
	 * 
	 * @param targetx X final position wanted
	 * @param targety Y final position wanted
	 */
	public void travelTo(double targetx, double targety) {
		
		double x = 30.48* targetx; //coordinates to travelTo
		double y = 30.48* targety;
		double[] xyt = odo.getXYT();//get current odo data
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
		
		turnTo(theta+180);//we turn to theta 
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RADIUS, toTravel), true);//travel to the point
		rightMotor.rotate(convertDistance(WHEEL_RADIUS, toTravel), false);
			 
	}
	
	/**
	 * The method setToStart makes sure that the robot is close enough from the lines
	 * to be able to perform the localization. It basically drives until it sees a line,
	 * turns 90 degrees, and drives again until it sees the other line. 
	 */
	private void setToStart() {

		leftMotor.setSpeed(ROTATE_SPEED);//set motor speeds
		rightMotor.setSpeed(ROTATE_SPEED);
		
		detectLine();//search for a line

	
		rightMotor.rotate(convertDistance(WHEEL_RADIUS, 4), true); //travels to wanted offset( 4 cm)
		leftMotor.rotate(convertDistance(WHEEL_RADIUS, 4 ), false);

		// Rotate the robot by 90 degrees
		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, 90), true);
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, 90), false);

		// checks for another line
		detectLine();
		leftMotor.rotate(convertDistance(WHEEL_RADIUS, 4), true);//travels to wanted offset (4 cm)
		rightMotor.rotate(convertDistance(WHEEL_RADIUS, 4), false);

		leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK, 90), true);//rotates 90 degrees
		rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, 90), false);
	}

	/**
	 * The method getData gets data from the light sensor
	 * @return light sensor value
	 */
	public double getData() {
		sampleColor.fetchSample(colorData, 0);
		double color = colorData[0] * 100.0f; //Scale
		
		return color;
	}
	
	/**
	 * The method detectLine drives the robot and checks if there is any black line on the course
	 */
	private void detectLine() {

		leftMotor.forward();
		rightMotor.forward();

		while (leftMotor.isMoving() && rightMotor.isMoving()) {
			
			//get values from sensor
			sampleColor.fetchSample(colorData, 0);
			float color = colorData[0] * 100;

			//difference between the last value and current value
			float diff = color - lastValue;
			//change the last value to the actual value for next measurement
			lastValue = color;
			if (color < ambientLight -5) {//if the color is darker than supposed
				Sound.beep();//we have a line
				break;
			}

		}
	}
	/**Converts the angle we want to turn to into a distance.
	 * 
	 * @param radius The radius of the wheels
	 * @param width Track of robot
	 * @param angle Angle we want to turn of
	 * @return distance we have to turn each wheel to turn of theta
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	/**Converts the distance to turn of to a rotating distance so it is easier to use with the rotate method of the motors.
	 * 
	 * @param radius Radius of wheels
	 * @param distance distance to travel of
	 * @return distance we have to travel of
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * The getAngles method sweeps for the dark lines in the corner. It then assigns the angle at each line to a variable to calculate the angle between axis
	 */
	public void getAngles() {
		int lines = 0;//number of lines counted
		
		sampleColor.fetchSample(colorData, 0);//get first value
		lastValue = colorData[0] * 100;
		
		leftMotor.setSpeed(ROTATE_SPEED);//setup motor speed
		rightMotor.setSpeed(ROTATE_SPEED);
		
		while(lines<4) {//while we saw less than 4 lines
			
			leftMotor.backward();//we rotate
			rightMotor.forward();
			
			sampleColor.fetchSample(colorData, 0);//get data

			float value = colorData[0] * 100;
			float diff = value - lastValue;//compare to last value recorded
			
			if(value<25) {//
				Sound.beep();//beeping 
				lines++;//incrmeent number of lines counted
				if(lines==1) {
					yNeg= (odo.getXYT())[2];//we are seeing negative y axis
				}
				if(lines==2) {
					xPos= (odo.getXYT())[2];//we are seeing positive x axis
				}
				if(lines==3) {
					yPos= (odo.getXYT())[2];//we are seeing positive y axis
				}
				if(lines==4) {
					xNeg= (odo.getXYT())[2];//we are seeing negative x axis
				}
			}
			lastValue = value;//set last value to current value for next measure
		}
		leftMotor.stop();
		rightMotor.stop();
	}
	
	/**
	 * This method calculates the distance to the x and y axis. 
	 * @param offset Offset of sensor to center of the wheels
	 * @param theta Angle measured between two lines
	 * @return position relative to axis
	 */
	public double calculateDistance(double offset, double theta) {
		double thetaR=Math.toRadians(theta);
		double dist = -(offset)*Math.cos((thetaR)/2);
		return dist;
	}
	
	/**
	 * This method takes the angle of the odometer and the wanted angle, and computes the angle to turn of.
	 * @param theta Wanted angle
	 */
	public void turnTo(double theta) {

		 //method from navigation program
			double minAngle;//minimal angle to turn
			double odoAngle = (odo.getXYT()[2]);//angle from odometer
			
			if ((theta-odoAngle)>180) {//we change the angle if we are not at the minimal angle
				minAngle = theta - odoAngle - 360;
			}
			else if((theta - odoAngle)<-180) {//we change the angle if we are not at the minimal angle
				minAngle = theta - odoAngle;
			}
			
			else {//no changes to be made on angle
				minAngle = theta - odoAngle;
			}
			
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			
			//rotate
			leftMotor.rotate(convertAngle(WHEEL_RADIUS, TRACK , minAngle), true);
			rightMotor.rotate(-convertAngle(WHEEL_RADIUS, TRACK, minAngle), false);
			
		}
}
