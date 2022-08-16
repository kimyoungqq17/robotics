package ca.mcgill.ecse211.localizer;

import ca.mcgill.ecse211.lab5.lab5;
import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class LightLocalizer {
	private static Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;
	
	private static final double LS_LENGTH = 15;			//needs to be tested //distance from the center of the robot to the light sensor
	private static final double TILE_LENGTH = 30.48;
	
	
	private static final double TRACK = 12.92;				//track of the robot
	private static final double WHEEL_RAD = 2.1;		//wheel radius
	
	double last_color;									//last color the light  sensor detected
	
	/**
	 * 
	 * construction of the class
	 * 
	 * @param odo This is the odometer
	 * @param colorSensor This is the light sensor
	 * @param colorData This is the light sensor data
	 */
	public LightLocalizer (Odometer odo, SampleProvider colorSensor, float[] colorData) {
		LightLocalizer.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}
	
	/**
	 * This method makes the robot to localize using color sensor
	 * @throws InterruptedException
	 */
	public void Localize() throws InterruptedException {
		lab5.leftmotor.setSpeed(100);
		lab5.rightmotor.setSpeed(100);

		int countline = 0;					//count how many line are detected
		double angles[] = new double[4];	//angle for each line
		
		
		double floor_color = getLightData();
		while(Math.abs(floor_color - getLightData()) < 100) {	 //need to be tested				//keep going until there is a black line
			
			lab5.leftmotor.backward();
			lab5.rightmotor.backward();
		}
		Sound.beep();
		lab5.leftmotor.rotate(-300,true);				//move forward for some extra distance
		lab5.rightmotor.rotate(-300);					//stop

		while (countline < 4) {							//needs to detect four lines
			lab5.leftmotor.backward();
			lab5.rightmotor.forward();
							//robot spin clockwise
			if(Math.abs(floor_color - getLightData()) >= 100) {			//if black line detected
				double detect_theta = odo.getXYT()[2];	//get the current angle
				angles[countline] = detect_theta;		//record the angle
				countline++;							//update angle count
				Sound.beep();							//beep
				Thread.sleep(1000);						//sleep for 1 sec, to make sure that the same line will not be detected twice
			}else {
				Thread.sleep(50);	
			}
		}
		
		lab5.leftmotor.stop(true);			
		lab5.rightmotor.stop();			
		
		double angle_x, angle_y;
		double position_x, position_y;
		
		angle_y = Math.abs(angles[0]-angles[2]);			//calculate angle for y correction
		angle_x = Math.abs(angles[1]-angles[3]);			//calculate angle for x correction
		
		position_x = (-1)*LS_LENGTH*Math.cos(Math.toRadians(angle_x)/2);    //calculate correct x position
		position_y = (-1)*LS_LENGTH*Math.cos(Math.toRadians(angle_y)/2);	//calculate correct y position
		
		odo.setX(position_x+TILE_LENGTH);							//update x
		odo.setY(position_y+TILE_LENGTH);							//update y
	}
	
	
	
	/** 
	 * Takes the new heading as input and make the robot turn to it
	 * 
	 * @param: double theta that represents an angle in radians
	 */
		public static void turnTo(double theta) {
		
			double angle = getMinAngle(theta-odo.getXYT()[2]);
		
			lab5.leftmotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle),true);
			lab5.rightmotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle),false);
		}

		/**
		 * Gets the smallest value (between 180 and -180) of an angle
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
	 * 
	 * This method takes the radius of wheels and the distance that needs to be traveled, and calculate 
	 * the rotation of the wheels need to make to travel that distance
	 * 
	 * @param radius is the wheel radius
	 * @param distance is the distance needs to be traveled
	 * @return rotation of the wheels
	 */
	private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method take the radius of wheels, width of the robot and the angle needs to be rotated by the robot, 
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
	
	/**
	 * This method is used to get data from light sensor
	 * 
	 * @return color data
	 */
	public int getLightData() {
		colorSensor.fetchSample(colorData,0);
		int color = (int)(colorData[0] * 1000);
		
		return color;
	}
	
}

