package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 100;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
  //private int rectConstant = 12;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
 // TODO: process a movement based on the us distance passed in (P style)

    int error=this.distance-this.bandCenter; //changed distance to this.distance for clarification
   
    if(Math.abs(error)<=bandWidth) { // going straight
    	
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);//we just keep the motors as they are, initial state
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
    }
    else if(error<0) { //too close to the wall
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + (bandCenter - Math.abs(distance)) * 11.0f);//increase the distance of the left wheel so the robot turns right
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);//keep the right wheel as it is, but it will go backwards
    	WallFollowingLab.leftMotor.forward();//left wheel forward
 		WallFollowingLab.rightMotor.backward();//right wheel backwards
    	
    }
    else if(error>0) {//if the robot is too far from the wall

        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
        if(distance>30) {
        	 WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+100);
        }
        else {
        WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+distance*1.5f);//increase the speed of the right wheel so the robot turns left
        }
        WallFollowingLab.leftMotor.forward();//both wheels go forward
 		WallFollowingLab.rightMotor.forward();
    }
    
  }
	  
  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
