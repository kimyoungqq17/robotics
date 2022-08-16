package ca.mcgill.ecse211.lab1;



import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  private static final int FILTER_OUT = 20;
  private int filterControl;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    this.filterControl=0;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
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
	    
	    WallFollowingLab.leftMotor.forward();
	    WallFollowingLab.rightMotor.forward();
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    
    if (this.distance > bandCenter - bandwidth && this.distance < bandCenter + bandwidth) {//checking if the robot is between the error range
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);//if this is the case, we don't do anything. The robots go forward.
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
	    WallFollowingLab.rightMotor.forward();
    }
    if (this.distance < (bandCenter - bandwidth)) {//if the robot is too close from the wall
    	WallFollowingLab.rightMotor.setSpeed(motorHigh+30);//increase speed of right wheel
    	WallFollowingLab.leftMotor.setSpeed(motorLow);//decrease speed of left wheel
    	WallFollowingLab.rightMotor.backward();//we make the right wheel go backwards so the robot can pivot
    	WallFollowingLab.leftMotor.forward();
    }
    
    if (this.distance > (bandCenter + bandwidth)) {// if the robot is too far from the wall
  	  WallFollowingLab.rightMotor.setSpeed(motorHigh);//right wheel speed doesn't change
  	  WallFollowingLab.leftMotor.setSpeed(motorLow);//we decrease the speed of the left motor so the robot turns left
  	  WallFollowingLab.rightMotor.forward();
  	  WallFollowingLab.leftMotor.forward();
  	  
    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
