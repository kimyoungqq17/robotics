package ca.mcgill.ecse211.ultrasonicpoll;


public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
