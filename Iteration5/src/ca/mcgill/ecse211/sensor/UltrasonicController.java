package ca.mcgill.ecse211.sensor;

public interface UltrasonicController {

  public void processUSData(int distance);

  public int readUSDistance();
}
