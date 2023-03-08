package frc.robot.subsystems.arm;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ArmTrajectory {
  private ArrayList<ArmPose> m_waypoints; // List of waypoints for the trajectory
  private double m_maxTranslationalSpeed; // Inches per second
  private double m_totalTime; // Seconds
  private double m_totalLength; // Inches
  private ArrayList<Double> m_waypointDists; // Cumulative distance from the beginning to each waypoint
  private ArrayList<Double> m_waypointTimes; // Cumulative time from the beginning to each waypoint

  /**
   * 
   * @param waypoints ArrayList of ArmPose waypoints
   */
  public ArmTrajectory(ArrayList<ArmPose> waypoints) {
    m_waypoints = new ArrayList<ArmPose>(waypoints);

    // Calculate characteristics of the trajectory 
    m_maxTranslationalSpeed = Constants.Arm.k_maxTrajectorySpeed;
    m_totalLength = calcTotalDistance(m_waypoints);
    m_totalTime = m_totalLength / m_maxTranslationalSpeed;

    // Calculate the cumulative distance and time for each of the waypoints
    m_waypointDists = new ArrayList<Double>();
    m_waypointTimes = new ArrayList<Double>();

    m_waypointDists.add(0.0);
    m_waypointTimes.add(0.0);

    for(int i = 1; i < m_waypoints.size(); i++) {
      double deltaDist = dist(m_waypoints.get(i-1), m_waypoints.get(i));
      double deltaTime = deltaDist / m_maxTranslationalSpeed;
      m_waypointDists.add(m_waypointDists.get(i-1) + deltaDist);
      m_waypointTimes.add(m_waypointTimes.get(i-1) + deltaTime);
    }

    SmartDashboard.putString("Waypoints", m_waypoints.toString());
    SmartDashboard.putString("WaypointDistances", m_waypointDists.toString());
    SmartDashboard.putString("WaypointTimes", m_waypointTimes.toString());
    SmartDashboard.putNumber("TotalTime", m_totalTime);
  }

  /**
   * By intepolating between the given waypoints
   * 
   * @param time Time in seconds since the beginning of the trajectory
   * @return ArmPose (x inches, y inches, wrist rotation degrees)
   */
  public ArmPose sample(double time) {
    if(time < 0.0) {
      return m_waypoints.get(0);
    }
    if(time > m_totalTime) {
      return m_waypoints.get(m_waypoints.size() - 1);
    }

    // At any point in time, the arm will be between two waypoints
    // Find which waypoints it is between
    int waypoint0 = 0;
    int waypoint1 = 0;
    for(int i = 1; i < m_waypoints.size(); i++) {
      if((m_waypointTimes.get(i-1) <= time) && 
         (m_waypointTimes.get(i) >= time)) {
          waypoint0 = i - 1;
          waypoint1 = i;
          break;
      }
    }

    // Calculate the total x and y translation between the two waypoints, and how long it should take
    double changeX = m_waypoints.get(waypoint1).getX() - m_waypoints.get(waypoint0).getX();
    double changeY = m_waypoints.get(waypoint1).getY() - m_waypoints.get(waypoint0).getY();
    double changeTime = m_waypointTimes.get(waypoint1) - m_waypointTimes.get(waypoint0);

    // Calculate the amount of time since waypoint0
    double deltaTime = time - m_waypointTimes.get(waypoint0);

    double deltaX = changeX * (deltaTime / changeTime);
    double deltaY = changeY * (deltaTime / changeTime);

    ArmPose targetPose = new ArmPose(
        m_waypoints.get(waypoint0).getX() + deltaX,
        m_waypoints.get(waypoint0).getY() + deltaY,
        new Rotation2d(0));

    return targetPose;
  }

  private double calcTotalDistance(ArrayList<ArmPose> waypoints) {
    double totalLength = 0.0;
    for(int i = 0; i < waypoints.size() - 1; i++)
    {
      totalLength += dist(waypoints.get(i), waypoints.get(i+1));
    }
    return totalLength;
  }
  
  private double dist(ArmPose pose0, ArmPose pose1) {
    return Math.sqrt(Math.pow((pose1.getX() - pose0.getX()), 2.0) + 
    Math.pow((pose1.getY() - pose0.getY()), 2.0));
  }

  public double getTotalLength() {
    return m_totalLength;
  }
  
  public double getTotalTime() {
    return m_totalTime;
  }
}
