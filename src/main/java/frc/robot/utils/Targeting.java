package frc.robot.utils;

import static frc.robot.Constants.FuelConstants.DEFAULT_LAUNCH_RPS;
import static frc.robot.Constants.FuelConstants.FULL_FIELD_X;
import static frc.robot.Constants.FuelConstants.FULL_FIELD_Y;
import static frc.robot.Constants.FuelConstants.HUB_X_COORD;
import static frc.robot.Constants.FuelConstants.HUB_Y_COORD;
import static frc.robot.Constants.FuelConstants.USE_SHOOTER_LIMELIGHT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// This is just a class that has helpful methods for targeting the hub.
// There is no need to instantiate this class because everything is static (meaning it never changes based on variables in the class)
public class Targeting {

  public static Rotation2d getTargetRotation(Pose2d pose) {
    // Change pose as if you were on blue alliance, because we use the coordinates
    // of the blue hub
    Translation2d bluePose = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        ? (new Translation2d(FULL_FIELD_X, FULL_FIELD_Y))
            .minus(pose.getTranslation())
        : pose.getTranslation();
    // Calculate angle by giving Rotation2d an x and y
    return new Rotation2d(bluePose.getX() - HUB_X_COORD, bluePose.getY() - HUB_Y_COORD);
  }

  public static double getTargetRPS(Pose2d pose) {
    if (!USE_SHOOTER_LIMELIGHT)
      return DEFAULT_LAUNCH_RPS;

    Translation2d coordinates = pose.getTranslation();
    Translation2d blueCoordinates = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        ? (new Translation2d(FULL_FIELD_X, FULL_FIELD_Y)).minus(coordinates)
        : coordinates;
    double distance = blueCoordinates.getDistance(new Translation2d(HUB_X_COORD, HUB_Y_COORD));
      
    //return SmartDashboard.getNumber("Default Launch RPS",0);
    // equation from spreadsheet measurements
    return (SmartDashboard.getNumber("slope", 9.9) * distance + SmartDashboard.getNumber("intercept", 30.78) );
  }

}
