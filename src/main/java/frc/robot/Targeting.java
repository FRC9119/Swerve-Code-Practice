package frc.robot;

import static frc.robot.Constants.FuelConstants.DEFAULT_LAUNCH_RPM;
import static frc.robot.Constants.FuelConstants.FULL_FIELD_X;
import static frc.robot.Constants.FuelConstants.FULL_FIELD_Y;
import static frc.robot.Constants.FuelConstants.HUB_X_COORD;
import static frc.robot.Constants.FuelConstants.HUB_Y_COORD;
import static frc.robot.Constants.FuelConstants.USE_SHOOTER_LIMELIGHT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Targeting {

    public static double getRadiansBetweenRobotAndHub(Pose2d pose) {
                Translation2d bluePose = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                                ? (new Translation2d(FULL_FIELD_X, FULL_FIELD_Y))
                                                .minus(pose.getTranslation())
                                : pose.getTranslation();
                return Math.atan2(bluePose.getY() - HUB_Y_COORD, bluePose.getX() - HUB_X_COORD);
        }
        
  public static double getTargetRPM(Pose2d pose) {
    if (!USE_SHOOTER_LIMELIGHT)
      return DEFAULT_LAUNCH_RPM;

    Translation2d coordinates = pose.getTranslation();
    Translation2d blueCoordinates = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        ? (new Translation2d(FULL_FIELD_X, FULL_FIELD_Y)).minus(coordinates)
        : coordinates;
    double distance = blueCoordinates.getDistance(new Translation2d(HUB_X_COORD, HUB_Y_COORD));
   // equation from spreadsheet measurements
    return 723.75*distance+2081;
  }
}
