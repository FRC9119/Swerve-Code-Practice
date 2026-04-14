package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class Vision extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    private final NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    private final StructPublisher<Pose2d> shootCamPosePub = visionTable.getStructTopic("shootCamPose", Pose2d.struct)
            .publish();
    private final StructPublisher<Pose2d> intakeCamPosePub = visionTable.getStructTopic("intakeCamPose", Pose2d.struct)
            .publish();

    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        boolean rotatingTooFast = Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) > 10;
        if (!rotatingTooFast) {
            Pigeon2 gyro = drivetrain.getPigeon2();
            double yaw = gyro.getYaw().getValueAsDouble();
            double pitch = gyro.getPitch().getValueAsDouble();
            double roll = gyro.getRoll().getValueAsDouble();

            LimelightHelpers.SetRobotOrientation("limelight-shoot",
                    yaw, 0.0, pitch, 0.0, roll, 0.0);
            LimelightHelpers.SetRobotOrientation("limelight-intake",
                    yaw, 0.0, pitch, 0.0, roll, 0.0);

            LimelightHelpers.PoseEstimate intakeCamEstimate = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-intake");
            LimelightHelpers.PoseEstimate shootCamEstimate = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-shoot");

            if (shootCamEstimate.tagCount != 0 && SmartDashboard.getBoolean("Use Shooter Limelight", true)) {
                drivetrain.addVisionMeasurement(useGyroRotation(shootCamEstimate.pose), Timer.getFPGATimestamp());

                shootCamPosePub.set(shootCamEstimate.pose);
                SignalLogger.writeStruct("Vision/shootCamPose", Pose2d.struct, shootCamEstimate.pose);
            }

            if (intakeCamEstimate.tagCount != 0 && SmartDashboard.getBoolean("Use Intake Limelight", true)) {
                System.out.println(intakeCamEstimate.pose);
                drivetrain.addVisionMeasurement(useGyroRotation(intakeCamEstimate.pose), Timer.getFPGATimestamp());

                intakeCamPosePub.set(intakeCamEstimate.pose);
                SignalLogger.writeStruct("Vision/intakeCamPose", Pose2d.struct, intakeCamEstimate.pose);
            }

        }
    }

    public Pose2d useGyroRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), drivetrain.getPigeon2().getRotation2d());
    }
}
