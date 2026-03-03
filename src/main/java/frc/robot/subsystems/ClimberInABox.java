package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimberInABox extends SubsystemBase {
    private final TalonFX kraken;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController alignX = new PIDController(ALIGN_XY_KP, 0, 0);
    private final PIDController alignY = new PIDController(ALIGN_XY_KP, 0, 0);

    public ClimberInABox(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        kraken = new TalonFX(CLIMB_MOTOR_ID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = MAX_CLIMB_CURRENT;
        kraken.getConfigurator().apply(config);
        kraken.setNeutralMode(NeutralModeValue.Brake);
    }

    public Command align(Alignment a) {
        return this.run(() -> {
            Pose2d goalPosition;
            Pose2d currentPosition = drivetrain.getState().Pose;
            // TODO: figure out what these poses are
            switch (a) {
                case Left:
                    goalPosition = new Pose2d();
                case Right:
                    goalPosition = new Pose2d();
                default:
                    goalPosition = new Pose2d();
            }
            // Calculate necessary velocities to get to goal position with PID
            double xVelocity = alignX.calculate(currentPosition.getX(), goalPosition.getX());
            double yVelocity = alignY.calculate(currentPosition.getY(), goalPosition.getY());
            // Start and configure swerve request
            SwerveRequest.FieldCentricFacingAngle alignRequest = new SwerveRequest.FieldCentricFacingAngle()
                    .withHeadingPID(ROTATION_KP, 0, 0).withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                    // Give request goal rotation, and calculated velocities
                    .withTargetDirection(goalPosition.getRotation()).withVelocityX(xVelocity).withVelocityY(yVelocity);
            // Apply request to drivetrain
            drivetrain.applyRequest(() -> alignRequest);

        });

    }

    public enum Alignment {
        Left, Right
    }

    public void climb() {
        kraken.set(1);
    }

    public void release() {
        kraken.set(-.5);
    }

    public void stop() {
        kraken.set(0);
    }
}
