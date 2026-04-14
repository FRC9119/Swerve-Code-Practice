// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.DriveConstants.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.Dashboard;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Targeting;
import frc.robot.utils.DriveTelemetry;

public class RobotContainer {
        // kSpeedAt12Volts desired top speed
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // 3/4 of a rotation per second max angular velocity
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.Velocity);
        // Request to drive and face hub
        public final SwerveRequest.FieldCentricFacingAngle targetHub = new SwerveRequest.FieldCentricFacingAngle()
                        .withHeadingPID(10, 0, 0)
                        // Add a 10% deadband
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.Velocity)
                        // Flip perspective (angle and velocity) when on red
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        public final SwerveRequest.Idle idle = new SwerveRequest.Idle();
        public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        // Init logging
        public final DriveTelemetry driveLogger = new DriveTelemetry(MaxSpeed);
        // Driver Controller
        private CommandPS5Controller driverController = new CommandPS5Controller(0);
        // Operator Controller
        private final CommandXboxController operatorController = new CommandXboxController(1);
        // Init Subsystems
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem(drivetrain);
        // Init auto
        public final Auto auto = new Auto(drivetrain, ballSubsystem);

        // Init dashboard, which sends all options to SmartDashboard/Elastic
        public final Dashboard dashboard = new Dashboard(auto, ballSubsystem, drivetrain);

        public RobotContainer() {
                // Make limelight webpage available on roboRIO IP
                LimelightHelpers.setupPortForwardingUSB(0);
                dashboard.sysIdRoutineChooser.onChange((newRoutine) -> addSysIdBindings(newRoutine));
                addDriverBindings();
                addOperatorBindings();
                addSysIdBindings(dashboard.sysIdRoutineChooser.getSelected());

                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // give logs to drivetrain
                drivetrain.registerTelemetry(driveLogger::telemeterizeDriveState);
        }

        private void addOperatorBindings() {
                /*
                 * While Y is held AND when constant USE_SHOOTER_LIMELIGHT is true, target hub.
                 * Otherwise run default drive SwerveRequest
                 */
                operatorController.y().whileTrue(ballSubsystem.spinUpCommand()
                                .until(() -> ballSubsystem.launchPID.atSetpoint())
                                .andThen(ballSubsystem.launchCommand()).finallyDo(() -> ballSubsystem.stop()))

                                .whileTrue(drivetrain.applyRequest(() -> {
                                        if (SmartDashboard.getBoolean("Use Targeting", true)) {
                                                if (driverController.cross().getAsBoolean())
                                                        return brake;
                                                return targetHub.withTargetDirection(
                                                                Targeting.getTargetRotation(drivetrain.getPose()))
                                                                .withVelocityX(-Math.atan(
                                                                                driverController.getRawAxis(1)
                                                                                                * MaxSpeed
                                                                                                * SPEED_SCALAR_WHILE_TARGETING))
                                                                .withVelocityY(-Math.atan(
                                                                                driverController.getRawAxis(0)
                                                                                                * MaxSpeed
                                                                                                * SPEED_SCALAR_WHILE_TARGETING));
                                        } else
                                                return drive.withVelocityX(
                                                                -Math.atan(driverController.getRawAxis(1))
                                                                                * SPEED_SCALAR)
                                                                .withVelocityY(-Math
                                                                                .atan(driverController.getRawAxis(0))
                                                                                * SPEED_SCALAR)
                                                                .withRotationalRate(-driverController.getRawAxis(2)
                                                                                * MaxAngularRate);

                                }));
                // Run outtake (called eject()) periodically while B is pressed
                operatorController.b()
                                .whileTrue(ballSubsystem.ejectCommand());
                // Run intake() periodically while X is pressed
                operatorController.x()
                                .whileTrue(ballSubsystem.intakeCommand());
                // Run unclog() periodically while X is pressed
                operatorController.a()
                                .whileTrue(ballSubsystem.unclogCommand());
                operatorController.leftTrigger()
                                .whileTrue(ballSubsystem.launchWithoutTargeting(55));
                operatorController.rightTrigger()
                                .whileTrue(ballSubsystem.launchWithoutTargeting(operatorController.getRightTriggerAxis() * 30 + 55));
                // Start and end SysId logging
                operatorController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
                operatorController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
        }

        private void addDriverBindings() {

                double speedScalar = MaxSpeed * (SPEED_SCALAR + driverController.getR2Axis() * (1 - SPEED_SCALAR));
                // Add joystick controll to swerve request
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-applyInputShaping(driverController.getRawAxis(1))
                                                                * speedScalar)
                                                .withVelocityY(-applyInputShaping(driverController.getRawAxis(0))
                                                                * speedScalar)
                                                .withRotationalRate(-applyInputShaping(driverController.getRawAxis(2))
                                                                * MaxAngularRate)));

                // reset the field-centric heading on left bumper press
                driverController.L1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // zero gyro yaw on right bumper press
                driverController.R1().onTrue(drivetrain.runOnce(() ->
                // get alliance and make sure it exists
                DriverStation.getAlliance().ifPresent((alliance) ->
                // set gyro to 0 if blue, 180 if red
                drivetrain.getPigeon2().setYaw(alliance == Alliance.Blue ? 0 : 180))));
        }

        private void addSysIdBindings(SysIdRoutine routine) {
                // Run SysId routines using d-pad buttons while holding circle
                // Note that each routine should be run exactly once in a single log. (ONLY FOR
                // TESTING)
                driverController = new CommandPS5Controller(0);
                addDriverBindings();
                driverController.povUp().whileTrue(routine.dynamic(Direction.kForward));
                driverController.povDown().whileTrue(routine.dynamic(Direction.kReverse));
                driverController.povLeft().whileTrue(routine.quasistatic(Direction.kForward));
                driverController.povRight().whileTrue(routine.quasistatic(Direction.kReverse));

        }

        public double applyInputShaping(double joystickAxis) {
                double deadband = .05;
                double exponent = 2;
                if (Math.abs(joystickAxis) < deadband)
                        return 0;
                return Math.signum(joystickAxis)
                                * Math.pow((Math.abs(joystickAxis) - deadband) / (1 - deadband), exponent);
        }

}
