package frc.robot;

import static frc.robot.Constants.FuelConstants.TIME_TO_LAUNCH_8;
import static frc.robot.Constants.FuelConstants.TIME_TO_LAUNCH_ALL;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.Targeting;

public class Auto {

        // Choreo initialization
        public final AutoFactory autoFactory;
        public final CANFuelSubsystem ballSubsystem;
        public final CommandSwerveDrivetrain drivetrain;

        private SwerveRequest.FieldCentricFacingAngle targetHubReq = new SwerveRequest.FieldCentricFacingAngle().withHeadingPID(10, 0, 0);

public Command launchSequence(){
        return Commands.parallel(targetHub().withTimeout(.5),ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchPID.atSetpoint())).andThen(ballSubsystem.launchCommand());
}

        public Command targetHub() {
                return drivetrain.applyRequest(() -> targetHubReq
                                .withTargetDirection(Targeting.getTargetRotation(drivetrain.getPose())));
        }

        public String safeOrRiskyPath(){
                return SmartDashboard.getBoolean("Can we win the auto race?", true) ? "_risky" : "_safe";
        }

        public Auto(CommandSwerveDrivetrain drivetrain, CANFuelSubsystem ballSubsystem) {
                // Set the local subsystem to the subsystems passed into the constructor
                // (real instances from RobotContainer)
                this.ballSubsystem = ballSubsystem;
                this.drivetrain = drivetrain;
                // Init Choreo's autoFactory
                autoFactory = new AutoFactory(
                                drivetrain::getPose,
                                drivetrain::resetPose,
                                drivetrain::followTrajectory,
                                true,
                                drivetrain);

                // run intake and spinup on respective event markers (that are added in choreo)
                autoFactory.bind("intake", ballSubsystem.intakeCommand());
                autoFactory.bind("spinup", ballSubsystem.spinUpCommand());
        }

        public AutoRoutine oneCycleLeft() {
                AutoRoutine routine = autoFactory.newRoutine("oneCycleLeft");

                AutoTrajectory cycle1Traj = routine.trajectory("cycle1Left"+safeOrRiskyPath());

                routine.active().onTrue(
                                Commands.sequence(
                                                cycle1Traj.resetOdometry(),
                                                cycle1Traj.cmd()));

                cycle1Traj.done().onTrue(launchSequence());

                return routine;

        }

        public AutoRoutine oneCycleRight() {
                AutoRoutine routine = autoFactory.newRoutine("oneCycleRight");

                AutoTrajectory cycle1Traj = routine.trajectory("cycle1Right"+safeOrRiskyPath());

                routine.active().onTrue(
                                Commands.sequence(
                                                cycle1Traj.resetOdometry(),
                                                cycle1Traj.cmd()));

                cycle1Traj.done().onTrue(launchSequence());

                return routine;

        }
         public AutoRoutine twoCycleLeft() {
                AutoRoutine routine = autoFactory.newRoutine("twoCycleLeft");

                AutoTrajectory cycle1Traj = routine.trajectory("cycle1Left"+safeOrRiskyPath());
                AutoTrajectory cycle2Traj = routine.trajectory("cycle2Left_singleTrench");
                routine.active().onTrue(
                                Commands.sequence(
                                                cycle1Traj.resetOdometry(),
                                                cycle1Traj.cmd()));

                cycle1Traj.done().onTrue(launchSequence().withTimeout(TIME_TO_LAUNCH_ALL).andThen(cycle2Traj.cmd().asProxy()));
                cycle2Traj.done().onTrue(launchSequence());
                return routine;

        }
        public AutoRoutine twoCycleRight() {
                AutoRoutine routine = autoFactory.newRoutine("twoCycleRight");

                AutoTrajectory cycle1Traj = routine.trajectory("cycle1Right"+safeOrRiskyPath());
                AutoTrajectory cycle2Traj = routine.trajectory("cycle2Right_singleTrench");
                routine.active().onTrue(
                                Commands.sequence(
                                                cycle1Traj.resetOdometry(),
                                                cycle1Traj.cmd()));

                cycle1Traj.done().onTrue(launchSequence().withTimeout(TIME_TO_LAUNCH_ALL).andThen(cycle2Traj.cmd().asProxy()));
                cycle2Traj.done().onTrue(launchSequence());
                return routine;

        }


        public AutoRoutine oneCycleCenter() {
                AutoRoutine routine = autoFactory.newRoutine("oneCycleCenter");

                AutoTrajectory cycle1Traj = routine.trajectory("cycle1Center");

                routine.active().onTrue(
                                Commands.sequence(
                                                cycle1Traj.resetOdometry(),
                                                cycle1Traj.cmd()));
                cycle1Traj.done().onTrue(launchSequence().withTimeout(TIME_TO_LAUNCH_8));

                return routine;

        }
        public AutoRoutine twoCycleCenterToLeft() {
                AutoRoutine routine = autoFactory.newRoutine("twoCycleCenterToLeft");

                AutoTrajectory cycle1Traj = routine.trajectory("cycle1Center");
                AutoTrajectory cycle2Traj = routine.trajectory("cycle2CenterToLeft");
                routine.active().onTrue(
                                Commands.sequence(
                                                cycle1Traj.resetOdometry(),
                                                cycle1Traj.cmd()));
                cycle1Traj.done().onTrue(launchSequence().withTimeout(TIME_TO_LAUNCH_8 + SmartDashboard.getNumber("Seconds to wait after center shoot before intaking", 0)).andThen(cycle2Traj.cmd().asProxy()));
                
                cycle2Traj.done().onTrue(launchSequence());

                return routine;

        }
        public AutoRoutine twoCycleCenterToRight() {
                AutoRoutine routine = autoFactory.newRoutine("twoCycleCenterToRight");

                AutoTrajectory cycle1Traj = routine.trajectory("cycle1Center");
                AutoTrajectory cycle2Traj = routine.trajectory("cycle2CenterToRight");
                routine.active().onTrue(
                                Commands.sequence(
                                                cycle1Traj.resetOdometry(),
                                                cycle1Traj.cmd()));
                cycle1Traj.done().onTrue(launchSequence().withTimeout(TIME_TO_LAUNCH_8 + SmartDashboard.getNumber("Seconds to wait after center shoot before intaking", 0)).andThen(cycle2Traj.cmd().asProxy()));
                
                cycle2Traj.done().onTrue(launchSequence());

                return routine;

        }



}
