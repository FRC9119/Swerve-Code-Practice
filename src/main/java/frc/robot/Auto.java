package frc.robot;

import static frc.robot.Constants.FuelConstants.TIME_TO_LAUNCH_8;
import static frc.robot.Constants.FuelConstants.TIME_TO_LAUNCH_ALL;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
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

        private SwerveRequest.FieldCentricFacingAngle targetHubReq = new SwerveRequest.FieldCentricFacingAngle();
       public Command targetHub (){
        return drivetrain.applyRequest(() -> targetHubReq.withTargetDirection(Targeting.getTargetRotation(drivetrain.getPose())));
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

        public AutoRoutine leftIntakeShoot() {
                AutoRoutine routine = autoFactory.newRoutine("leftIntakeShoot");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromLeft");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterLeftIntake");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(scoreTraj.cmd());

                scoreTraj.done().onTrue(ballSubsystem.launchCommand());

                return routine;

        }

        public AutoRoutine rightIntakeShoot() {
                AutoRoutine routine = autoFactory.newRoutine("rightIntakeShoot");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromRight");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterRightIntake_trench");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(scoreTraj.cmd());

                scoreTraj.done().onTrue(ballSubsystem.launchCommand());

                return routine;

        }

        public AutoRoutine centerShoot() {
                AutoRoutine routine = autoFactory.newRoutine("centerShoot");

                AutoTrajectory scoreTraj = routine.trajectory("scoreFromCenter");

                routine.active().onTrue(
                                Commands.sequence(
                                                scoreTraj.resetOdometry(),
                                                scoreTraj.cmd()));
                scoreTraj.done().onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                .andThen(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_8)));

                return routine;

        }
        
        public AutoRoutine centerShootSweepLeft (double seconds) {
                AutoRoutine routine = autoFactory.newRoutine("centerShootSweepLeft");

                AutoTrajectory scoreTraj = routine.trajectory("scoreFromCenter");
                AutoTrajectory sweepTraj = routine.trajectory("intakeLeftFromCenterScore");

                routine.active().onTrue(
                                Commands.sequence(
                                                scoreTraj.resetOdometry(),
                                                scoreTraj.cmd()));
                scoreTraj.done().onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                .andThen(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_8)).andThen(Commands.waitSeconds(seconds).andThen(sweepTraj.cmd())));

                return routine;
        }
        public AutoRoutine centerShootSweepRight (double seconds) {
                AutoRoutine routine = autoFactory.newRoutine("centerShootSweepRight");

                AutoTrajectory scoreTraj = routine.trajectory("scoreFromCenter");
                AutoTrajectory sweepTraj = routine.trajectory("intakeRightFromCenterScore");

                routine.active().onTrue(
                                Commands.sequence(
                                                scoreTraj.resetOdometry(),
                                                scoreTraj.cmd()));
                scoreTraj.done().onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                .andThen(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_8)).andThen(Commands.waitSeconds(seconds).andThen(sweepTraj.cmd())));

                return routine;
        }

        public AutoRoutine leftTwoCycleFullField() {
                AutoRoutine routine = autoFactory.newRoutine("leftTwoCycleFullField");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromLeft");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterLeftIntake");
                AutoTrajectory sweepAndScoreTraj = routine.trajectory("sweepScoreFromLeftScore");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(scoreTraj.cmd());

                scoreTraj.done().onTrue(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(sweepAndScoreTraj.cmd()));

                sweepAndScoreTraj.done()
                                .onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()));

                return routine;
        }

        
        public AutoRoutine leftTwoCycleBump() {
                AutoRoutine routine = autoFactory.newRoutine("leftTwoCycleBump");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromLeft");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterLeftIntake");
                AutoTrajectory sweepAndScoreTraj = routine.trajectory("sweepScoreFromLeftScore_bump");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(scoreTraj.cmd());

                scoreTraj.done().onTrue(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(sweepAndScoreTraj.cmd()));

                sweepAndScoreTraj.done()
                                .onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()));

                return routine;
        }
        
        public AutoRoutine rightTwoCycleBump() {
                AutoRoutine routine = autoFactory.newRoutine("rightTwoCycleBump");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromRight");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterRighttIntake");
                AutoTrajectory sweepAndScoreTraj = routine.trajectory("sweepScoreFromRightScore_bump");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(scoreTraj.cmd());

                scoreTraj.done().onTrue(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(sweepAndScoreTraj.cmd()));

                sweepAndScoreTraj.done()
                                .onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()));

                return routine;
        }

        public AutoRoutine rightTwoCycleFullField() {
                AutoRoutine routine = autoFactory.newRoutine("rightTwoCycleFullField");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromRight");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterRightIntake");
                AutoTrajectory sweepTraj = routine.trajectory("sweepScoreFromRightScore");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(scoreTraj.cmd());

                scoreTraj.done().onTrue(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(sweepTraj.cmd()));
                sweepTraj.done()
                                .onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()));

                return routine;
        }


}
