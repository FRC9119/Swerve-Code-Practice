package frc.robot;

import static frc.robot.Constants.FuelConstants.TIME_TO_LAUNCH_8;
import static frc.robot.Constants.FuelConstants.TIME_TO_LAUNCH_ALL;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Auto {

        // Choreo initialization
        public final AutoFactory autoFactory;
        public final CANFuelSubsystem ballSubsystem;

        public Auto(CommandSwerveDrivetrain drivetrain, CANFuelSubsystem ballSubsystem) {
                // Set the local subsystem to the subsystems passed into the constructor
                // (real instances from RobotContainer)
                this.ballSubsystem = ballSubsystem;
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
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterRightIntake");

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
                                .andThen(ballSubsystem.launchCommand().withTimeout(5)));

                return routine;

        }

        public AutoRoutine leftTwoCycle() {
                AutoRoutine routine = autoFactory.newRoutine("leftTwoCycle");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromLeft");
                AutoTrajectory scoreTraj1 = routine.trajectory("scoreAfterLeftIntake");
                AutoTrajectory sweepAndScoreTraj = routine.trajectory("sweepThenScoreFromLeftScore");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(scoreTraj1.cmd());

                scoreTraj1.done().onTrue(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(sweepAndScoreTraj.cmd()));

                sweepAndScoreTraj.done()
                                .onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()));

                return routine;
        }

        public AutoRoutine rightTwoCycle() {
                AutoRoutine routine = autoFactory.newRoutine("rightTwoCycle");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromRight");
                AutoTrajectory scoreTraj1 = routine.trajectory("scoreAfterRightIntake");
                AutoTrajectory outpostTraj = routine.trajectory("outpostAfterRightScore");
                AutoTrajectory scoreTraj2 = routine.trajectory("scoreAfterOutpost");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(scoreTraj1.cmd());

                scoreTraj1.done().onTrue(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(outpostTraj.cmd()));
                outpostTraj.done().onTrue(Commands.waitSeconds(2).andThen(scoreTraj2.cmd()));
                scoreTraj2.done()
                                .onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()));

                return routine;
        }

        public AutoRoutine rightThreeCycle() {
                AutoRoutine routine = autoFactory.newRoutine("rightTwoCycle");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromRight");
                AutoTrajectory scoreTraj1 = routine.trajectory("scoreAfterRightIntake");
                AutoTrajectory outpostTraj = routine.trajectory("outpostAfterRightScore");
                AutoTrajectory scoreTraj2 = routine.trajectory("scoreAfterOutpost");
                AutoTrajectory intakeTraj2 = routine.trajectory("intakeStraightFromRightScore");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(scoreTraj1.cmd());

                scoreTraj1.done().onTrue(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(outpostTraj.cmd()));
                outpostTraj.done().onTrue(Commands.waitSeconds(2).andThen(scoreTraj2.cmd()));
                scoreTraj2.done()
                                .onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_ALL)
                                                                .andThen(intakeTraj2.cmd())));
                intakeTraj2.done()
                                .onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()));

                return routine;
        }

}
