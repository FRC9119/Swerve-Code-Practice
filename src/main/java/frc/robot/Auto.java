package frc.robot;

import static frc.robot.Constants.ClimbConstants.CLIMB_CYCLE_TIME;
import static frc.robot.Constants.FuelConstants.TIME_TO_LAUNCH_8;
import static frc.robot.Constants.FuelConstants.TIME_TO_LAUNCH_ALL;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberInABox;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Auto {

        // Choreo initialization
        public final AutoFactory autoFactory;
        public final CANFuelSubsystem ballSubsystem;
        public final ClimberInABox climbSubsystem;

        public Auto(CommandSwerveDrivetrain drivetrain, CANFuelSubsystem ballSubsystem, ClimberInABox climbSubsystem) {
                // Set the local subsystem to the subsystems passed into the constructor
                // (real instances from RobotContainer)
                this.ballSubsystem = ballSubsystem;
                this.climbSubsystem = climbSubsystem;
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

        // Use this auto as a reference
        // This function defines a routine and then returns it
        // It's a generator
        public AutoRoutine leftIntakeShootClimb() {
                // Initialize the routine with name
                AutoRoutine routine = autoFactory.newRoutine("leftIntakeShootClimb");

                // Load the routine's trajectories (all of them)
                AutoTrajectory intakeTraj = routine.trajectory("intakeFromLeft");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterLeftIntake");
                AutoTrajectory climbTraj = routine.trajectory("climbAfterLeftScore");

                // When the routine begins, reset odometry and start the first trajectory
                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                // When the trajectory is done, start the next trajectory and at the same time, raise the climb arm up
                intakeTraj.done().onTrue(Commands.parallel(scoreTraj.cmd(), climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME)));

                // When the trajectory is done, score
                scoreTraj.done().onTrue(ballSubsystem.launchCommand()
                                // After TIME_TO_LAUNCH_8 seconds, stop scoring and start the climb trajectory
                                .withTimeout(TIME_TO_LAUNCH_8).andThen(climbTraj.cmd()));
                // When we arrive at the climbing position, climb
                climbTraj.done().onTrue(climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME));

                // Return the finished routine
                return routine;

        }

        public AutoRoutine rightIntakeShootClimb() {
                AutoRoutine routine = autoFactory.newRoutine("rightIntakeShootClimb");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromRight");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterRightIntake");
                AutoTrajectory climbTraj = routine.trajectory("climbAfterRightScore");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(Commands.parallel(scoreTraj.cmd(), climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME)));

                scoreTraj.done().onTrue(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_8).andThen(climbTraj.cmd()));
                climbTraj.done().onTrue(climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME));

                return routine;

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

        // Use this as another complex example
        public AutoRoutine centerShootClimbLeft() {
                // create routine and name it
                AutoRoutine routine = autoFactory.newRoutine("centerShootClimbLeft");

                // load all trajectories
                AutoTrajectory scoreTraj = routine.trajectory("scoreFromCenter");
                AutoTrajectory climbTraj = routine.trajectory("climbLeftFromCenter");
                // on start, set robot position, start trajectory, and get the shooter up to
                // speed
                routine.active().onTrue(
                                Commands.sequence(
                                                scoreTraj.resetOdometry(),
                                                scoreTraj.cmd()));
                // after first trajectory
                scoreTraj.done().onTrue(
                        // put climb arm up
                                Commands.parallel( climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME),
                                // at the same time, stay still until launcher is up to speed
                                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                // then launch for three seconds
                                                .andThen(ballSubsystem.launchCommand()
                                                                .withTimeout(TIME_TO_LAUNCH_8)
                                                                // start next trajectory afterwards
                                                                .andThen(climbTraj.cmd()))));
                // once in position, climb for the amount of time specified in Constants.java
                climbTraj.done().onTrue(climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME));

                return routine;
        }

        public AutoRoutine centerShootClimbRight() {
                AutoRoutine routine = autoFactory.newRoutine("centerShootClimbRight");

                AutoTrajectory scoreTraj = routine.trajectory("scoreFromCenter");
                AutoTrajectory climbTraj = routine.trajectory("climbRightFromCenter");
                routine.active().onTrue(
                                Commands.sequence(
                                                scoreTraj.resetOdometry(),
                                                scoreTraj.cmd()));
                scoreTraj.done().onTrue(
                                Commands.parallel(climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME),
                                ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()
                                                                .withTimeout(TIME_TO_LAUNCH_8)
                                                                .andThen(climbTraj.cmd()))));

                climbTraj.done().onTrue(climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME));

                return routine;
        }
public AutoRoutine leftTwoCycle (){
                        AutoRoutine routine = autoFactory.newRoutine("leftTwoCycle");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromLeft");
                AutoTrajectory scoreTraj1 = routine.trajectory("scoreAfterLeftIntake");
                AutoTrajectory depotTraj = routine.trajectory("depotAfterLeftScore");
                AutoTrajectory scoreTraj2 = routine.trajectory("scoreAfterDepot");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.done().onTrue(scoreTraj1.cmd());

                scoreTraj1.done().onTrue(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_ALL).andThen(depotTraj.cmd()));
              
                depotTraj.done().onTrue(scoreTraj2.cmd());
                scoreTraj2.done().onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()));

                return routine;
                }
                public AutoRoutine rightTwoCycle (){
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

                scoreTraj1.done().onTrue(ballSubsystem.launchCommand().withTimeout(TIME_TO_LAUNCH_ALL).andThen(outpostTraj.cmd()));
                outpostTraj.done().onTrue(Commands.waitSeconds(2).andThen(scoreTraj2.cmd()));
                scoreTraj2.done().onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()));

                return routine;
                }
}
