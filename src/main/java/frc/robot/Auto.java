package frc.robot;

import static frc.robot.Constants.ClimbConstants.CLIMB_CYCLE_TIME;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberInABox;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Auto {

        // Choreo initialization
        private final AutoFactory autoFactory;
        public final AutoChooser autoChooser;
        private final CANFuelSubsystem ballSubsystem;
        private final ClimberInABox climbSubsystem;

        public Auto(CommandSwerveDrivetrain drivetrain, CANFuelSubsystem ballSubsystem, ClimberInABox climbSubsystem) {
                // Set the local subsystem to the subsystems passed into the constructor
                // (real instances from RobotContainer)
                this.ballSubsystem = ballSubsystem;
                this.climbSubsystem = climbSubsystem;
                // Init Choreo's autoFactory
                autoFactory = new AutoFactory(
                                () -> drivetrain.getState().Pose,
                                drivetrain::resetPose,
                                drivetrain::followTrajectory,
                                true,
                                drivetrain);
                // Make autoChooser with all autos and add it to dashboard
                autoChooser = new AutoChooser();

                autoChooser.addRoutine("Intake, shoot (from left)", this::leftIntakeShoot);
                autoChooser.addRoutine("Intake, shoot (from right)", this::rightIntakeShoot);
                autoChooser.addRoutine("Intake, shoot, climb (from left)", this::leftIntakeShootClimb);
                autoChooser.addRoutine("Intake, shoot, climb (from right)", this::rightIntakeShootClimb);
                autoChooser.addRoutine("Shoot (from center)", this::centerShoot);
                autoChooser.addRoutine("Shoot, climb left (from center)", this::centerShootClimbLeft);
                autoChooser.addRoutine("Shoot, climb right (from center)", this::centerShootClimbRight);

                SmartDashboard.putData("auto", autoChooser);
        }

        // Use this auto as a reference
        // This function defines a routine and then returns it
        // It's a generator
        private AutoRoutine leftIntakeShootClimb() {
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

                // Starting at the event marker named "intake", run the intake
                intakeTraj.atTime("intake").onTrue(ballSubsystem.intakeCommand());

                // When the trajectory is done, start the next trajectory
                intakeTraj.done().onTrue(scoreTraj.cmd());

                // While the trajectory is active, prepare the scoring subsystem
                scoreTraj.active().whileTrue(ballSubsystem.spinUpCommand());

                // When the trajectory is done, score
                scoreTraj.done().onTrue(ballSubsystem.launchCommand()
                                // After 3 seconds, stop scoring and start the climb trajectory
                                .withTimeout(3).andThen(climbTraj.cmd()));
                // When we arrive at the climbing position, climb
                climbTraj.done().onTrue(climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME));

                // Return the finished routine
                return routine;

        }

        private AutoRoutine rightIntakeShootClimb() {
                AutoRoutine routine = autoFactory.newRoutine("rightIntakeShootClimb");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromRight");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterRightIntake");
                AutoTrajectory climbTraj = routine.trajectory("climbAfterRightScore");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.atTime("intake").onTrue(ballSubsystem.intakeCommand());

                intakeTraj.done().onTrue(scoreTraj.cmd());

                scoreTraj.active().whileTrue(ballSubsystem.spinUpCommand());

                scoreTraj.done().onTrue(ballSubsystem.launchCommand().withTimeout(3).andThen(climbTraj.cmd()));
                climbTraj.done().onTrue(climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME));

                return routine;

        }

        private AutoRoutine leftIntakeShoot() {
                AutoRoutine routine = autoFactory.newRoutine("leftIntakeShoot");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromLeft");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterLeftIntake");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.atTime("intake").onTrue(ballSubsystem.intakeCommand());

                intakeTraj.done().onTrue(scoreTraj.cmd());

                scoreTraj.active().whileTrue(ballSubsystem.spinUpCommand());

                scoreTraj.done().onTrue(ballSubsystem.launchCommand());

                return routine;

        }

        private AutoRoutine rightIntakeShoot() {
                AutoRoutine routine = autoFactory.newRoutine("rightIntakeShoot");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromRight");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterRightIntake");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.atTime("intake").onTrue(ballSubsystem.intakeCommand());

                intakeTraj.done().onTrue(scoreTraj.cmd());

                scoreTraj.active().whileTrue(ballSubsystem.spinUpCommand());

                scoreTraj.done().onTrue(ballSubsystem.launchCommand());

                return routine;

        }

        private AutoRoutine centerShoot() {
                AutoRoutine routine = autoFactory.newRoutine("centerShoot");

                AutoTrajectory scoreTraj = routine.trajectory("scoreFromCenter");

                routine.active().onTrue(
                                Commands.sequence(
                                                scoreTraj.resetOdometry(),
                                                scoreTraj.cmd(),
                                                ballSubsystem.spinUpCommand()));
                scoreTraj.done().onTrue(ballSubsystem.launchCommand());

                return routine;

        }

        // Use this as another complex example
        private AutoRoutine centerShootClimbLeft() {
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
                                                scoreTraj.cmd(),
                                                ballSubsystem.spinUpCommand()));
                // after first trajectory
                scoreTraj.done().onTrue(
                                // stay still until launcher is up to speed
                                Commands.waitUntil(() -> ballSubsystem.launchBang.atSetpoint())
                                                // then launch for three seconds
                                                .andThen(ballSubsystem.launchCommand()
                                                                .withTimeout(3)
                                                                // start next trajectory afterwards
                                                                .andThen(climbTraj.cmd())));
                // once in position, climb for the amount of time specified in Constants.java
                climbTraj.done().onTrue(climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME));

                return routine;
        }

        private AutoRoutine centerShootClimbRight() {
                AutoRoutine routine = autoFactory.newRoutine("centerShootClimbRight");

                AutoTrajectory scoreTraj = routine.trajectory("scoreFromCenter");
                AutoTrajectory climbTraj = routine.trajectory("climbRightFromCenter");
                routine.active().onTrue(
                                Commands.sequence(
                                                scoreTraj.resetOdometry(),
                                                scoreTraj.cmd(),
                                                ballSubsystem.spinUpCommand()));
                scoreTraj.done().onTrue(
                                Commands.waitUntil(() -> ballSubsystem.launchBang.atSetpoint())
                                                .andThen(ballSubsystem.launchCommand()
                                                                .withTimeout(3)
                                                                .andThen(climbTraj.cmd())));

                climbTraj.done().onTrue(climbSubsystem.climbCommand().withTimeout(CLIMB_CYCLE_TIME));

                return routine;
        }
}
