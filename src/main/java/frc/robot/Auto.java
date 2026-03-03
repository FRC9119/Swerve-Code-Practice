package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Auto {

        // Choreo initialization
        private final AutoFactory autoFactory;
        public final AutoChooser autoChooser;
        private final CANFuelSubsystem ballSubsystem;

        public Auto(CommandSwerveDrivetrain drivetrain, CANFuelSubsystem ballSubsystem) {
                // Set the local ballSubsystem to the ballSubsystem passed into the constructor (from RobotContainer)
                this.ballSubsystem = ballSubsystem;
                // Init Choreo's autoFactory
                autoFactory = new AutoFactory(
                                () -> drivetrain.getState().Pose,
                                drivetrain::resetPose,
                                drivetrain::followTrajectory,
                                true,
                                drivetrain);
                // Make autoChooser and add it to dashboard
                autoChooser = new AutoChooser();
                autoChooser.addRoutine("Intake and shoot", this::pickupAndScoreAuto);
                SmartDashboard.putData("auto", autoChooser);
        }

        // Choreo sample auto routine (from their website)
        private AutoRoutine pickupAndScoreAuto() {
                System.out.println("test");
                AutoRoutine routine = autoFactory.newRoutine("intakeandshoot");

                // Load the routine's trajectories (all of them)
                AutoTrajectory pickupTraj = routine.trajectory("intakeFromLeft");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterLeftIntake");

                // When the routine begins, reset odometry and start the first trajectory
                routine.active().onTrue(
                                Commands.sequence(
                                                pickupTraj.resetOdometry(),
                                                pickupTraj.cmd()));

                // Put all your trajectories and commands here to create the auton routine
                // Starting at the event marker named "intake", run the intake
                pickupTraj.atTime("intake").onTrue(ballSubsystem.intakeCommand());

                // When the trajectory is done, start the next trajectory
                pickupTraj.done().onTrue(scoreTraj.cmd());

                // While the trajectory is active, prepare the scoring subsystem
                scoreTraj.active().whileTrue(ballSubsystem.spinUpCommand());

                // When the trajectory is done, score
                scoreTraj.done().onTrue(ballSubsystem.launchCommand());

                return routine;

        }
}
