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
        return targetHub().withTimeout(1).andThen(ballSubsystem.spinUpCommand().until(()->ballSubsystem.launchPID.atSetpoint())).andThen(ballSubsystem.launchCommand());
}

        public Command targetHub() {
                return drivetrain.applyRequest(() -> targetHubReq
                                .withTargetDirection(Targeting.getTargetRotation(drivetrain.getPose())));
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
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterLeftIntake_trench");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.chain(scoreTraj);

                scoreTraj.done().onTrue(launchSequence());

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

                intakeTraj.chain(scoreTraj);

                scoreTraj.done().onTrue(launchSequence());

                return routine;

        }

        public AutoRoutine centerShoot() {
                AutoRoutine routine = autoFactory.newRoutine("centerShoot");

                AutoTrajectory scoreTraj = routine.trajectory("scoreFromCenter");

                routine.active().onTrue(
                                Commands.sequence(
                                                scoreTraj.resetOdometry(),
                                                scoreTraj.cmd()));
                scoreTraj.done().onTrue(ballSubsystem.spinUpCommand().until(() -> ballSubsystem.launchPID.atSetpoint())
                                .andThen(ballSubsystem.launchCommand().alongWith(targetHub()).withTimeout(TIME_TO_LAUNCH_8)));

                return routine;

        }

        public AutoRoutine centerShootSweepLeft() {
                AutoRoutine routine = autoFactory.newRoutine("centerShootSweepLeft");

                AutoTrajectory scoreTraj = routine.trajectory("scoreFromCenter");
                AutoTrajectory sweepTraj = routine.trajectory("intakeLeftFromCenterScore");

                double secondsToWait = SmartDashboard.getNumber("Seconds to wait after center shoot before intaking", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                scoreTraj.resetOdometry(),
                                                scoreTraj.cmd()));
                scoreTraj.done().onTrue(launchSequence().withTimeout(TIME_TO_LAUNCH_8)
                                .andThen(Commands.waitSeconds(secondsToWait).andThen(sweepTraj.cmd().asProxy())));

                return routine;
        }

        public AutoRoutine centerShootSweepRight() {
                AutoRoutine routine = autoFactory.newRoutine("centerShootSweepRight");

                AutoTrajectory scoreTraj = routine.trajectory("scoreFromCenter");
                AutoTrajectory sweepTraj = routine.trajectory("intakeRightFromCenterScore");

                double secondsToWait = SmartDashboard.getNumber("Seconds to wait after center shoot before intaking", 0);

                routine.active().onTrue(
                                Commands.sequence(
                                                scoreTraj.resetOdometry(),
                                                scoreTraj.cmd()));
                scoreTraj.done().onTrue(launchSequence()
                                .withTimeout(TIME_TO_LAUNCH_8)
                                .andThen(Commands.waitSeconds(secondsToWait).andThen(sweepTraj.cmd().asProxy())));

                return routine;
        }

        public AutoRoutine leftTwoCycleFullField() {
                AutoRoutine routine = autoFactory.newRoutine("leftTwoCycleFullField");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromLeft");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterLeftIntake_trench");
                AutoTrajectory sweepAndScoreTraj = routine.trajectory("sweepScoreFromLeftScore");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.chain(scoreTraj);

                scoreTraj.done().onTrue(launchSequence().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(sweepAndScoreTraj.cmd().asProxy()));

                sweepAndScoreTraj.done()
                                .onTrue(launchSequence());

                return routine;
        }

        public AutoRoutine leftTwoCycleBump() {
                AutoRoutine routine = autoFactory.newRoutine("leftTwoCycleBump");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromLeft");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterLeftIntake_trench");
                AutoTrajectory sweepAndScoreTraj = routine.trajectory("sweepScoreFromLeftScore_bump");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.chain(scoreTraj);

                scoreTraj.done().onTrue(launchSequence().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(sweepAndScoreTraj.cmd().asProxy()));

                sweepAndScoreTraj.done()
                                .onTrue(launchSequence());

                return routine;
        }

        public AutoRoutine rightTwoCycleBump() {
                AutoRoutine routine = autoFactory.newRoutine("rightTwoCycleBump");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromRight");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterRightIntake_trench");
                AutoTrajectory sweepAndScoreTraj = routine.trajectory("sweepScoreFromRightScore_bump");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.chain(scoreTraj);

                scoreTraj.done().onTrue(launchSequence().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(sweepAndScoreTraj.cmd().asProxy()));

                sweepAndScoreTraj.done()
                                .onTrue(launchSequence());

                return routine;
        }

        public AutoRoutine rightTwoCycleFullField() {
                AutoRoutine routine = autoFactory.newRoutine("rightTwoCycleFullField");

                AutoTrajectory intakeTraj = routine.trajectory("intakeFromRight");
                AutoTrajectory scoreTraj = routine.trajectory("scoreAfterRightIntake_trench");
                AutoTrajectory sweepTraj = routine.trajectory("sweepScoreFromRightScore");

                routine.active().onTrue(
                                Commands.sequence(
                                                intakeTraj.resetOdometry(),
                                                intakeTraj.cmd()));

                intakeTraj.chain(scoreTraj);

                scoreTraj.done().onTrue(launchSequence().withTimeout(TIME_TO_LAUNCH_ALL)
                                .andThen(sweepTraj.cmd().asProxy()));
                sweepTraj.done()
                                .onTrue(launchSequence());

                return routine;
        }

}
