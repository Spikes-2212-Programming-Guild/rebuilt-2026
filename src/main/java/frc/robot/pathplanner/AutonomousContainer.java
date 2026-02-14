package frc.robot.pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.com.spikes2212.control.PIDSettings;
import frc.robot.com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.swerve.DrivetrainRebuilt;
import frc.robot.subsystems.swerve.SwerveModuleHolder;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class AutonomousContainer {

    private static final RobotConfig CONFIG = getRobotConfig();

    private static final RootNamespace namespace = new RootNamespace("autonomous");
    private final SendableChooser<Command> autoChooser;

    private static final TrapezoidProfile.Constraints pathConstraints =
            new TrapezoidProfile.Constraints(CONFIG.moduleConfig.maxDriveVelocityMPS, -1);

    private static final PIDController X_PID_CONTROLLER =
            setPIDSettingsIntoController(SwerveModuleHolder.getFrontLeft().getDriveMotorPIDSettings());
    private static final PIDController Y_PID_CONTROLLER =
            setPIDSettingsIntoController(SwerveModuleHolder.getFrontLeft().getTurnMotorPIDSettings());
    private static final ProfiledPIDController ROTATIONAL_PID_CONTROLLER =
            new ProfiledPIDController(-1, -1, -1, pathConstraints);

    private static final double ROBOT_POSE_LATENCY = -1;
    private static final double FF_SCALER = -1;
    private static final double TIME_STEP = 0.02;
    private static final double PID_TO_POSE_TIMEOUT = -1;

    private final DrivetrainRebuilt drivetrain;

    private Pose2d pathplannerTargetPose;

    public AutonomousContainer(DrivetrainRebuilt drivetrain) {
        this.drivetrain = drivetrain;
        autoChooser = AutoBuilder.buildAutoChooser();
        PathfindingCommand.warmupCommand().schedule();
        configureAutoBuilder();
        updatePathplannerPose();
        getAutoPath();
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
                drivetrain::getEstimatedPose,
                drivetrain::resetPose,
                drivetrain::getSpeeds,
                this::updatePathFollowingOutput,
                new PPHolonomicDriveController(
                        new PIDConstants(0, 0, 0),
                        new PIDConstants(0, 0, 0)),
                CONFIG,
                this::shouldMirror,
                drivetrain);
    }

    private ChassisSpeeds pathRelativeSpeedsByPID(Pose2d targetPose) {
        Pose2d currentPose = drivetrain.getFixedPoseByLatency(ROBOT_POSE_LATENCY);
        double xSpeed = X_PID_CONTROLLER.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = Y_PID_CONTROLLER.calculate(currentPose.getY(), targetPose.getY());
        double rotationalSpeed = ROTATIONAL_PID_CONTROLLER.calculate(drivetrain.getAngle().getRadians(),
                targetPose.getRotation().getRadians());

        xSpeed = X_PID_CONTROLLER.atSetpoint() ? 0 : xSpeed;
        ySpeed = Y_PID_CONTROLLER.atSetpoint() ? 0 : ySpeed;
        rotationalSpeed = ROTATIONAL_PID_CONTROLLER.atSetpoint() ? 0 : rotationalSpeed;

        return new ChassisSpeeds(xSpeed, ySpeed, rotationalSpeed);
    }

    private ChassisSpeeds getScaledFFSpeeds(ChassisSpeeds feedForwardSpeeds) {
        return feedForwardSpeeds.times(FF_SCALER);
    }

    public void updatePathFollowingOutput(ChassisSpeeds feedForwardSpeeds) {
        ChassisSpeeds calculatePID = pathRelativeSpeedsByPID(pathplannerTargetPose);
        ChassisSpeeds scaledFeedForward = getScaledFFSpeeds(feedForwardSpeeds);
        ChassisSpeeds output = calculatePID.plus(scaledFeedForward);
        drivetrain.drive(
                output.vxMetersPerSecond,
                output.vyMetersPerSecond,
                output.omegaRadiansPerSecond,
                false,
                TIME_STEP,
                true
        );
    }

    private Command PIDtoTargetPose(Pose2d targetPose) {
        return new FunctionalCommand(
                () -> {
                },
                () -> PIDtoPose(targetPose),
                (interrupted) -> {
                },
                () -> drivetrain.atPose(targetPose)).withTimeout(PID_TO_POSE_TIMEOUT);
    }

    private void PIDtoPose(Pose2d targetPose) {
        drivetrain.drive(
                pathRelativeSpeedsByPID(targetPose).vxMetersPerSecond,
                pathRelativeSpeedsByPID(targetPose).vyMetersPerSecond,
                pathRelativeSpeedsByPID(targetPose).omegaRadiansPerSecond,
                false,
                TIME_STEP,
                true
        );
    }

    public Command correctPathToPose(Pose2d targetPose, PathConstraints constraints) {
        return new SequentialCommandGroup(
                AutoBuilder.pathfindToPose(targetPose, constraints),
                PIDtoTargetPose(targetPose)
        );
    }

    private void updatePathplannerPose() {
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> pathplannerTargetPose = pose);
    }

    private static PIDController setPIDSettingsIntoController(PIDSettings pidSettings) {
        return new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }

    public boolean shouldMirror() {
        return DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Blue).
                orElse(false);
    }

    private void getAutoPath() {
        namespace.putData("auto chooser", autoChooser);
    }

    public Command getSelectedAutoCommand() {
        return autoChooser.getSelected();
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
