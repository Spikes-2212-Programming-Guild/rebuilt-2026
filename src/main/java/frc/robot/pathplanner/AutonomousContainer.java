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

    private static final RootNamespace NAMESPACE = new RootNamespace("autonomous");

    //@TODO add the named commands and the paths to the branch
    //@TODO add autoChooser

    private static final TrapezoidProfile.Constraints pathConstraints =
            new TrapezoidProfile.Constraints(CONFIG.moduleConfig.maxDriveVelocityMPS, -1);

    private static final PIDController X_PID_CONTROLLER =
            setPIDSettings(SwerveModuleHolder.getFrontLeft().getDriveMotorPIDSettings());

    private static final PIDController Y_PID_CONTROLLER =
            setPIDSettings(SwerveModuleHolder.getFrontLeft().getTurnMotorPIDSettings());

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
        PathfindingCommand.warmupCommand().schedule();
        configureAutoBuilder();
        updateTargetPose();
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
                drivetrain::getEstimatedPose,
                drivetrain::resetPose,
                drivetrain::getSpeeds,
                this::updatePathDriveSpeeds,
                new PPHolonomicDriveController(
                        new PIDConstants(0, 0, 0),
                        new PIDConstants(0, 0, 0)),
                CONFIG,
                this::shouldMirror,
                drivetrain
        );
    }

    private ChassisSpeeds getPIDChassisSpeedsToPose(Pose2d targetPose) {
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

    public void updatePathDriveSpeeds(ChassisSpeeds feedForwardSpeeds) {
        ChassisSpeeds calculatePID = getPIDChassisSpeedsToPose(pathplannerTargetPose);
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

    private Command getPIDtoPoseCommand(Pose2d targetPose) {
        return new FunctionalCommand(
                () -> {
                },
                () -> driveWithPIDtoPose(targetPose),
                (interrupted) -> {
                },
                () -> drivetrain.atPose(targetPose)
        ).withTimeout(PID_TO_POSE_TIMEOUT);
    }

    private void driveWithPIDtoPose(Pose2d targetPose) {
        drivetrain.drive(
                getPIDChassisSpeedsToPose(targetPose).vxMetersPerSecond,
                getPIDChassisSpeedsToPose(targetPose).vyMetersPerSecond,
                getPIDChassisSpeedsToPose(targetPose).omegaRadiansPerSecond,
                false,
                TIME_STEP,
                true
        );
    }

    public Command driveRobotByPathToPose(Pose2d targetPose, PathConstraints constraints) {
        return new SequentialCommandGroup(
                AutoBuilder.pathfindToPose(targetPose, constraints),
                getPIDtoPoseCommand(targetPose)
        );
    }

    private void updateTargetPose() {
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> pathplannerTargetPose = pose);
    }

    private static PIDController setPIDSettings(PIDSettings pidSettings) {
        return new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }

    private boolean shouldMirror() {
        return DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Blue).
                orElse(false);
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
