package frc.robot.pathplanner;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
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
import frc.robot.com.spikes2212.dashboard.AutoChooser;
import frc.robot.com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.swerve.DrivetrainRebuilt;
import frc.robot.subsystems.swerve.SwerveModuleHolder;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class AutonomousContainer {

    private static final RobotConfig CONFIG = getRobotConfig();

    private static final RootNamespace namespace = new RootNamespace("autonomous");

    //@TODO add the named commands and the paths to the branch
    //@TODO get the path constraints values after calibration
    //@TODO add the paths once added in to the autoChooser

    public static final AutoChooser autoChooser = null;

    private static final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(CONFIG.moduleConfig.maxDriveVelocityMPS, -1);

    private static final double ROBOT_POSE_LATENCY = -1;
    private static final double FF_SCALER = -1;
    private static final double TIME_STEP = 0.02;
    private static final double PID_TO_POSE_TIMEOUT = -1;

    private final PIDController X_PID_CONTROLLER;
    private final PIDController Y_PID_CONTROLLER;
    private final ProfiledPIDController ROTATIONAL_PID_CONTROLLER;

    private final PathConstraints pathConstraints;
    private final DrivetrainRebuilt drivetrain;

    private PIDSettings X_CONTROLLER_PID_SETTINGS;
    private PIDSettings Y_CONTROLLER_PID_SETTINGS;
    private PIDSettings ROTATIONAL_CONTROLLER_PID_SETTINGS;

    private Pose2d pathplannerTargetPose;

    public AutonomousContainer(DrivetrainRebuilt drivetrain) {
        this.drivetrain = drivetrain;
        this.pathConstraints = new PathConstraints(0,0,0,0);
        X_PID_CONTROLLER = buildPIDControllerFromSettings(X_CONTROLLER_PID_SETTINGS);
        Y_PID_CONTROLLER = buildPIDControllerFromSettings(Y_CONTROLLER_PID_SETTINGS);
        ROTATIONAL_PID_CONTROLLER = buildProfiledPIDControllerFromSettings(ROTATIONAL_CONTROLLER_PID_SETTINGS);
        PathfindingCommand.warmupCommand().schedule();
        configureDashboard();
        configureAutoBuilder();
        setupTargetPoseUpdateLoop();
        getSelectedCommand();
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
                drivetrain::getEstimatedPose,
                drivetrain::resetPose,
                drivetrain::getSpeeds,
                this::driveByCorrectedSpeed,
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

    private void driveByCorrectedSpeed(ChassisSpeeds feedForwardSpeeds) {
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
                () -> drivetrain.atPose(targetPose),
                drivetrain
        ).withTimeout(PID_TO_POSE_TIMEOUT);
    }

    private void driveWithPIDtoPose(Pose2d targetPose) {
        ChassisSpeeds speeds = getPIDChassisSpeedsToPose(targetPose);
        drivetrain.drive(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                false,
                TIME_STEP,
                true
        );
    }

    public Command driveRobotByPathToPose(Pose2d targetPose) {
        return new SequentialCommandGroup(
                AutoBuilder.pathfindToPose(targetPose, pathConstraints),
                getPIDtoPoseCommand(targetPose)
        );
    }

    private void setupTargetPoseUpdateLoop() {
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> pathplannerTargetPose = pose);
    }

    private PIDController buildPIDControllerFromSettings(PIDSettings pidSettings) {
        return new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }

    private ProfiledPIDController buildProfiledPIDControllerFromSettings(PIDSettings pidSettings){
        return new ProfiledPIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD(), constraints);
    }
    public static Command getSelectedCommand() {
        return autoChooser.getSelected();
    }

    private boolean shouldMirror() {
        return DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Blue).
                orElse(false);
    }

    private static PathPlannerPath loadPathFromFile(String pathName) {
        try {
            return PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }

    private void configureDashboard(){
        namespace.putData("auto chooser", autoChooser);
        X_CONTROLLER_PID_SETTINGS = namespace.addPIDNamespace("x pid settings", PIDSettings.EMPTY_PID_SETTINGS);
        Y_CONTROLLER_PID_SETTINGS = namespace.addPIDNamespace("y pid settings", PIDSettings.EMPTY_PID_SETTINGS);
        ROTATIONAL_CONTROLLER_PID_SETTINGS =
                namespace.addPIDNamespace("rotational pid settings", PIDSettings.EMPTY_PID_SETTINGS);
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
