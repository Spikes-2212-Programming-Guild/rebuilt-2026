package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.AutoChooser;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.RotateAccordingToGyro;
import frc.robot.subsystems.swerve.Drivetrain;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.Supplier;

public class AutonomousContainer {

    private static final RobotConfig CONFIG = getRobotConfig();

    private static final RootNamespace namespace = new RootNamespace("autonomouss");

    private static final PathConstraints pathConstraints = new PathConstraints(5.1,
            2, 8, 2);

    private static final double ROBOT_POSE_LATENCY = 0;
    private static final double TIME_STEP = 0.02;
    private static final double PID_TO_POSE_TIMEOUT = 0.1;
    private static final Supplier<Double> FF_SCALER = namespace.addConstantDouble("ff scaler", 1);

    private static final PIDSettings X_CONTROLLER_SETTINGS =
            namespace.addPIDNamespace("x controller settings", PIDSettings.EMPTY_PID_SETTINGS);
    private static final PIDSettings Y_CONTROLLER_SETTINGS =
            namespace.addPIDNamespace("y controller settings", PIDSettings.EMPTY_PID_SETTINGS);
    private static final PIDSettings ROTATIONAL_CONTROLLER_SETTINGS =
            namespace.addPIDNamespace("rotational controller settings", PIDSettings.EMPTY_PID_SETTINGS);

    private final SendableChooser<Command> autoChooser;

    private final PIDController xPidController;
    private final PIDController yPidController;
    private final PIDController rotationalPidController;

    private final Drivetrain drivetrain;
    private static final PathContainer pathContainer = PathContainer.getInstance();

    private Pose2d pathplannerTargetPose;

    public AutonomousContainer(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        xPidController = buildPIDControllerFromSettings(drivetrain.getBackLeftModule().getDriveMotorPIDSettings());
        yPidController = buildPIDControllerFromSettings(drivetrain.getBackRightModule().getTurnMotorPIDSettings());
        rotationalPidController = buildPIDControllerFromSettings(RotateAccordingToGyro.rotatePIDSettings);

//          PathfindingCommand.warmupCommand().execute();
//        PathContainer.createAutos();

        configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();

        setupTargetPoseUpdateLoop();
        configureDashboard();
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
                AutonomousContainer::shouldMirror,
                drivetrain
        );
    }

    private ChassisSpeeds getPIDChassisSpeedsToPose(Pose2d targetPose) {
        Pose2d currentPose = drivetrain.getEstimatedPoseByLatency(drivetrain.getSpeeds(), ROBOT_POSE_LATENCY);
        double xSpeed = xPidController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yPidController.calculate(currentPose.getY(), targetPose.getY());
        double rotationalSpeed = rotationalPidController.calculate(drivetrain.getAngle().getRadians(),
                targetPose.getRotation().getRadians());

        xSpeed = xPidController.atSetpoint() ? 0 : xSpeed;
        ySpeed = yPidController.atSetpoint() ? 0 : ySpeed;
        rotationalSpeed = rotationalPidController.atSetpoint() ? 0 : rotationalSpeed;

        return new ChassisSpeeds(xSpeed, ySpeed, rotationalSpeed);
    }

    private ChassisSpeeds getScaledFFSpeeds(ChassisSpeeds feedForwardSpeeds) {
        return feedForwardSpeeds.times(FF_SCALER.get());
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

    private void configureDashboard() {
        namespace.putData("auto chooser", autoChooser);
        namespace.putBoolean("mirror", AutonomousContainer::shouldMirror);
    }

    public Command getSelectedCommand() {
        return autoChooser.getSelected();
    }

    private static PIDController buildPIDControllerFromSettings(PIDSettings pidSettings) {
        return new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }

    public static boolean shouldMirror() {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Blue).isPresent();
    }

    private static RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }
}
