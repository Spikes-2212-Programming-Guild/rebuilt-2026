package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.AutoChooser;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.advancedcommands.Collect;
import frc.robot.commands.advancedcommands.CollectAndPass;
import frc.robot.commands.advancedcommands.Pass;
import frc.robot.commands.advancedcommands.TuneAndShoot;
import frc.robot.subsystems.forbar.Collection;
import frc.robot.subsystems.forbar.CollectionMovement;
import frc.robot.subsystems.shoot.Shooter;
import frc.robot.subsystems.spindexer.Kicker;
import frc.robot.subsystems.spindexer.SpinningMagazine;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utils.VisionService;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.function.Supplier;

public class AutonomousContainer {

    private static final RobotConfig CONFIG = getRobotConfig();

    private static final RootNamespace NAMESPACE = new RootNamespace("autonomous");

    //@TODO get the path constraints values after calibration
    //@TODO add the paths from pathplanner

    private static final TrapezoidProfile.Constraints constraints =
            new TrapezoidProfile.Constraints(CONFIG.moduleConfig.maxDriveVelocityMPS, -1);

    private static final double ROBOT_POSE_LATENCY = -1;
    private static final double FF_SCALER = -1;
    private static final double TIME_STEP = 0.02;
    private static final double PID_TO_POSE_TIMEOUT = -1;

    private static final PIDSettings X_CONTROLLER_SETTINGS =
            NAMESPACE.addPIDNamespace("x controller settings", PIDSettings.EMPTY_PID_SETTINGS);
    private static final PIDSettings Y_CONTROLLER_SETTINGS =
            NAMESPACE.addPIDNamespace("y controller settings", PIDSettings.EMPTY_PID_SETTINGS);
    private static final PIDSettings ROTATIONAL_CONTROLLER_SETTINGS =
            NAMESPACE.addPIDNamespace("rotational controller settings", PIDSettings.EMPTY_PID_SETTINGS);

    private static final AutoChooser autoChooser = AutonomousContainer.configureAutoChooser();

    private final PIDController xPidController;
    private final PIDController yPidController;
    private final ProfiledPIDController rotationalPidController;

    private final PathConstraints pathConstraints;
    private final Drivetrain drivetrain;
    private static final PathContainer pathContainer = PathContainer.getInstance();

    private Pose2d pathplannerTargetPose;

    public AutonomousContainer(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.pathConstraints = new PathConstraints(-1,-1,-1,-1);

        xPidController = buildPIDControllerFromSettings(drivetrain.getFrontRightModule().getDriveMotorPIDSettings());
        yPidController = buildPIDControllerFromSettings(drivetrain.getFrontRightModule().getDriveMotorPIDSettings());
        rotationalPidController = buildProfiledPIDControllerFromSettings(drivetrain.getFrontRightModule().
                getTurnMotorPIDSettings());

        PathfindingCommand.warmupCommand().execute();
        configureAutoBuilder();
        setupTargetPoseUpdateLoop();
        configureDashboard();
        registerNamedCommands();
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

    private static PIDController buildPIDControllerFromSettings(PIDSettings pidSettings) {
        return new PIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
    }

    private static ProfiledPIDController buildProfiledPIDControllerFromSettings(PIDSettings pidSettings){
        return new ProfiledPIDController(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD(), constraints);
    }

    private static AutoChooser configureAutoChooser(){
        return new AutoChooser(
                NAMESPACE,
                pathContainer.getShootAndToss(),
                pathContainer.getIntakeFromDepot(),
                pathContainer.getIntakeFromFeeder(),
                pathContainer.getIntakeAndShoot(),
                pathContainer.getJustToss(),
                pathContainer.getJustShoot(),
                pathContainer.getGoAndWait()
        );
    }

    private void configureDashboard(){
        NAMESPACE.putData("auto chooser", autoChooser);
    }

    public static Command getSelectedCommand() {
        return autoChooser.getSelected();
    }

    private void registerNamedCommands(){
        NamedCommands.registerCommand("collect and pass", new CollectAndPass(
                Collection.getInstance(), CollectionMovement.getInstance(), Shooter.getInstance(), () -> 0.5,
                SpinningMagazine.getInstance(), Kicker.getInstance()));
        NamedCommands.registerCommand("collect", new Collect(Collection.getInstance(),
                CollectionMovement.getInstance()));
        NamedCommands.registerCommand("pass", new Pass(Shooter.getInstance(), () -> 0.5,
                SpinningMagazine.getInstance(), Kicker.getInstance()));
        NamedCommands.registerCommand("aligned shoot", new TuneAndShoot(Shooter.getInstance(),
                Kicker.getInstance(), SpinningMagazine.getInstance(), VisionService.getInstance()));
    }

    public static boolean shouldMirror() {
        return DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).
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