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
import frc.robot.com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.swerve.DrivetrainRebuilt;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class AutonomousContainer {

    private static final RobotConfig CONFIG = getRobotConfig();

    private static final RootNamespace namespace = new RootNamespace("autonomous");
    private static final SendableChooser<Command> autoChooser = getAutoPath();

    private static final TrapezoidProfile.Constraints pathConstraints =
            new TrapezoidProfile.Constraints(CONFIG.moduleConfig.maxDriveVelocityMPS, -1);

    private static final PIDController X_PID_CONTROLLER = new PIDController(-1, -1, -1);
    private static final PIDController Y_PID_CONTROLLER = new PIDController(-1, -1, -1);
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
        updatePathplannerPose();
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
                drivetrain::getEstimatedPose,
                drivetrain::resetPose,
                drivetrain::getSelfRelativeSpeeds,
                this::driveCorrection,
                new PPHolonomicDriveController(
                        new PIDConstants(0, 0, 0),
                        new PIDConstants(0,0,0)),
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

    public void driveCorrection(ChassisSpeeds feedForwardSpeeds) {
        ChassisSpeeds pidCorrection = pathRelativeSpeedsByPID(pathplannerTargetPose);
        ChassisSpeeds scaledFeedForward = getScaledFFSpeeds(feedForwardSpeeds);
        ChassisSpeeds output = pidCorrection.plus(scaledFeedForward);
        drivetrain.driveSelfRelative(output, TIME_STEP, true);
    }

    private Command PIDtoTargetPose(Pose2d targetPose, double timeoutSeconds){
        return new FunctionalCommand(() -> {},
                () -> PIDtoPose(targetPose),
                (interrupted) -> {},
                () -> drivetrain.atPose(targetPose)).withTimeout(timeoutSeconds);
    }

    private void PIDtoPose(Pose2d targetPose){
        drivetrain.driveSelfRelative(pathRelativeSpeedsByPID(targetPose), TIME_STEP, true);
    }

    public Command correctPathToPose(Pose2d targetPose, PathConstraints constraints) {
        return new SequentialCommandGroup(
                AutoBuilder.pathfindToPose(targetPose, constraints),
                PIDtoTargetPose(targetPose, PID_TO_POSE_TIMEOUT)
        );
    }

    private void updatePathplannerPose(){
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> pathplannerTargetPose = pose);
    }

    public boolean shouldMirror(){
        return DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Blue).orElse(false);
    }

    private static SendableChooser<Command> getAutoPath() {
        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
        namespace.putData("auto chooser", chooser);
        return chooser;
    }

    //should be in Robot - autonomousInit()

    public static Command getSelectedAutoCommand() {
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
