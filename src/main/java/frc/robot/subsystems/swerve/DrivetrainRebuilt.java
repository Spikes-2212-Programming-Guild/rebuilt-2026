package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import frc.robot.com.spikes2212.command.drivetrains.swerve.SwerveModule;
import frc.robot.util.localization.RobotPoseEstimator;
import frc.robot.util.odometry.OdometryMeasurement;
import frc.robot.util.odometry.PeriodicTaskScheduler;

public class DrivetrainRebuilt extends SwerveDrivetrain {

    private final static String NAMESPACE_NAME = "swerve drivetrain";

    private final static double MAX_POSSIBLE_VELOCITY = -1;
    private final static double TRACK_WIDTH = -1;
    private final static double TRACK_LENGTH = -1;

    private final static double TRANSLATION_POSE_TOLERANCE = -1;
    private final static double TRANSLATION_VELOCITY_TOLERANCE = -1;
    private final static double ROTATION_TOLERANCE_IN_DEGREES = -1;
    private static final double ROTATION_VELOCITY_TOLERANCE = -1;

    private final StructArrayPublisher<SwerveModuleState> currentStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("current states", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> desiredStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("desired states", SwerveModuleState.struct).publish();

    private final Pigeon2 gyro;

    private final RobotPoseEstimator poseEstimator;

    private final SwerveModulePosition[] swerveModulePositions;

    private static DrivetrainRebuilt instance;

    public static DrivetrainRebuilt getInstance() {
        if (instance == null) {
            new DrivetrainRebuilt(NAMESPACE_NAME, SwerveModuleHolder.getFrontLeft(),
                    SwerveModuleHolder.getFrontRight(), SwerveModuleHolder.getBackLeft(),
                    SwerveModuleHolder.getBackRight(), TRACK_WIDTH, TRACK_LENGTH, MAX_POSSIBLE_VELOCITY,
                    new Pigeon2(RobotMap.CAN.SWERVE_GYRO_PIGEON_2_ID));
        }
        return instance;
    }

    private DrivetrainRebuilt(String namespaceName, SwerveModule frontLeftModule, SwerveModule frontRightModule,
                              SwerveModule backLeftModule, SwerveModule backRightModule, double drivetrainTrackWidth,
                              double drivetrainTrackLength, double maxPossibleVelocity, Pigeon2 gyro) {
        super(namespaceName, frontLeftModule, frontRightModule, backLeftModule, backRightModule, drivetrainTrackWidth,
                drivetrainTrackLength, maxPossibleVelocity);
        this.gyro = gyro;

        swerveModulePositions = getModulePositions();

        poseEstimator = new RobotPoseEstimator(
                getKinematics(), getAngle(), swerveModulePositions, new Pose2d(),
                () -> new OdometryMeasurement(Timer.getFPGATimestamp(), getAngle(), swerveModulePositions),
                PeriodicTaskScheduler.getInstance());

        setStates(currentStates,
                new SwerveModuleState[]{
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState()
                });
        setStates(desiredStates,
                new SwerveModuleState[]{
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState()
                });
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    @Override
    public void resetAngleSensor() {
        gyro.reset();
    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative,
                      double timeStep, boolean useVelocityPID) {
        super.drive(xSpeed, ySpeed, rotationSpeed, isFieldRelative,
                timeStep, useVelocityPID);

        setStates(currentStates,
                new SwerveModuleState[]{
                        frontLeftModule.getModuleState(),
                        frontRightModule.getModuleState(),
                        backLeftModule.getModuleState(),
                        backRightModule.getModuleState()
                });

        setStates(desiredStates, getDesiredStates(xSpeed, ySpeed, rotationSpeed, isFieldRelative, timeStep));
    }

    public void setStates(StructArrayPublisher<SwerveModuleState> states, SwerveModuleState[] desiredStatesToSet) {
        states.set(desiredStatesToSet);
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{frontLeftModule.getModulePosition(),
                frontRightModule.getModulePosition(), backLeftModule.getModulePosition(),
                backRightModule.getModulePosition()};
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPose();
    }

    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPose(newPose);
    }

    public ChassisSpeeds getSelfRelativeSpeeds() {
        return kinematics.toChassisSpeeds(
                frontLeftModule.getModuleState(),
                frontRightModule.getModuleState(),
                backLeftModule.getModuleState(),
                backRightModule.getModuleState());
    }

    public Pose2d getFixedPoseByLatency(double latency) {
        return poseEstimator.getEstimatedPoseByLatency(getSpeeds(), latency);
    }

    private boolean atAxis(double currentAxisPose, double targetAxisPose, double currentVelocity) {
        boolean isAtPose = Math.abs(currentAxisPose - targetAxisPose) <= TRANSLATION_POSE_TOLERANCE;
        boolean isRobotStill = Math.abs(currentVelocity) <= TRANSLATION_VELOCITY_TOLERANCE;
        return isAtPose && isRobotStill;
    }

    private boolean atRotation(Rotation2d rotation2d) {
        double error = rotation2d.minus(getAngle()).getDegrees();
        boolean isAtRotation = Math.abs(error) <= ROTATION_TOLERANCE_IN_DEGREES;
        boolean isRotationStill = Math.abs(getSpeeds().omegaRadiansPerSecond)
                <= ROTATION_VELOCITY_TOLERANCE;
        return isAtRotation && isRotationStill;
    }

    public boolean atPose(Pose2d pose2d) {
        boolean atXAxis = atAxis(getEstimatedPose().getX(), pose2d.getX(),
                getSpeeds().vxMetersPerSecond);
        boolean atYAxis = atAxis(getEstimatedPose().getY(), pose2d.getY(),
                getSpeeds().vyMetersPerSecond);
        return atXAxis && atYAxis && atRotation(pose2d.getRotation());
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro", getAngle()::getDegrees);
    }
}
