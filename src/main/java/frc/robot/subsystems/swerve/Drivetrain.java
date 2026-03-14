package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.RobotMap;
import com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import com.spikes2212.command.drivetrains.swerve.SwerveModule;

import java.util.function.Supplier;

public class Drivetrain extends SwerveDrivetrain {

    public static final double MAX_POSSIBLE_VELOCITY = 5.1;

    private static final String NAMESPACE_NAME = "swerve drivetrain";

    private static final CANBus CANIVORE = new CANBus("canivore");

    private static final double TRACK_WIDTH = 0.545;
    private static final double TRACK_LENGTH = 0.545;
    private static final int GYRO_OFFSET = 180;

    private final static double TRANSLATION_POSE_TOLERANCE = -1;
    private final static double TRANSLATION_VELOCITY_TOLERANCE = -1;
    private final static double ROTATION_TOLERANCE_IN_DEGREES = -1;
    private static final double ROTATION_VELOCITY_TOLERANCE = -1;

    private final StructArrayPublisher<SwerveModuleState> currentStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("current states", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> desiredStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("desired states", SwerveModuleState.struct).publish();

    private final Pigeon2 gyro;

    private final SwerveDriveOdometry odometry;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(NAMESPACE_NAME, SwerveModuleHolder.getFrontLeft(),
                    SwerveModuleHolder.getFrontRight(), SwerveModuleHolder.getBackLeft(),
                    SwerveModuleHolder.getBackRight(), TRACK_WIDTH, TRACK_LENGTH, MAX_POSSIBLE_VELOCITY,
                    new Pigeon2(RobotMap.CAN.SWERVE_GYRO_PIGEON_2_ID, CANIVORE));
        }
        return instance;
    }

    private Drivetrain(String namespaceName, SwerveModule frontLeftModule, SwerveModule frontRightModule,
                       SwerveModule backLeftModule, SwerveModule backRightModule,
                       double drivetrainTrackWidth, double drivetrainTrackLength,
                       double maxPossibleVelocity, Pigeon2 gyro) {
        super(namespaceName, frontLeftModule, frontRightModule, backLeftModule, backRightModule,
                drivetrainTrackWidth, drivetrainTrackLength, maxPossibleVelocity);
        this.gyro = gyro;

        setStructArrayStates(currentStates,
                new SwerveModuleState[]{
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState()
                });
        setStructArrayStates(desiredStates,
                new SwerveModuleState[]{
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState()
                });

        odometry = new SwerveDriveOdometry(getKinematics(), getAngle(), getSwerveModulePositions());

        configureDashboard();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    @Override
    public void resetAngleSensor() {
        gyro.setYaw(GYRO_OFFSET);
    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean isFieldRelative,
                      double timeStep, boolean useVelocityPID) {
        super.drive(xSpeed, ySpeed, rotationSpeed, isFieldRelative,
                timeStep, useVelocityPID);

        setStructArrayStates(currentStates,
                new SwerveModuleState[]{
                        frontLeftModule.getModuleState(),
                        frontRightModule.getModuleState(),
                        backLeftModule.getModuleState(),
                        backRightModule.getModuleState()
                });

        setStructArrayStates(desiredStates, getDesiredStates(xSpeed, ySpeed, rotationSpeed, isFieldRelative,
                timeStep));
    }

    public void setStructArrayStates(StructArrayPublisher<SwerveModuleState> states,
                                     SwerveModuleState[] desiredStatesToSet) {
        states.set(desiredStatesToSet);
    }

    public Pose2d getEstimatedPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d newPose) {
        odometry.resetPose(newPose);
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

    public Pose2d getEstimatedPoseByLatency(ChassisSpeeds relativeSpeeds, double latencySeconds) {
        double predictedXSpeed = relativeSpeeds.vxMetersPerSecond * latencySeconds;
        double predictedYSpeed = relativeSpeeds.vyMetersPerSecond * latencySeconds;
        Rotation2d predictedRotationSpeed =
                Rotation2d.fromRadians(relativeSpeeds.omegaRadiansPerSecond * latencySeconds);
        return getEstimatedPose().
                transformBy(new Transform2d(predictedXSpeed, predictedYSpeed, predictedRotationSpeed));
    }

    public double getXSpeed(){
        return getSpeeds().vxMetersPerSecond;
    }

    public double getYSpeed(){
        return getSpeeds().vyMetersPerSecond;
    }

    public double getRotationSpeed(){
        return getSpeeds().omegaRadiansPerSecond;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro", () -> this.getAngle().getDegrees());
    }
}
