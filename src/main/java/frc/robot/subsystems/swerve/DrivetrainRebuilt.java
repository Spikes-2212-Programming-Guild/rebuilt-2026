package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.RobotMap;
import frc.robot.com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import frc.robot.com.spikes2212.command.drivetrains.swerve.SwerveModule;

public class DrivetrainRebuilt extends SwerveDrivetrain {

    private final static String NAMESPACE_NAME = "swerve drivetrain";

    public final static double MAX_POSSIBLE_VELOCITY = -1;
    private final static double TRACK_WIDTH = 0.545;
    private final static double TRACK_LENGTH = 0.545;

    private final StructArrayPublisher<SwerveModuleState> currentStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("current states", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> desiredStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("desired states", SwerveModuleState.struct).publish();

    private final Pigeon2 gyro;

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
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    @Override
    public void resetAngleSensor() {
        //@TODO check at what angle the gyro resets
        gyro.reset();
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

    @Override
    public void configureDashboard() {
        namespace.putNumber("gyro", getAngle()::getDegrees);
    }
}
