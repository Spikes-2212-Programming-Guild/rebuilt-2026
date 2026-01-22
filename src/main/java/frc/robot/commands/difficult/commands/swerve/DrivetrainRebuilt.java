package frc.robot.commands.difficult.commands.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotMap;
import frc.robot.com.spikes2212.command.drivetrains.swerve.SwerveDrivetrain;
import frc.robot.com.spikes2212.command.drivetrains.swerve.SwerveModule;

public class DrivetrainRebuilt extends SwerveDrivetrain {

    private final static String NAMESPACE_NAME = "swerve drivetrain";

    private final static double MAX_POSSIBLE_VELOCITY = -1;
    private final static double TRACK_WIDTH = -1;
    private final static double TRACK_LENGTH = -1;

    private final Pigeon2 gyro;

    private static DrivetrainRebuilt instance;

public static DrivetrainRebuilt getInstance(){
    if (instance == null){
        new DrivetrainRebuilt(NAMESPACE_NAME, SwerveModuleHolder.getFrontLeft(), SwerveModuleHolder.getFrontRight(),
                SwerveModuleHolder.getBackLeft(), SwerveModuleHolder.getBackRight(),TRACK_WIDTH, TRACK_LENGTH,
                MAX_POSSIBLE_VELOCITY, new Pigeon2(RobotMap.CAN.SWERVE_GYRO_PIGEON_2_ID));
    }
    return instance;
}
    private DrivetrainRebuilt(String namespaceName, SwerveModule frontLeftModule, SwerveModule frontRightModule, SwerveModule backLeftModule, SwerveModule backRightModule, double drivetrainTrackWidth, double drivetrainTrackLength, double maxPossibleVelocity, Pigeon2 gyro) {
        super(namespaceName, frontLeftModule, frontRightModule, backLeftModule, backRightModule, drivetrainTrackWidth, drivetrainTrackLength, maxPossibleVelocity);
        this.gyro = gyro;
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
    public void configureDashboard() {

    }
}
