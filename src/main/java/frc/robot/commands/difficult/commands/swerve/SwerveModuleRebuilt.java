package frc.robot.commands.difficult.commands.swerve;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.com.spikes2212.command.drivetrains.swerve.SwerveModule;
import frc.robot.com.spikes2212.control.FeedForwardSettings;
import frc.robot.com.spikes2212.control.PIDSettings;
import frc.robot.com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;

public class SwerveModuleRebuilt extends SwerveModule {

    private final static double DRIVE_GEAR_RATIO = -1;
    private final static double TURN_GEAR_RATIO = -1;
    private final static double WHEAL_DIAMETER_TO_INCHES = -1;
    private final static double INCHES_TO_METERS = 0.0254;
    private final static int DEGREES_IN_ROTATION = 360;
    private final static int SECONDS_IN_MINUTE = 60;
    private final static int ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;

    private final TalonFXWrapper driveMotor;
    private final SparkWrapper turnMotor;
    private final CANcoder cancoder;


    public SwerveModuleRebuilt(String namespaceName, TalonFXWrapper driveMotor, SparkWrapper turnMotor,
                               boolean driveMotorInverted, boolean turnMotorInverted, double absoluteEncoderOffset,
                               PIDSettings driveMotorPIDSettings, PIDSettings turnMotorPIDSettings,
                               FeedForwardSettings driveMotorFeedForwardSettings,
                               FeedForwardSettings turnMotorFeedForwardSettings, CANcoder cancoder) {
        super(namespaceName, driveMotor, turnMotor, driveMotorInverted, turnMotorInverted, absoluteEncoderOffset,
                driveMotorPIDSettings, turnMotorPIDSettings, driveMotorFeedForwardSettings,
                turnMotorFeedForwardSettings);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.cancoder = cancoder;
    }

    @Override
    protected void configureDriveController() {
        driveMotor.setEncoderConversionFactor(DRIVE_GEAR_RATIO * WHEAL_DIAMETER_TO_INCHES *
                INCHES_TO_METERS * Math.PI);
    }

    @Override
    protected void configureTurnController() {
        turnMotor.setPositionConversionFactor(TURN_GEAR_RATIO * DEGREES_IN_ROTATION);
        turnMotor.setVelocityConversionFactor((TURN_GEAR_RATIO * DEGREES_IN_ROTATION) / SECONDS_IN_MINUTE);
    }

    @Override
    protected void configureAbsoluteEncoder() {
        MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs().
                withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_POSITION_DISCONTINUITY_POINT).
                withSensorDirection(turnMotorInverted ? SensorDirectionValue.Clockwise_Positive :
                        SensorDirectionValue.CounterClockwise_Positive).withMagnetOffset(absoluteEncoderOffset);
        cancoder.getConfigurator().apply(magnetSensorConfigs);
    }

    @Override
    protected Rotation2d getAbsoluteModuleAngle() {
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValueAsDouble() * DEGREES_IN_ROTATION);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute encoder", getAbsoluteModuleAngle()::getDegrees);
        namespace.putNumber("relative angle", this::getRelativeModuleAngle);
        namespace.putNumber("current velocity", driveMotor::getVelocity);



    }
}
