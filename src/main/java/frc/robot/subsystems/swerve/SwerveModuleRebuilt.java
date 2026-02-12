package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.com.spikes2212.command.drivetrains.swerve.SwerveModule;
import frc.robot.com.spikes2212.control.FeedForwardSettings;
import frc.robot.com.spikes2212.control.PIDSettings;
import frc.robot.com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;

public class SwerveModuleRebuilt extends SwerveModule {

    private static final double MIN_SPEED_LIMIT = 0;
    private static final double DRIVE_GEAR_RATIO = 1 / 6.12;
    private static final double TURN_GEAR_RATIO = (double) 7 / 150;
    private static final double WHEEL_DIAMETER_METERS = 0.1016;
    private static final int DEGREES_IN_ROTATION = 360;
    private static final int SECONDS_IN_MINUTE = 60;
    private static final int ABSOLUTE_POSITION_DISCONTINUITY_POINT = 1;

    private static final int DRIVE_CURRENT_LIMIT_AMP = 40;
    private static final int TURN_CURRENT_LIMIT_AMP = 40;

    private static final double DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS =
            DRIVE_GEAR_RATIO * WHEEL_DIAMETER_METERS * Math.PI;
    private static final double TURN_VELOCITY_IN_ROTATION =
            (TURN_GEAR_RATIO * DEGREES_IN_ROTATION) / SECONDS_IN_MINUTE;
    private static final double TURN_POSITION_IN_ROTATION =
            TURN_GEAR_RATIO * DEGREES_IN_ROTATION;

    private final TalonFXWrapper driveMotor;
    private final SparkWrapper turnMotor;
    private final CANcoder cancoder;

    public SwerveModuleRebuilt(String namespaceName, TalonFXWrapper driveMotor, SparkWrapper turnMotor,
                               boolean driveMotorInverted, boolean turnMotorInverted,
                               double absoluteEncoderOffset, PIDSettings driveMotorPIDSettings,
                               PIDSettings turnMotorPIDSettings, FeedForwardSettings driveMotorFeedForwardSettings,
                               FeedForwardSettings turnMotorFeedForwardSettings, CANcoder cancoder) {
        super(namespaceName, driveMotor, turnMotor, driveMotorInverted, turnMotorInverted, absoluteEncoderOffset,
                driveMotorPIDSettings, turnMotorPIDSettings, driveMotorFeedForwardSettings,
                turnMotorFeedForwardSettings, MIN_SPEED_LIMIT);
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.cancoder = cancoder;
        setCurrents();
    }

    @Override
    protected void configureDriveController() {
        driveMotor.setEncoderConversionFactor(DRIVE_MOTOR_ROTATION_TO_WHEEL_ROTATIONS);
    }

    @Override
    protected void configureTurnController() {
        turnMotor.setPositionConversionFactor(TURN_POSITION_IN_ROTATION);
        turnMotor.setVelocityConversionFactor(TURN_VELOCITY_IN_ROTATION);
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
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValue().in(Units.Degree));
    }

    public void setCurrents() {
        driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().
                withSupplyCurrentLimit(DRIVE_CURRENT_LIMIT_AMP));
        turnMotor.applyConfiguration(turnMotor.getSparkConfiguration().
                secondaryCurrentLimit(TURN_CURRENT_LIMIT_AMP));

    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute encoder", getAbsoluteModuleAngle()::getDegrees);
        namespace.putNumber("relative angle", this::getRelativeModuleAngle);
        namespace.putNumber("current drive velocity", driveMotor::getVelocity);
        namespace.putNumber("current turn velocity", turnMotor::getVelocity);

        namespace.putCommand("set angle to 0", new FunctionalCommand(() -> {
        },
                () -> setTargetState(
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        DrivetrainRebuilt.MAX_POSSIBLE_VELOCITY, false
                ), b -> stop(), () -> false));

        namespace.putCommand("drive at 0.2", new RunCommand(() -> driveMotor.set(0.2)) {
            @Override
            public void end(boolean interrupted) {
                driveMotor.stopMotor();
            }
        });

        namespace.putCommand("turn at 0.2", new RunCommand(() -> turnMotor.set(0.2)) {
            @Override
            public void end(boolean interrupted) {
                turnMotor.stopMotor();
            }
        });
    }
}
