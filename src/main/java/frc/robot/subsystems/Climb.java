package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.RobotMap;

public class Climb extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "climb";

    private static final boolean RIGHT_MOTOR_INVERTED = false;

    private static final double MAX_ARM_POSITION_METERS = -1;
    private static final double MIN_ARM_POSITION_METERS = -1;

    private static final double GEAR_RATIO = -1;
    private static final double SPOOL_DIAMETER_METERS = -1;

    private final SparkWrapper leftSparkMax;
    private final SparkWrapper rightSparkMax;

    private static Climb instance;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(
                            RobotMap.CAN.CLIMB_SPARK_MAX_LEFT_ID, SparkLowLevel.MotorType.kBrushless),
                    SparkWrapper.createSparkMax(
                            RobotMap.CAN.CLIMB_SPARK_MAX_RIGHT_ID, SparkLowLevel.MotorType.kBrushless),
                    RIGHT_MOTOR_INVERTED
            );
        }
        return instance;
    }

    private Climb(String namespaceName, SparkWrapper leftSparkMax,
                  SparkWrapper rightSparkMax, boolean rightMotorInverted) {
        super(namespaceName, leftSparkMax, rightSparkMax);
        this.leftSparkMax = leftSparkMax;
        this.rightSparkMax = rightSparkMax;
        //@TODO maybe set the position to 0 at the start or add a "reset" command that does it
        leftSparkMax.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        rightSparkMax.setIdleMode(SparkBaseConfig.IdleMode.kBrake);
        rightSparkMax.setInverted(rightMotorInverted);
        configureRelativeEncoder();
        configureDashboard();
    }

    private void configureRelativeEncoder() {
        double spoolCircumference = SPOOL_DIAMETER_METERS * Math.PI;
        leftSparkMax.setPositionConversionFactor(spoolCircumference * GEAR_RATIO);
    }

    @Override
    public boolean canMove(double speed) {
        return (
                (speed > 0 && leftSparkMax.getPosition() < MAX_ARM_POSITION_METERS) ||
                        (speed < 0 && leftSparkMax.getPosition() > MIN_ARM_POSITION_METERS)
        );
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("left position", leftSparkMax::getPosition);
        namespace.putNumber("right position", rightSparkMax::getPosition);

        namespace.putNumber("left velocity", leftSparkMax::getVelocity);
        namespace.putNumber("right velocity", rightSparkMax::getVelocity);
    }
}
