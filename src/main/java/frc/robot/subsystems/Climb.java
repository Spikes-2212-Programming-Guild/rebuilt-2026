package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import frc.robot.RobotMap;

public class Climb extends SmartMotorControllerGenericSubsystem {

    public enum ArmPose {

        TOP(-1), BOTTOM(-1);

        private final double positionMeters;

        public double getPositionMeters() {
            return positionMeters;
        }

        ArmPose(double position) {
            this.positionMeters = position;
        }
    }

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
            (speed > 0 && leftSparkMax.getPosition() != MAX_ARM_POSITION_METERS) ||
            (speed < 0 && leftSparkMax.getPosition() != MIN_ARM_POSITION_METERS)
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
