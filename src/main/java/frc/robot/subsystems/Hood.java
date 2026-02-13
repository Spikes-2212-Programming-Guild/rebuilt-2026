package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SparkWrapper;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class Hood extends SmartMotorControllerGenericSubsystem {

    public enum HoodPose {

        MAX_ANGLE(-1), MIN_ANGLE(0),
        POSE1(-1), POSE2(-1), POSE3(-1);

        public final double neededAngle;

        HoodPose(double neededAngle) {
            this.neededAngle = neededAngle;
        }
    }

    private static final String NAMESPACE_NAME = "hood";

    private static final double DEGREES_IN_ROTATION = 360;
    private static final double SECONDS_IN_MINUTE = 60;
    private static final double GEAR_RATIO = -1.0;
    private static final double DISTANCE_PER_PULSE = GEAR_RATIO * DEGREES_IN_ROTATION;

    private static final double CURRENT_LIMIT_AMP = 40;

    private static final double MOTION_EPSILON = -1.0;     // Minimum degrees change to be considered "moving"
    private static final double STALL_TIME_LIMIT = -1.0;   // Seconds to wait before triggering stall protection

    private final SparkWrapper sparkMax;
    private final AnalogPotentiometer absoluteEncoder;

    private double lastPositionDegrees = 0;
    private double lastMoveTime = 0;
    private boolean isStalled = false;

    private static Hood instance;

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood(NAMESPACE_NAME,
                    SparkWrapper.createSparkMax(RobotMap.CAN.HOOD_SPARK_MAX, SparkLowLevel.MotorType.kBrushless),
                    new AnalogPotentiometer(RobotMap.DIO.HOOD_ABSOLUTE_ENCODER, DEGREES_IN_ROTATION, 0));
        }
        return instance;
    }

    private Hood(String namespaceName, SparkWrapper sparkMax, AnalogPotentiometer absoluteEncoder) {
        super(namespaceName, sparkMax);
        this.sparkMax = sparkMax;
        this.absoluteEncoder = absoluteEncoder;

        configureHood();
        configureDashboard();
    }

    private void configureHood() {
        sparkMax.applyConfiguration(sparkMax.getSparkConfiguration().secondaryCurrentLimit(CURRENT_LIMIT_AMP));
        sparkMax.setVelocityConversionFactor(DISTANCE_PER_PULSE / SECONDS_IN_MINUTE);
        sparkMax.setPositionConversionFactor(DISTANCE_PER_PULSE);
    }

    public double getAbsDegrees() {
        return absoluteEncoder.get();
    }

    @Override
    public void periodic() {
        super.periodic();
        checkForStall();
    }

    /**
     * Checks if the encoder is changing while the motor is supposed to be moving.
     * Resets the timer if movement exceeds MOTION_EPSILON, indicating healthy operation.
     * Flags the motor as stalled if position remains static for longer than STALL_TIME_LIMIT.
     */
    private void checkForStall() {
        double currentPos = getAbsDegrees();
        double currentTime = Timer.getFPGATimestamp();

        double deltaPos = Math.abs(currentPos - lastPositionDegrees);

        if (deltaPos > MOTION_EPSILON) {
            lastMoveTime = currentTime;
            lastPositionDegrees = currentPos;
            isStalled = false;
        }

        else {
            if (currentTime - lastMoveTime > STALL_TIME_LIMIT) {
                isStalled = true;
            }
        }
    }

    @Override
    public boolean canMove(double speed) {
        return !isStalled;
    }

    public void calibrateEncoderPosition() {
        sparkMax.setPosition(getAbsDegrees());
    }

    public void configureDashboard() {
        namespace.putNumber("hood pose", sparkMax::getPosition);
        namespace.putNumber("abs encoder degrees", this::getAbsDegrees);
        namespace.putBoolean("is stalled", () -> isStalled);
    }
}
