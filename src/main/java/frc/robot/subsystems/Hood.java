package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class Hood extends SmartMotorControllerGenericSubsystem {

    public enum HoodPose {
        TOSS_POSE(-1), MAX_ANGLE(-1), MIN_ANGLE(-1);

        public final double neededAngle;

        HoodPose(double neededPitch) {
            this.neededAngle = neededPitch;
        }
    }

    private static final String NAMESPACE_NAME = "hood";
    private static final double GEAR_RATIO = -1.0;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DISTANCE_PER_PULSE = GEAR_RATIO * DEGREES_IN_ROTATIONS;

    private static final double MOTION_EPSILON = 1.0;     // Minimum degrees change to be considered "moving"
    private static final double STALL_TIME_LIMIT = 0.5;   // Seconds to wait before triggering stall protection
    private static final double MIN_SPEED_TO_CHECK = 0.1; // Minimum motor power to check for stall

    private final TalonFXWrapper talonFX;
    private final DigitalInput bottomLimit;
    private final DutyCycleEncoder absoluteEncoder;

    private double lastPositionDegrees = 0;
    private double lastMoveTime = 0;
    private boolean isStalled = false;

    private static Hood instance;

    public static Hood getInstance() {
        if (instance == null) {
            instance = new Hood(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.HOOD_MOTOR),
                    new DigitalInput(RobotMap.DIO.HOOD_BOTTOM_LIMIT),
                    new DutyCycleEncoder(RobotMap.DIO.HOOD_ABSOLUTE_ENCODER));
        }
        return instance;
    }

    private Hood(String namespaceName, TalonFXWrapper talonFX, DigitalInput bottomLimit, DutyCycleEncoder absoluteEncoder) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
        this.bottomLimit = bottomLimit;
        this.absoluteEncoder = absoluteEncoder;

        talonFX.setEncoderConversionFactor(DISTANCE_PER_PULSE);
        configureDashboard();
    }

    public double getAbsDegrees() {
        return absoluteEncoder.get() * 360.0;
    }

    @Override
    public void periodic() {
        super.periodic();
        checkForStall();
    }

    /**
     * Checks if the encoder is changing while the motor is supposed to be moving.
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
        return !((bottomLimit.get() && speed < 0) ||
                (Math.abs(speed) > MIN_SPEED_TO_CHECK && isStalled));
    }

    public void calibrateEncoderPosition() {
        if (bottomLimit.get()) {
            talonFX.setPosition(HoodPose.MIN_ANGLE.neededAngle);
        }
    }

    public void configureDashboard() {
        namespace.putBoolean("bottom limit", bottomLimit::get);
        namespace.putNumber("hood pose", talonFX::getPosition);
        namespace.putNumber("Abs Encoder Deg", this::getAbsDegrees);
        namespace.putBoolean("Is Stalled", () -> isStalled);
    }
}
