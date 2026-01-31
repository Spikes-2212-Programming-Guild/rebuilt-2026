package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class CollectionMovement extends MotoredGenericSubsystem {

    public enum CollectionMovementPose {

        MAX_POSITION(-1), MIN_POSITION(-1);

        private final double neededSpeed;

        public double getNeededSpeed() {
            return this.neededSpeed;
        }

        CollectionMovementPose(double neededSpeed) {
            this.neededSpeed = neededSpeed;
        }
    }

    private static final String NAMESPACE_NAME = "collection movement";

    private static final boolean IS_ENCODER_INVERTED = false;

    private static final double ABSOLUTE_UPPER_DEG = -1;
    private static final double ABSOLUTE_LOWER_DEG = -1;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double MOTION_EPSILON = -1.0;
    private static final double STALL_TIME_LIMIT = -1.0;   // Seconds to wait before triggering stall protection// Minimum degrees change to be considered "moving"

    private final DutyCycleEncoder absoluteEncoder;
    private final TalonFXWrapper talonFX;
    private static DigitalInput bottomLimit;
    private double lastPositionDegrees = 0;
    private double lastMoveTime = 0;
    private boolean isStalled = false;


    private static CollectionMovement instance;

    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement(NAMESPACE_NAME,
                    new DutyCycleEncoder(RobotMap.DIO.COLLECTION_MOVEMENT_THROUGH_BORE_ID),
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX_ID),
                    IS_ENCODER_INVERTED,bottomLimit);
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, DutyCycleEncoder absoluteEncoder,
                               TalonFXWrapper talonFX, boolean isInverted, DigitalInput bottomLimit) {
        super(namespaceName, talonFX);
        this.absoluteEncoder = absoluteEncoder;
        this.bottomLimit = bottomLimit;
        this.absoluteEncoder.setInverted(isInverted);
        this.talonFX = talonFX;
        configureDashboard();
    }

    public double getAbsDegrees() {
        return absoluteEncoder.get();
    }

    public void resetRelativeEncoderRelativeToAbsoluteEncoder() {
        talonFX.setPosition(absoluteEncoder.get());
    }

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
        if (bottomLimit.get() && speed < 0) {
            return false;
        }

        return !(Math.abs(speed) > ABSOLUTE_LOWER_DEG && Math.abs(speed) > ABSOLUTE_UPPER_DEG) || !isStalled;
    }

        public double encoderPositionToDegrees () {
            return absoluteEncoder.get() * DEGREES_IN_ROTATIONS;
        }

        @Override
        public void configureDashboard () {
            namespace.putNumber("talon relative encoder position", talonFX::getPosition);
            namespace.putNumber("motor current speed", talonFX::getVelocity);
        }
    }
