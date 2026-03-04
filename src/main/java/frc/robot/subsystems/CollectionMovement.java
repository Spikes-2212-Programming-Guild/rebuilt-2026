package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;

public class CollectionMovement extends SmartMotorControllerGenericSubsystem {

    public enum CollectionMovementPose {

        MAX_POSE(-1), MIN_POSE(-1), OPEN_POSE(-1), CLOSE_POSE(-1);

        public final double neededPose;

        CollectionMovementPose(double neededPose) {
            this.neededPose = neededPose;
        }
    }

    private static final String NAMESPACE_NAME = "collection movement";

    private static final double CURRENT_LIMIT_AMP = 40;
    private static final double MOTION_EPSILON = -1.0;     // Minimum degrees change to be considered "moving"
    private static final double STALL_TIME_LIMIT = -1.0;   // Seconds to wait before triggering stall protection
    private static final double GEAR_RATIO = 0.2;
    private static final double DEGREES_IN_ROTATIONS = 360;
    private static final double DISTANCE_PER_PULSE = GEAR_RATIO * DEGREES_IN_ROTATIONS;

    private final TalonFXWrapper talonFX;

    private double lastPositionDegrees = 0;
    private double lastMoveTime = 0;
    private boolean isStalled = false;

    private static CollectionMovement instance;

    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX_ID, new CANBus("canivore")));
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, TalonFXWrapper talonFX) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;

        talonFX.setEncoderConversionFactor(DISTANCE_PER_PULSE);
        talonFX.getConfigurator().apply(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(CURRENT_LIMIT_AMP));
        configureDashboard();
    }

    @Override
    public void periodic() {
        super.periodic();
        checkForStall();
    }

    private void checkForStall() {
        double currentPos = talonFX.getPosition();
        double currentTime = Timer.getFPGATimestamp();

        double deltaPos = Math.abs(currentPos - lastPositionDegrees);

        if (deltaPos > MOTION_EPSILON) {
            lastMoveTime = currentTime;
            lastPositionDegrees = currentPos;
            isStalled = false;
        } else {
            if (currentTime - lastMoveTime > STALL_TIME_LIMIT) {
                isStalled = true;
            }
        }
    }

    @Override
    public boolean canMove(double speed) {
        return (talonFX.getPosition() > CollectionMovementPose.OPEN_POSE.neededPose && speed < 0)
                || (talonFX.getPosition() < CollectionMovementPose.CLOSE_POSE.neededPose && speed > 0)
                || !isStalled;
    }

    public void calibrateEncoderPosition(double speed) {
        if(isStalled && speed < 0) {
            talonFX.setPosition(CollectionMovementPose.MIN_POSE.neededPose);
        } else if(isStalled && speed > 0) {
            talonFX.setPosition(CollectionMovementPose.MAX_POSE.neededPose);
        }
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("velocity", talonFX::getVelocity);
        namespace.putNumber("relative", talonFX::getPosition);
    }
}
