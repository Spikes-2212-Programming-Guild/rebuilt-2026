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

        CollectionMovementPose(double neededSpeed) {
            this.neededSpeed = neededSpeed;
        }

        public double getNeededSpeed() {
            return this.neededSpeed;
        }
    }

    private static final String NAMESPACE_NAME = "collection movement";
    private static final double MOTION_EPSILON = -1.0;
    private static final double STALL_TIME_LIMIT = -1.0;   // Seconds to wait before triggering stall protection// Minimum degrees change to be considered "moving"
    private static CollectionMovement instance;
    private final DutyCycleEncoder absoluteEncoder;
    private final TalonFXWrapper talonFX;
    private double lastPositionDegrees = 0;
    private double lastMoveTime = 0;
    private boolean isStalled = false;


    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement(NAMESPACE_NAME,
                    new DutyCycleEncoder(RobotMap.DIO.COLLECTION_MOVEMENT_THROUGH_BORE_ID),
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX_ID));
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, DutyCycleEncoder absoluteEncoder,
                               TalonFXWrapper talonFX) {
        super(namespaceName, talonFX);
        this.talonFX = talonFX;
        this.absoluteEncoder = absoluteEncoder;
        configureDashboard();
    }

    public double getAbsDegrees() {
        return absoluteEncoder.get();
    }

    private void checkForStall() {
        double currentPos = getAbsDegrees();
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
        return speed != 0 || !isStalled;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("talon relative encoder position", talonFX::getPosition);
        namespace.putNumber("motor current speed", talonFX::getVelocity);
    }
}
