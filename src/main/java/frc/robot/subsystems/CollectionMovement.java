package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    private final DutyCycleEncoder throughBore;
    private final TalonFXWrapper talonFX;

    private static CollectionMovement instance;

    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement(NAMESPACE_NAME,
                    new DutyCycleEncoder(RobotMap.DIO.COLLECTION_MOVEMENT_THROUGH_BORE_ID),
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX_ID),
                    IS_ENCODER_INVERTED);
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, DutyCycleEncoder throughBore,
                               TalonFXWrapper talonFX, boolean isInverted) {
        super(namespaceName, talonFX);
        this.throughBore = throughBore;
        this.throughBore.setInverted(isInverted);
        this.talonFX = talonFX;
        configureDashboard();
    }

    public void resetRelativeEncoderRelativeToAbsoluteEncoder() {
        talonFX.setPosition(throughBore.get());
    }

    @Override
    public boolean canMove(double speed) {
        if (!(speed < 0 && encoderPositionToDegrees() == ABSOLUTE_UPPER_DEG)) {
            return true;
        } else return !(speed > 0 && encoderPositionToDegrees() == ABSOLUTE_LOWER_DEG);
    }

    public double encoderPositionToDegrees() {
        return throughBore.get() * DEGREES_IN_ROTATIONS;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("talon relative encoder position", talonFX::getPosition);
        namespace.putNumber("through bore position", this::encoderPositionToDegrees);
        namespace.putNumber("motor current speed", talonFX::getVelocity);
    }
}
