package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;

public class CollectionMovement extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "collection movement";

    private static final double ABSOLUTE_UPPER_DEG = -1;
    private static final double ABSOLUTE_LOWER_DEG = -1;
    private static final double GEAR_RATIO = -1;
    private static final double ROTATIONS_TO_DEG = 360;

    private final DutyCycleEncoder throughBore;
    private final TalonFXWrapper motor;

    private static CollectionMovement instance;

    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement(NAMESPACE_NAME,
                    new DutyCycleEncoder(RobotMap.DIO.COLLECTION_MOVEMENT_THROUGH_BORE_ID),
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX_ID));
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, DutyCycleEncoder throughBore,
                               TalonFXWrapper motor) {
        super(namespaceName, motor);
        this.throughBore = throughBore;
        this.motor = motor;
        configureDashboard();
    }

    public void syncEncoder() {
        motor.setPosition(throughBore.get());
    }

    @Override
    public boolean canMove(double speed) {
        return ((speed < 0 && throughBore.get() * ROTATIONS_TO_DEG == ABSOLUTE_UPPER_DEG) ||
                (speed > 0 && throughBore.get() * ROTATIONS_TO_DEG == ABSOLUTE_LOWER_DEG));
    }

    public double encoderPositionToDeg(){
        return throughBore.get() * ROTATIONS_TO_DEG;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("talon relative encoder position", motor::getPosition);
        namespace.putNumber("through bore position", this::encoderPositionToDeg);
        namespace.putNumber("motor current speed", motor::getVelocity);
    }
}
