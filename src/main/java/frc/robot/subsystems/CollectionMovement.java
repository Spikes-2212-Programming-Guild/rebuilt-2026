package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;

public class CollectionMovement extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "collection movement";
    private static final double ABSOLUTE_UPPER_DEG = -1;
    private static final double ABSOLUTE_LOWER_DEG = -1;

    private final DutyCycleEncoder throughBore;
    private final TalonFXWrapper motor;

    private static CollectionMovement instance;

    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement(NAMESPACE_NAME,
                    new DutyCycleEncoder(-1)
                    , new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX_ID));
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, DutyCycleEncoder throughBore,
                               TalonFXWrapper motor) {
        super(namespaceName, motor);
        this.throughBore = throughBore;
        this.motor = motor;
    }

    public void syncEncoder() {
        boolean upperSynced = false;
        boolean lowerSynced = false;
        if (throughBore.get() == ABSOLUTE_UPPER_DEG && upperSynced) {
            motor.setPosition(throughBore.get());
            upperSynced = true;
        } else if (throughBore.get() == ABSOLUTE_LOWER_DEG && lowerSynced) {
            motor.setPosition(throughBore.get());
            lowerSynced = true;
        }
    }

    @Override
    public boolean canMove(double speed) {
        return (throughBore.get() != ABSOLUTE_UPPER_DEG && throughBore.get() != ABSOLUTE_LOWER_DEG);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("through bore position", throughBore::get);
        namespace.putNumber("motor current speed", motor::getVelocity);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
