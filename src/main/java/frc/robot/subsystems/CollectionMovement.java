package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class CollectionMovement extends MotoredGenericSubsystem {

    private final static String namespaceName = "collection movement";

    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;
    private final TalonFXWrapper motor;

    private static CollectionMovement instance;

    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement(namespaceName,
                    new DigitalInput(RobotMap.DIO.COLLECTION_MOVEMENT_TOP_LIMIT_SWITCH),
                    new DigitalInput(RobotMap.DIO.COLLECTION_MOVEMENT_BOTTOM_LIMIT_SWITCH),
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX_ID));
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, DigitalInput topLimit, DigitalInput bottomLimit,
                               TalonFXWrapper motor) {
        super(namespaceName, motor);
        this.topLimit = topLimit;
        this.bottomLimit = bottomLimit;
        this.motor = motor;
    }

    @Override
    public boolean canMove(double speed) {
        return ((!bottomLimit.get() && speed > 0) || (!topLimit.get() && speed < 0));
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("top limit input", topLimit::get);
        namespace.putBoolean("bottom limit switch", bottomLimit::get);
        namespace.putNumber("motor current speed", motor::get);
    }
}
