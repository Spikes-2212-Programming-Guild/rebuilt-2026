package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class CollectionMovement extends MotoredGenericSubsystem {

    private final static String namespaceName = "Collection movement";

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;
    private final TalonFXWrapper talonFX;

    private static CollectionMovement instance;

    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement(namespaceName,
                    new DigitalInput(RobotMap.DIO.COLLECTION_MOVEMENT_TOP_LIMIT_SWITCH),
                    new DigitalInput(RobotMap.DIO.COLLECTION_MOVEMENT_BOTTOM_LIMIT_SWITCH),
                    new TalonFXWrapper(RobotMap.CAN.COLLECTION_MOVEMENT_TALON_FX));
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch,
                               TalonFXWrapper talonFX) {
        super(namespaceName, talonFX);
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
        this.talonFX = talonFX;
    }

    @Override
    public boolean canMove(double speed) {
        return ((bottomLimitSwitch.get() && speed > 0) || (topLimitSwitch.get() && speed < 0));
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("top limit input", topLimitSwitch::get);
        namespace.putBoolean("bottom limit switch", bottomLimitSwitch::get);
        namespace.putNumber("motor current speed", talonFX::get);
    }
}
