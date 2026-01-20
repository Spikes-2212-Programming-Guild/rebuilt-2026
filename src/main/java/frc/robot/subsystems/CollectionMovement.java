package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.RobotMap;

public class CollectionMovement extends MotoredGenericSubsystem {

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    private static CollectionMovement instance;

    public static CollectionMovement getInstance() {
        if (instance == null) {
            instance = new CollectionMovement("collection movement",
                    new DigitalInput(RobotMap.DIO.TOP_LIMIT_SWITCH),
                    new DigitalInput(RobotMap.DIO.BOTTOM_LIMIT_SWITCH),
                    new TalonFXWrapper(RobotMap.CAN.TALON_COLLECTION_MOVEMENT));
        }
        return instance;
    }

    private CollectionMovement(String namespaceName, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch,
                               MotorController... motorControllers) {
        super(namespaceName, motorControllers);
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
    }

    @Override
    public boolean canMove(double speed) {
        return !((bottomLimitSwitch.get() && speed < 0) || (topLimitSwitch.get() && speed > 0));
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("top limit input", topLimitSwitch::get);
        namespace.putBoolean("bottom limit switch", bottomLimitSwitch::get);
        namespace.putNumber("motor current speed", motorController::get);
    }
}
