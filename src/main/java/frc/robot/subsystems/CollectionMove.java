package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import static frc.robot.RobotMap.CAN.TALON_COLLECTION_MOVEMENT_ID;
import static frc.robot.RobotMap.DIO.BOTTOM_LIMIT_SWITCH_ID;
import static frc.robot.RobotMap.DIO.TOP_LIMIT_SWITCH_ID;

public class CollectionMove extends MotoredGenericSubsystem {

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;
    private static CollectionMove instance;

    public static CollectionMove getInstance(){
        if(instance == null){
            instance = new CollectionMove("collection movement",
                    new DigitalInput(TOP_LIMIT_SWITCH_ID),
                    new DigitalInput(BOTTOM_LIMIT_SWITCH_ID),
                    new TalonFXWrapper(TALON_COLLECTION_MOVEMENT_ID));
        }
        return instance;
    }

    private CollectionMove(String namespaceName, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch,
                          MotorController... motorControllers) {
        super(namespaceName, motorControllers);
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
    }

    @Override
    public boolean canMove(double speed) {
        if (!topLimitSwitch.get() && !bottomLimitSwitch.get()) {
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        super.stop();
    }
}
