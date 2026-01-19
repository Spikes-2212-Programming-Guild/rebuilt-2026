package subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.RobotMap.DIO.BOTTOM_LIMIT_SWITCH_ID;
import static frc.robot.RobotMap.DIO.TOP_LIMIT_SWITCH_ID;

public class CollectionMovement extends SmartMotorControllerGenericSubsystem {
    private TalonFXWrapper motor;
    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;

    public CollectionMovement(String namespaceName, TalonFXWrapper motor
            , DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch
            , SmartMotorController... motorControllers) {
        super(namespaceName, motorControllers);
        this.motor = motor;
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
    }

    public void configLimitSwitches(){
        topLimitSwitch = new DigitalInput(TOP_LIMIT_SWITCH_ID);
        bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_SWITCH_ID);
    }

    public boolean isAtTopLimitSwitch(){
        return topLimitSwitch.get();
    }

    public boolean isAtBottomLimitSwitch(){
        return bottomLimitSwitch.get();
    }
}
