package subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.RobotMap.DIO.BOTTOM_LIMIT_SWITCH_ID;
import static frc.robot.RobotMap.DIO.TOP_LIMIT_SWITCH_ID;

public class collectionMovement extends SmartMotorControllerGenericSubsystem {
    private TalonFXWrapper motor;
    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;

    public collectionMovement(String namespaceName, TalonFXWrapper motor
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

    public void stopAtTopLimitSwitch(){
        if(topLimitSwitch.get()){
            motor.stopMotor();
        }
    }

    public void stopAtBottomLimitSwitch(){
        if(bottomLimitSwitch.get()){
            motor.stopMotor();
        }
    }
}
