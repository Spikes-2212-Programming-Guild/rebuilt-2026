package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.RobotMap.CAN.TALON_COLLECTION_MOVEMENT_ID;
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
        this.motor = new TalonFXWrapper(TALON_COLLECTION_MOVEMENT_ID);
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
        configLimitSwitches();
    }

    public void configLimitSwitches(){
        topLimitSwitch = new DigitalInput(TOP_LIMIT_SWITCH_ID);
        bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_SWITCH_ID);
    }

    @Override
    public boolean canMove(double speed) {
        if(!topLimitSwitch.get()){
            return true;
        } else if(!bottomLimitSwitch.get()){
            return true;
        }
        return false;
    }
}
