package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.RobotMap.CAN.TALON_COLLECTION_MOVEMENT_ID;
import static frc.robot.RobotMap.DIO.BOTTOM_LIMIT_SWITCH_ID;
import static frc.robot.RobotMap.DIO.TOP_LIMIT_SWITCH_ID;

public class CollectionMove extends GenericSubsystem {
    private TalonFXWrapper motor;
    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;

    public CollectionMove(String namespaceName, TalonFXWrapper motor, DigitalInput topLimitSwitch
            , DigitalInput bottomLimitSwitch) {
        super(namespaceName);
        this.motor = motor;
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
    }

    public void configLimitSwitches(){
        topLimitSwitch = new DigitalInput(TOP_LIMIT_SWITCH_ID);
        bottomLimitSwitch = new DigitalInput(BOTTOM_LIMIT_SWITCH_ID);
    }

    @Override
    protected void apply(double v) {}

    @Override
    public boolean canMove(double speed) {
        if(!topLimitSwitch.get()){
            return true;
        } else if(!bottomLimitSwitch.get()){
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void configureDashboard() {}
}
