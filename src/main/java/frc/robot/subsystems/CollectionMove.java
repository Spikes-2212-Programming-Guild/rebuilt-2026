package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.RobotMap.DIO.BOTTOM_LIMIT_SWITCH_ID;
import static frc.robot.RobotMap.DIO.TOP_LIMIT_SWITCH_ID;

public class CollectionMove extends GenericSubsystem {
    private final TalonFXWrapper talonFX;
    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;

    public CollectionMove(String namespaceName, TalonFXWrapper talonFX, DigitalInput topLimitSwitch
            , DigitalInput bottomLimitSwitch) {
        super(namespaceName);
        this.talonFX = talonFX;
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
        talonFX.stopMotor();
    }

    @Override
    public void configureDashboard() {}
}
