package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;

public class CollectionMove extends GenericSubsystem {
    private final TalonFXWrapper talonFX;
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    public CollectionMove(String namespaceName, TalonFXWrapper talonFX, DigitalInput topLimitSwitch
            , DigitalInput bottomLimitSwitch) {
        super(namespaceName);
        this.talonFX = talonFX;
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
    }

    @Override
    protected void apply(double v) {
    }

    @Override
    public boolean canMove(double speed) {
        if (!topLimitSwitch.get()) {
            return true;
        } else if (!bottomLimitSwitch.get()) {
            return true;
        }
        return false;
    }

    @Override
    public void stop() {
        talonFX.stopMotor();
    }

    @Override
    public void configureDashboard() {
    }
}
