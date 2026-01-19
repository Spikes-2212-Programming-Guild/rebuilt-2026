package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class CollectionMove extends MotoredGenericSubsystem {

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    public CollectionMove(String namespaceName, DigitalInput topLimitSwitch, DigitalInput bottomLimitSwitch,
                          MotorController... motorControllers) {
        super(namespaceName, motorControllers);
        this.topLimitSwitch = topLimitSwitch;
        this.bottomLimitSwitch = bottomLimitSwitch;
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
        super.stop();
    }
}
