package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.SmartMotorController;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class SpinningMagazine extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "spinning magazine";

    public SpinningMagazine(String NAMESPACE_NAME, SmartMotorController motorController) {
        super(NAMESPACE_NAME, motorController);
    }

    private static SpinningMagazine instance;

    public static SpinningMagazine getInstance() {
        if (instance == null) {
            instance = new SpinningMagazine(NAMESPACE_NAME, new TalonFXWrapper(RobotMap.CAN.SPINNING_MAGAZINE_TALON_FX_ID));
        }
        return instance;
    }

    // will change in future when it will be decided on what causes it to start/stop
    @Override
    public boolean canMove(double speed) {
        return super.canMove(speed);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("speed", motorController::get);
    }

}