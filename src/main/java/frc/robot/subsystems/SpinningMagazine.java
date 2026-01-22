package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class SpinningMagazine extends MotoredGenericSubsystem {

    private final static String NAMESPACE_NAME = "spinning magazine";

    private final TalonFXWrapper motor;

    private SpinningMagazine(TalonFXWrapper motor) {
        super(NAMESPACE_NAME, motor);
        this.motor = motor;
    }

    private static SpinningMagazine instance;

    public static SpinningMagazine getInstance() {
        if (instance == null) {
            instance = new SpinningMagazine(new TalonFXWrapper(RobotMap.CAN.SPINNING_MAGAZINE_TALON_FX_ID));
        }
        return instance;
    }

    //@TODO ask what causes it to start/stop and add it
    @Override
    public boolean canMove(double speed) {
        return super.canMove(speed);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("speed", motor::getVelocity);
    }

}