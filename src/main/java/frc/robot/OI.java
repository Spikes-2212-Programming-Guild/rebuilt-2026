package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.advancedcommands.Collect;
import frc.robot.commands.simplecommands.MoveCollection;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;

public class OI {

    private final PlaystationControllerWrapper navigatorJoystick = new PlaystationControllerWrapper(0);
    private final Joystick leftJoystick = new Joystick(1);
    private final Joystick rightJoystick = new Joystick(2);

    private final CollectionMovement collectionMovement = CollectionMovement.getInstance();
    private final Collection collection = Collection.getInstance();

    public OI() {
        navigatorJoystick.getSquareButton().onTrue(new MoveCollection(collectionMovement, ()-> 0.05));
        navigatorJoystick.getCrossButton().onTrue(new Collect(collection, collectionMovement));
    }
}
