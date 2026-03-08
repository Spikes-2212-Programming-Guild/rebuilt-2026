package frc.robot;

import com.spikes2212.util.PlaystationControllerWrapper;
import frc.robot.commands.advancedcommands.Collect;
import frc.robot.commands.advancedcommands.MoveCollectionUp;
import frc.robot.subsystems.forbar.Collection;
import frc.robot.subsystems.forbar.CollectionMovement;

public class OI /*GEVALD*/{

    private final PlaystationControllerWrapper driverPlaystation = new PlaystationControllerWrapper(0);
    private final PlaystationControllerWrapper navigatorPlaystation = new PlaystationControllerWrapper(1);

    private final Collection collection = Collection.getInstance();
    private final CollectionMovement collectionMovement = CollectionMovement.getInstance();


    public OI() {
        navigatorPlaystation.getL2Button().whileTrue(new Collect(collection, collectionMovement))
                .onFalse(new MoveCollectionUp(collection, collectionMovement));
        }

    public double getLeftX(){
        return driverPlaystation.getLeftX();
    }

    public double getLeftY(){
        return driverPlaystation.getLeftY();
    }

    public double getRightX(){
        return driverPlaystation.getRightX();
    }

    public double getRightY(){
        return driverPlaystation.getRightY();
    }
}
