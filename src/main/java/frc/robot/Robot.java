// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.advancedcommands.Collect;
import frc.robot.commands.simplecommands.MoveCollection;
import frc.robot.commands.simplecommands.SimpleSpin;
import frc.robot.subsystems.Collection;
import frc.robot.subsystems.CollectionMovement;
import frc.robot.subsystems.SpinningMagazine;

import javax.swing.*;

public class Robot extends TimedRobot {

    private final RootNamespace namespace = new RootNamespace("robot");
    //private final Hood hood = Hood.getInstance();
    private final CollectionMovement collectionMovement = CollectionMovement.getInstance();
    private final Collection collection = Collection.getInstance();
    private final SpinningMagazine spinningMagazine = SpinningMagazine.getInstance();

    @Override
    public void robotInit() {

        namespace.putCommand("move collection down", new MoveCollection(collectionMovement, ()-> 0.1));
        namespace.putCommand("move collection up", new MoveCollection(collectionMovement, ()-> -0.2));
        namespace.putCommand("collect", new Collect(collection, collectionMovement));
        namespace.putCommand("slow intake", new MoveGenericSubsystem(collection, ()-> 0.3));
        namespace.putCommand("spindexer", new SimpleSpin(spinningMagazine));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        collectionMovement.calibrateEncoderPosition(collectionMovement.getSpeed());
        namespace.update();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
