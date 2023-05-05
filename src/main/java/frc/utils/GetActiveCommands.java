package frc.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GetActiveCommands extends ICommandScheduler<GetActiveCommands> implements AutoCloseable, NTSendable {
    private CommandScheduler scheduler = CommandScheduler.getInstance();
    private GetActiveCommands commands;

    private final List<Consumer<Command>> m_initActions = new ArrayList<>();
    private final List<Consumer<Command>> m_executeActions = new ArrayList<>();
    private final List<Consumer<Command>> m_interruptActions = new ArrayList<>();
    private final List<Consumer<Command>> m_finishActions = new ArrayList<>();
    //on instantiation, terminate any live window commands and 
    public GetActiveCommands() {
        
    }

    @Override
    protected GetActiveCommands createObject() {
        if (commands == null) {
            commands = new GetActiveCommands();
        }
        return commands;
    }

    @Override
    public void setPeriod(double time) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPeriod'");
    }


    @Override
    protected List<EventLoop> getButtonLoops() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getButtonLoops'");
    }


    @Override
    protected void setButtonLoops() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setButtonLoops'");
    }


    @Override
    public void initializeCommands(Command command, Set<Subsystem> requirements) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'initializeCommands'");
    }


    @Override
    public void scheduleCommand(Command command) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'scheduleCommand'");
    }


    @Override
    public void scheduleCommands(Command... command) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'scheduleCommands'");
    }


    @Override
    public void run() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'run'");
    }


    @Override
    public void setDefaultCommand(Command command) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDefaultCommand'");
    }


    @Override
    public void getDefaultCommand(Command command) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDefaultCommand'");
    }


    @Override
    public void removeDefaultCommand(Command command) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'removeDefaultCommand'");
    }


    @Override
    public void registerSubsystems(Subsystem... subsystems) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'registerSubsystems'");
    }


    @Override
    public void unregisterSubsystems(Subsystem... subsystems) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'unregisterSubsystems'");
    }


    @Override
    public void cancel() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'cancel'");
    }


    @Override
    public void cancelAll() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'cancelAll'");
    }


    @Override
    public boolean isScheduled(Command... commands) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isScheduled'");
    }


    @Override
    public Command commandsRequiring(Subsystem subsystem) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'commandsRequiring'");
    }


    @Override
    public void enableScheduler() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enableScheduler'");
    }


    @Override
    public void disableScheduler() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'disableScheduler'");
    }


    @Override
    public void onCommandInitialize() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'onCommandInitialize'");
    }


    @Override
    public void onCommandExecute() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'onCommandExecute'");
    }


    @Override
    public void onCommandInterrupt() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'onCommandInterrupt'");
    }


    @Override
    public void onCommandFinish() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'onCommandFinish'");
    }


    @Override
    public void registerComposedCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'registerComposedCommands'");
    }


    @Override
    public void clearComposedCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'clearComposedCommands'");
    }


    @Override
    public boolean isComposed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isComposed'");
    }


    @Override
    public Set<Command> getComposedCommands() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getComposedCommands'");
    }

      
    @Override
    public void close() throws Exception {
        SendableRegistry.remove(scheduler);
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        scheduler.initSendable(builder);
    }
}
