package frc.utils;

import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class ICommandScheduler<Impl extends ICommandScheduler<Impl>> {

    private Impl impl;

    public ICommandScheduler() {
        this.impl = createObject();
    }

    protected abstract Impl createObject();

    @SuppressWarnings("unchecked")
    public synchronized Impl getInstance() {
        return (Impl)this;
    }

    public abstract void setPeriod(double time);

    protected abstract List<EventLoop> getButtonLoops();

    protected abstract void setButtonLoops();

    public abstract void initializeCommands(Command command, Set<Subsystem> requirements);

    public abstract void scheduleCommand(Command command);

    public abstract void scheduleCommands(Command... command);

    public abstract void setDefaultCommand(Command command);

    public abstract void getDefaultCommand(Command command);

    public abstract void removeDefaultCommand(Command command);

    public abstract void run();

    public abstract void registerSubsystems(Subsystem... subsystems);

    public abstract void unregisterSubsystems(Subsystem... subsystems);

    public abstract void cancel();

    public abstract void cancelAll();

    public abstract boolean isScheduled(Command... commands);

    public abstract Command commandsRequiring(Subsystem subsystem);

    public abstract void enableScheduler();

    public abstract void disableScheduler();

    public abstract void onCommandInitialize();

    public abstract void onCommandExecute();

    public abstract void onCommandInterrupt();

    public abstract void onCommandFinish();

    public abstract void registerComposedCommands();

    public abstract void clearComposedCommands();

    public abstract boolean isComposed();

    public abstract Set<Command> getComposedCommands();
}
