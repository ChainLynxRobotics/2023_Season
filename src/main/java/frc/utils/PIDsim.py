import numpy as np
from matplotlib import pyplot as plt 
from tabulate import tabulate as table

#output: table of system response vs time, graphs of system response and control output vs time (3 entry horizontal table)
#send to jairen

class const:
    kp = 0.1
    ki = 0.0001
    kd = 1
    kf = 0.01
    iZone = 1e-4
    iState = 0
    prev_err = 0
    kMinOutput = -1
    kMaxOutput = 1
    setpoint = 2
    dt = 0.02
    drag = 0.2
    mass = 0.6

def pid_run(setpoint:float, current) -> float:
    error = setpoint-current

    p  = error*const.kp

    if np.abs(error) <= const.iZone or const.iZone == 0:
        iState += error*const.ki
    else:
        iState = 0
   
    d = (error - const.prev_err)*const.kd
    const.prev_err = error
    f = setpoint*const.kf

    output = p + iState + d + f
    output = np.maximum(np.minimum(const.kMaxOutput, output), const.kMinOutput)
    return output


class elevator_motor:
    def __init__(self, pos, vel, drag_coeff, mass, t):
        self.pos = pos
        self.vel = vel
        self.t = t
        self.drag_coeff = drag_coeff
        self.mass = mass

    def next_vel(self, func:float, dt:float):
        self.vel += func
        self.t += dt

    def next_pos(self, func:float, dt:float):
        self.pos += func
        self.t += dt

    #call in main graphing loop to update pid
    def actuate(self, dt):
       net_force = pid_run(const.setpoint, self.pos) - self.drag_coeff*np.absolute(np.power(self.vel, 2))
       self.vel = self.next_vel(net_force/self.mass, dt)
       self.pos = self.next_pos(self.vel, dt)
       self.t += dt

        


#graph controller output about setpoint
def main():
    motor = elevator_motor(0, 0, const.drag, const.mass, 0)


    plt.title("pid simulation") 
    plt.xlabel("time") 
    plt.ylabel("position") 

    #for table
    columns = ["time", "position", "controller output"]
    data = [[0,0,0]]*500

    time = 0
    counter = 0
    while time < 10:
        motor.actuate(const.dt)
        pos = motor.pos
        eff = pid_run(const.setpoint, motor.pos)

        data[counter] = [time, pos, eff]

        plt.plot(time, motor.pos, "ro") #elevator position
        plt.plot(time, pid_run(const.setpoint, motor.pos), "bo") #motor output

        time += const.dt
        counter += 1

    plt.show()
    print(table(data, columns))

  
if __name__ == "__main__":
    main()