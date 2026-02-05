
import robot_interface as MiniDriver


class Driver():
    
    def __init__(self) :
       
        self.udp = MiniDriver.UDP(0xee, 8080, "192.168.123.161", 8082)
        self.cmd = MiniDriver.HighCmd()
        self.udp.InitCmdData(self.cmd)

    def Move(self, XDot,YDot,ThetaDot):

        self.udp.Recv()
        self.udp.GetRecv(MiniDriver.HighState())
        
        self.cmd.mode = 2
        self.cmd.gaitType = 0
        self.cmd.velocity = [XDot, YDot]
        self.cmd.yawSpeed = ThetaDot
        self.cmd.footRaiseHeight = 0

        self.udp.SetSend(self.cmd)
        self.udp.Send()


if __name__ == "__main__":
    pass
    # Robot = Driver()

    # # while True:
    # Robot.Move(0.5,0,0)
