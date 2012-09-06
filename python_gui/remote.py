#! /usr/bin/env python

from Tkinter import *
from serial import *

class Remote(Frame):
    def __init__(self, master=None):
        Frame.__init__(self, master)
        self.pack()

       # Basic Layout
        topPart = Frame(root)
        topPart.pack(fill="both", expand="yes", side="top")

        bottomPart = Frame(root)
        bottomPart.pack(fill="both", expand="yes", side="top")

        leftTop = LabelFrame(topPart, text="Main Control")
        leftTop.pack(fill="both", expand="yes", side="left")

        rightTop = LabelFrame(topPart, text="Motor Control")
        rightTop.pack(fill="both", expand="no", side="right", ipadx=10, ipady=10)

        leftBottom = LabelFrame(bottomPart, text="Debug Information")
        leftBottom.pack(fill="both", expand="no", side="left")

        leftBottomTop = Frame(leftBottom)
        leftBottomTop.pack(fill="both", expand="no", side="top")

        leftBottomBottom = Frame(leftBottom)
        leftBottomBottom.pack(fill="both", expand="yes", side="bottom")
        
        leftBottomTopLeft = Frame(leftBottomTop, padx=5)
        leftBottomTopLeft.pack(fill="both", expand="yes", side="left")

        leftBottomTopRight = Frame(leftBottomTop, padx=5)
        leftBottomTopRight.pack(fill="both", expand="yes", side="right")

        leftBottomBottomLeft = LabelFrame(leftBottomBottom, text="IMU")
        leftBottomBottomLeft.pack(fill="both", expand="yes", side="left")

        leftBottomBottomRight = LabelFrame(leftBottomBottom, text="PID")
        leftBottomBottomRight.pack(fill="both", expand="yes", side="right")

        rightBottom = LabelFrame(bottomPart, text="Serial Output")
        rightBottom.pack(fill="both", expand="yes", side="right")

        xIMUDbgFrame = Frame(leftBottomBottomLeft)
        xIMUDbgFrame.pack(fill="both", expand="yes", side="left")

        yIMUDbgFrame = Frame(leftBottomBottomLeft)
        yIMUDbgFrame.pack(fill="both", expand="yes", side="right")

        xPIDDbgFrame = Frame(leftBottomBottomRight)
        xPIDDbgFrame.pack(fill="both", expand="yes", side="left")

        yPIDDbgFrame = Frame(leftBottomBottomRight)
        yPIDDbgFrame.pack(fill="both", expand="yes", side="right")

        rightBottomTop = Frame(rightBottom, padx=5)
        rightBottomTop.pack(fill="x", side="top")

        rightBottomBottom = Frame(rightBottom, padx=5, pady=5)
        rightBottomBottom.pack(fill="both", expand="yes", side="bottom")   

        rightTopTop = Frame(rightTop)
        rightTopTop.pack(fill="x", expand="no", side="top", padx=5, pady=5)

        rightTopBottom = Frame(rightTop)
        rightTopBottom.pack(fill="both", expand="yes", side="bottom", padx=5, pady=5)

        leftTopLeft = Frame(leftTop)
        leftTopLeft.pack(fill="both", expand="yes", side="left", padx=5)

        leftTopRight = Frame(leftTop)
        leftTopRight.pack(fill="both", expand="no", side="right", padx=10)

        leftTopLeftTop = Frame(leftTopLeft)
        leftTopLeftTop.pack(fill="x", expand="no", side="top")

        leftTopLeftBottom = Frame(leftTopLeft)
        leftTopLeftBottom.pack(fill="both", expand="yes", padx=20, pady=20)

        leftTopRightTop = Frame(leftTopRight)
        leftTopRightTop.pack(expand="no", side="top")

        leftTopRightBottom = Frame(leftTopRight)
        leftTopRightBottom.place(anchor="center", relx=.5, rely=.5)
        
        motorControlTop = Frame(leftTopRightTop)
        motorControlTop.pack(fill="both", expand="no", side="top", padx=10)
        
        motorControlBottom = Frame(leftTopRightTop)
        motorControlBottom.pack(fill="both", expand="no", side="bottom", padx=10, pady=10)

        stick = Frame(leftTopLeftBottom)
        stick.place(anchor="center", relx=.5, rely=.5)

        # Serial Output Area 
        enDbgSerButton = Button(rightBottomTop)
        enDbgSerButton["text"] = "Enable Serial Debug"
        enDbgSerButton["command"] = self.enDbgSerial
        enDbgSerButton.pack(fill="x", expand="yes", side="left")

        disDbgSerButton = Button(rightBottomTop)
        disDbgSerButton["text"] = "Disable Serial Debug"
        disDbgSerButton["command"] = self.disDbgSerial
        disDbgSerButton.pack(fill="x", expand="yes", side="right")
           
        self.serialOutputScroll = Scrollbar(rightBottomBottom)
        self.serialOutputScroll.pack(fill="y", side="right")

        self.serialOutput = Listbox(rightBottomBottom, bd=0, yscrollcommand=self.serialOutputScroll.set)
        self.serialOutput.pack(fill="both", expand="yes")
        
        self.serialOutputScroll.config(command=self.serialOutput.yview)

        for i in range(100):
            self.serialOutput.insert(END, i)
            
        self.serialOutput.yview(END)


        # Manual Motor Control Area
        # Description Labels
        allMTRLabel = Label(rightTopTop, text="All Motors").grid(row=0, sticky=W)
        xAxisLabel = Label(rightTopTop, text="X-Axis").grid(row=1, sticky=W)
        yAxisLabel = Label(rightTopTop, text="Y-Axis").grid(row=2, sticky=W)
        x1Label = Label(rightTopTop, text="X1").grid(row=3, sticky=W)
        x2Label = Label(rightTopTop, text="X2").grid(row=4, sticky=W)
        y1Label = Label(rightTopTop, text="Y1").grid(row=5, sticky=W)
        y2Label = Label(rightTopTop, text="Y2").grid(row=6, sticky=W)
        lockLabel = Label(rightTopTop, text="Enable").grid(row=7, column=1, sticky=S)

        # Entry Fields for the Speed
        self.allMTRSpeed = Entry(rightTopTop, state=DISABLED)
        self.allMTRSpeed.grid(row=0, column=2)
        self.xAxisSpeed = Entry(rightTopTop, state=DISABLED)
        self.xAxisSpeed.grid(row=1, column=2)
        self.yAxisSpeed = Entry(rightTopTop, state=DISABLED)
        self.yAxisSpeed.grid(row=2, column=2)
        self.x1Speed = Entry(rightTopTop, state=DISABLED)
        self.x1Speed.grid(row=3, column=2)
        self.x2Speed = Entry(rightTopTop, state=DISABLED)
        self.x2Speed.grid(row=4, column=2)
        self.y1Speed = Entry(rightTopTop, state=DISABLED)
        self.y1Speed.grid(row=5, column=2)
        self.y2Speed = Entry(rightTopTop, state=DISABLED)
        self.y2Speed.grid(row=6, column=2)

        # Locks for the Entry Fields
        self.allMTRLockStatus = IntVar()
        self.xAxisLockStatus = IntVar()
        self.yAxisLockStatus = IntVar()
        self.x1LockStatus = IntVar()
        self.x2LockStatus = IntVar()
        self.y1LockStatus = IntVar()
        self.y2LockStatus = IntVar()

        allMTRLock = Checkbutton(rightTopTop, variable = self.allMTRLockStatus, onvalue = 1, offvalue = 0).grid(row=0, column=1)
        xAxisLock = Checkbutton(rightTopTop, variable = self.xAxisLockStatus, onvalue = 1, offvalue = 0).grid(row=1, column=1)
        yAxisLock = Checkbutton(rightTopTop, variable = self.yAxisLockStatus, onvalue = 1, offvalue = 0).grid(row=2, column=1)
        x1Lock = Checkbutton(rightTopTop, variable = self.x1LockStatus, onvalue = 1, offvalue = 0).grid(row=3, column=1)
        x2Lock = Checkbutton(rightTopTop, variable = self.x2LockStatus, onvalue = 1, offvalue = 0).grid(row=4, column=1)
        y1Lock = Checkbutton(rightTopTop, variable = self.y1LockStatus, onvalue = 1, offvalue = 0).grid(row=5, column=1)
        y2Lock = Checkbutton(rightTopTop, variable = self.y2LockStatus, onvalue = 1, offvalue = 0).grid(row=6, column=1)

        # Send Button
        setManualSpeed = Button(rightTopBottom)
        setManualSpeed["text"] = "Send selected Values"
        setManualSpeed["command"] = self.setManSpeed
        setManualSpeed.pack(fill="x", expand="yes", side="top")

        # Unlock Button
        unlockMotorFields = Button(rightTopBottom)
        unlockMotorFields["text"] = "Unlock Fields"
        unlockMotorFields["command"] = self.unlockFields
        unlockMotorFields.pack(fill="x", expand="yes", side="bottom")


        # Debugging Area - Enable/Disable Buttons
        enDbgIMUButton = Button(leftBottomTopLeft)
        enDbgIMUButton["text"] = "Enable IMU Debug"
        enDbgIMUButton["command"] = self.enDbgIMU
        enDbgIMUButton.pack(fill="x", expand="yes", side="left")

        disDbgIMUButton = Button(leftBottomTopLeft)
        disDbgIMUButton["text"] = "Disable IMU Debug"
        disDbgIMUButton["command"] = self.disDbgIMU
        disDbgIMUButton.pack(fill="x", expand="yes", side="right")

        enDbgPIDButton = Button(leftBottomTopRight)
        enDbgPIDButton["text"] = "Enable PID Debug"
        enDbgPIDButton["command"] = self.enDbgPID
        enDbgPIDButton.pack(fill="x", expand="yes", side="left")

        disDbgPIDButton = Button(leftBottomTopRight)
        disDbgPIDButton["text"] = "Disable PID Debug"
        disDbgPIDButton["command"] = self.disDbgPID
        disDbgPIDButton.pack(fill="x", expand="yes", side="right")


        # Debugging Area - Scales and Labels
        xIMUDbgLabel = Label(xIMUDbgFrame, text="x-Axis").pack(side="left")
        yIMUDbgLabel = Label(yIMUDbgFrame, text="y-Axis").pack(side="left")

        xPIDDbgLabel = Label(xPIDDbgFrame, text="x-Axis").pack(side="left")
        yPIDDbgLabel = Label(yPIDDbgFrame, text="y-Axis").pack(side="left")
        
        self.xIMUDbgScale = Scale(xIMUDbgFrame, from_=120.0, to=240.0)
        self.xIMUDbgScale.pack(fill="both", expand="yes", side="left")
        self.xIMUDbgScale.set(180.0)

        self.yIMUDbgScale = Scale(yIMUDbgFrame, from_=120.0, to=240.0)
        self.yIMUDbgScale.pack(fill="both", expand="yes", side="left")
        self.yIMUDbgScale.set(180.0)

        self.xPIDDbgScale = Scale(xPIDDbgFrame, from_=-20.0, to=20.0)
        self.xPIDDbgScale.pack(fill="both", expand="yes", side="left")

        self.yPIDDbgScale = Scale(yPIDDbgFrame, from_=-20.0, to=20.0)
        self.yPIDDbgScale.pack(fill="both", expand="yes", side="left")


        # Remote Control Area
        speedPlusButton = Button(leftTopLeftTop)
        speedPlusButton["text"] = "Speed +"
        speedPlusButton["command"] = self.speedUp
        speedPlusButton.pack(fill="x", expand="yes", side="left", padx=5)
        
        speedMinusButton = Button(leftTopLeftTop)
        speedMinusButton["text"] = "Speed -"
        speedMinusButton["command"] = self.speedDown
        speedMinusButton.pack(fill="x", expand="yes", side="right")


        # Control Stick
        forwardButton = Button(stick, height=4, width=8)
        forwardButton["text"] = "^"
        forwardButton["command"] = self.forward
        forwardButton.grid(row=0, column=1)

        leftButton = Button(stick, height=4, width=8)
        leftButton["text"] = "<"
        leftButton["command"] = self.left
        leftButton.grid(row=1, column=0)
        
        rightButton = Button(stick, height=4, width=8)
        rightButton["text"] = ">"
        rightButton["command"] = self.right
        rightButton.grid(row=1, column=2)
       
        centerButton = Button(stick, height=2, width=4)
        centerButton["text"] = "o"
        centerButton["command"] = self.center
        centerButton.grid(row=1, column=1)

        backButton = Button(stick, height=4, width=8)
        backButton["text"] = "v"
        backButton["command"] = self.back
        backButton.grid(row=2, column=1)

        
        # Top Motor Control
        enMotorsButton = Button(motorControlTop)
        enMotorsButton["text"] = "Enable Motors"
        enMotorsButton["command"] = self.enMotors
        enMotorsButton.pack(fill="x", expand="yes", side="left", padx=5)

        disMotorsButton = Button(motorControlTop)
        disMotorsButton["text"] = "Disable Motors"
        disMotorsButton["command"] = self.disMotors
        disMotorsButton.pack(fill="x", expand="yes", side="right", padx=5)

        zeroSpeedButton = Button(motorControlBottom)
        zeroSpeedButton["text"] = "0 Speed"
        zeroSpeedButton["command"] = self.zeroSpeed
        zeroSpeedButton.pack(fill="x", expand="yes", side="top", pady=3)

        flushCacheButton = Button(motorControlBottom)
        flushCacheButton["text"] = "Flush Cache "
        flushCacheButton["command"] = self.flushCache
        flushCacheButton.pack(fill="x", expand="yes", side="bottom", pady=3)


        # Bottom Motor Control
        self.xOffsetField = Entry(leftTopRightBottom, width=5)
        self.xOffsetField.insert(0, -4)
        self.xOffsetField.grid(row=0, column=0, sticky=E)
        
        self.yOffsetField = Entry(leftTopRightBottom, width=5)
        self.yOffsetField.insert(0, -3)
        self.yOffsetField.grid(row=1, column=0)

        setXOffsetButton = Button(leftTopRightBottom, padx=20)
        setXOffsetButton["text"] = "Set x IMU Offset"
        setXOffsetButton["command"] = self.setIMUx
        setXOffsetButton.grid(row=0, column=1, sticky=E)
        
        setYOffsetButton = Button(leftTopRightBottom, padx=20)
        setYOffsetButton["text"] = "Set y IMU Offset"
        setYOffsetButton["command"] = self.setIMUy
        setYOffsetButton.grid(row=1, column=1, sticky=E)

        Placeholder1 = Label(leftTopRightBottom, text=" ").grid(row=2, column=1)
        Placeholder2 = Label(leftTopRightBottom, text=" ").grid(row=3, column=1)
        
        QUIT = Button(leftTopRightBottom)
        QUIT["text"] = "Exit"
        QUIT["command"] = self.gracefulQuit
        QUIT.grid(row=3, column=1)
        

    def gracefulQuit(self):
        # disable serial link first
        root.destroy()
        print "quit only if motors are off!!!"
            
    def setIMUx(self):
        print "setting imu x offset"

    def setIMUy(self):
        print "setting imu y offset"
        
    def flushCache(self):
        print "flushing cache"
        
    def zeroSpeed(self):
        print "setting speed to zero"
        
    def speedUp(self):
        print "speeding up"

    def speedDown(self):
        print "speeding down"
        
    def forward(self):
        print "going forward"

    def left(self):
        print "going left"

    def right(self):
        print "going right"

    def back(self):
        print "going back"

    def center(self):
        print "centering"

    def enMotors(self):
        print "enabling motors"

    def disMotors(self):
        print "disabling motors"

    def setManSpeed(self):
        print "settings selected speeds"
 
    def enDbgIMU(self):
        print "en debug imu"

    def disDbgIMU(self):
        print "dis debug imu"

    def enDbgPID(self):
        print "en debug pid"

    def disDbgPID(self):
        print "dis debug pid"

    def enDbgSerial(self):
        print "en debug serial"

    def disDbgSerial(self):
        print "dis debug serial"
    
    def unlockFields(self):
        print "Unlocking entry fields..."
        self.allMTRSpeed.config(state=NORMAL)
        self.allMTRSpeed.update()
        if self.allMTRLockStatus.get() == 1:
            self.allMTRSpeed.config(state=NORMAL)
            self.allMTRSpeed.update()
        elif self.allMTRLockStatus.get() == 0:
            self.allMTRSpeed.config(state=DISABLED)
            self.allMTRSpeed.update()

        if self.xAxisLockStatus.get() == 1:
            self.xAxisSpeed.config(state=NORMAL)
            self.xAxisSpeed.update()
        elif self.xAxisLockStatus.get() == 0:
            self.xAxisSpeed.config(state=DISABLED)
            self.xAxisSpeed.update()

        if self.yAxisLockStatus.get() == 1:
            self.yAxisSpeed.config(state=NORMAL)
            self.yAxisSpeed.update()
        elif self.yAxisLockStatus.get() == 0:
            self.yAxisSpeed.config(state=DISABLED)
            self.yAxisSpeed.update()
            
        if self.x1LockStatus.get() == 1:
            self.x1Speed.config(state=NORMAL)
            self.x1Speed.update()
        elif self.x1LockStatus.get() == 0:
            self.x1Speed.config(state=DISABLED)
            self.x1Speed.update()

        if self.x2LockStatus.get() == 1:
            self.x2Speed.config(state=NORMAL)
            self.x2Speed.update()
        elif self.x2LockStatus.get() == 0:
            self.x2Speed.config(state=DISABLED)
            self.x2Speed.update()

        if self.y1LockStatus.get() == 1:
            self.y1Speed.config(state=NORMAL)
            self.y1Speed.update()
        elif self.y1LockStatus.get() == 0:
            self.y1Speed.config(state=DISABLED)
            self.y1Speed.update()

        if self.y2LockStatus.get() == 1:
            self.y2Speed.config(state=NORMAL)
            self.y2Speed.update()
        elif self.y2LockStatus.get() == 0:
            self.y2Speed.config(state=DISABLED)
            self.y2Speed.update()


def serial_read(i):
    app.serialOutput.insert(END, "Run number: " + str(i))
    app.serialOutput.yview(END)
    root.after(200, serial_read, i+1)


root = Tk()
app = Remote(master=root)
app.master.title("Quadcopter Remote Control")
app.master.minsize(800, 600)

# schedule serial data reading
root.after(1000, serial_read(1))

# start event handling
app.mainloop()



