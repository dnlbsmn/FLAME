import wx
import serial
from threading import Thread

# Handles the reading from serial 
class serial_reader(object):
    # Initialise
    def __init__(self, callback=None):
        self.callback = callback
        self.thread = None
        self.alive = False

    # Initialises communication with arduino
    def start_reader(self, serial_cfg):
        self.ser_cfg = serial_cfg
        self.serial = serial.Serial(**serial_cfg)

        self.serial.timeout = 0.1
        self.alive = True

        # Begin background thread reading serial
        self.thread = Thread(target=self.serial_read)
        self.thread.daemon = True
        self.thread.start()

    # End reader thread
    def stop_reader(self):
        #Stop the receiver thread, wait until it is finished.
        if self.thread is not None:
            # signal no more reads
            self.alive = False
            # wait until thread has finished
            self.thread.join()
            self.thread = None

        # cleanup
        self.serial.close()

    #Thread that handles the incoming traffic.
    def serial_read(self):

        while self.alive:
            try:
                text = self.serial.readline().decode()
                if text and self.callback:
                    # return value to main loop in thread-safe manner
                    wx.CallAfter(self.callback, text)
            except serial.serialutil.SerialException:
                # will happen when Windows goes in sleep mode
                print('serial.serialutil.SerialException')

# UI stuffs
class MyFrame(wx.Frame):    
    # Initialise
    def __init__(self):
        self.init_gui()

        self.f1 = "0.0"
        self.f2 = "0.0"
        self.po = "0.0"
        self.lock = False
        
        # Start reader thread
        self.serial_cfg = {'port': 'COM7', 'baudrate': 9600}
        self.ser_rd = serial_reader(callback=self.on_serial)
        self.ser_rd.start_reader(self.serial_cfg)

        # Define close logic
        self.Bind(wx.EVT_CLOSE, self.on_close)

    # Setup window layout
    def init_gui(self):
        super().__init__(parent=None, title='Canopy control', size=(430, 200))
        self.panel = wx.Panel(self)       
        self.init_col1()
        self.init_col2()
        self.init_col3()
        self.init_col4()
        big_sizer = wx.BoxSizer(wx.HORIZONTAL) 
        big_sizer.Add(self.col_1_sizer)
        big_sizer.Add(self.col_2_sizer)
        big_sizer.Add(self.col_3_sizer)
        big_sizer.Add(self.col_4_sizer)

        self.panel.SetSizer(big_sizer)       
        self.Show()

    # Define Labels column
    def init_col1(self):
        self.col_1_sizer = wx.BoxSizer(wx.VERTICAL) 

        # Title Line
        line1= wx.BoxSizer(wx.HORIZONTAL) 
        l1_label1 = wx.StaticText(self.panel, -1, " ") 
        line1.Add(l1_label1, 0, wx.EXPAND|wx.CENTER|wx.ALL,5)
        self.col_1_sizer.Add(line1)    

        # Frequency 1 set
        line2 = wx.BoxSizer(wx.HORIZONTAL) 
        l2_label1 = wx.StaticText(self.panel, -1, "Motor 1 Frequency") 
        line2.Add(l2_label1, 1, wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,8)
        self.col_1_sizer.Add(line2)

        # Frequency 2 set
        line3 = wx.BoxSizer(wx.HORIZONTAL) 
        l3_label1 = wx.StaticText(self.panel, -1, "Motor 2 Frequency") 
        line3.Add(l3_label1, 1, wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,8)
        self.col_1_sizer.Add(line3)

        # Phase offset
        line4 = wx.BoxSizer(wx.HORIZONTAL) 
        l4_label1 = wx.StaticText(self.panel, -1, "Phase offset") 
        line4.Add(l4_label1, 1, wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,8)
        self.col_1_sizer.Add(line4)

    # Define target input column
    def init_col2(self):
        self.col_2_sizer = wx.BoxSizer(wx.VERTICAL) 

        # Title Line
        l1_label1 = wx.StaticText(self.panel, -1, "Targets") 
        self.col_2_sizer.Add(l1_label1, 0, wx.CENTER|wx.ALL,5)

        # Frequency 1 set
        self.frequency1_set = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_2_sizer.Add(self.frequency1_set,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 
        self.frequency1_set.Bind(wx.EVT_TEXT, self.f1_event)

        # Frequency 2 set
        self.frequency2_set = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_2_sizer.Add(self.frequency2_set,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 
        self.frequency2_set.Bind(wx.EVT_TEXT, self.f2_event)

        # Phase offset
        self.phase_offset_set = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_2_sizer.Add(self.phase_offset_set,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 
        self.phase_offset_set.Bind(wx.EVT_TEXT, self.po_event)

        btns = wx.BoxSizer(wx.HORIZONTAL)
        # Lock btn
        self.lock_btn = wx.Button(self.panel, label='Unlock')
        self.lock_btn.Bind(wx.EVT_BUTTON, self.on_press_lock)
        btns.Add(self.lock_btn, 0, wx.ALL | wx.LEFT, 5)

        # Apply btn
        self.apply_btn = wx.Button(self.panel, label='Apply')
        self.apply_btn.Bind(wx.EVT_BUTTON, self.on_press_apply)
        btns.Add(self.apply_btn, 0, wx.ALL, 5)
        self.col_2_sizer.Add(btns)
    
    # Define increment buttons
    def init_col3(self):
        self.col_3_sizer = wx.BoxSizer(wx.VERTICAL) 

        # Title Line
        l1_label1 = wx.StaticText(self.panel, -1, " ") 
        self.col_3_sizer.Add(l1_label1, 0, wx.EXPAND|wx.CENTER|wx.ALL,6)

        # increment buttons 
        for btn_params in [['^', "f1_increase", wx.TOP], ['v', "f1_decrease", wx.BOTTOM], ['^', "f2_increase", wx.TOP], ['v', "f2_decrease", wx.BOTTOM], ['^', "po_increase", wx.TOP], ['v', "po_decrease", wx.BOTTOM]]:
            btn = wx.Button(self.panel, label=btn_params[0], size=(25, 14))
            btn.Bind(wx.EVT_BUTTON, lambda evt, temp=btn_params[1]: self.increment_buttons(evt, temp))
            self.col_3_sizer.Add(btn, 0, btn_params[2], 2)

    # Encoder readings column
    def init_col4(self):
        self.col_4_sizer = wx.BoxSizer(wx.VERTICAL) 

        # Title Line
        l1_label1 = wx.StaticText(self.panel, -1, "Encoder readings") 
        self.col_4_sizer.Add(l1_label1, 0, wx.ALL, 5)

        # Frequency 1 set
        self.frequency1_val = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_4_sizer.Add(self.frequency1_val,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 

        # Frequency 2 set
        self.frequency2_val = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_4_sizer.Add(self.frequency2_val,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 

        # Phase offset
        self.phase_offset_val = wx.TextCtrl(self.panel, value = "0", style = wx.TE_READONLY|wx.TE_CENTER) 
        self.col_4_sizer.Add(self.phase_offset_val,1,wx.EXPAND|wx.ALIGN_LEFT|wx.ALL,5) 

        # Stop btn
        self.stop_btn = wx.Button(self.panel, label='Stop',)
        self.stop_btn.Bind(wx.EVT_BUTTON, self.on_press_stop)
        self.col_4_sizer.Add(self.stop_btn, 0, wx.ALL, 5,)
    
    # Get value of static text for phase offset
    def po_event(self, event):
        self.po = event.GetString()

    # Get value of static text for motor 1 frequency
    def f1_event(self, event):
        self.f1 = event.GetString()

    # Get value of static text for motor 2 frequency
    def f2_event(self, event):
        self.f2 = event.GetString()

    # Apply button logic
    def on_press_apply(self, event):
        # If all inputs are valid send string of commands to arduino
        if (self.check_if_in_range(self.f1, 0, 3) & self.check_if_in_range(self.f2, 0, 3) & self.check_if_in_range(self.po, 0, 360)):
            s = "gui," + self.f1 + ',' + self.f2 + ',' + self.po
            self.ser_rd.serial.write(bytes(s + '\n', 'utf-8'))

    # Stop button logic
    def on_press_stop(self, event):
        # Send stop command to arduino
        s = "x"
        self.ser_rd.serial.write(bytes(s + '\n', 'utf-8'))

    # Lock button logic
    def on_press_lock(self, event):
        # Change editability of target inputs
        self.lock = not self.lock
        self.frequency1_set.SetEditable(self.lock)
        self.frequency2_set.SetEditable(self.lock)
        self.phase_offset_set.SetEditable(self.lock)

        # Change button name
        if (self.lock == False):
            self.lock_btn.SetLabel("Unlock")
        else:
            self.lock_btn.SetLabel("Lock")

    # Check is string is a valid input
    def check_if_in_range(self, val, min, max):
        if (not val.replace(".", "").isnumeric()):
            return 0
        elif (float(val) < min) | (float(val) > max):
            return 0
        return 1

    # Callback for incremental buttons
    def increment_buttons(self, event, label):
        if label == "f1_increase":
            if self.check_if_in_range(str(float(self.f1)+0.1), 0, 3):
                self.f1 = str(round(float(self.f1)+0.1, 2))
        elif label == "f1_decrease":
            if self.check_if_in_range(str(float(self.f1)-0.1), 0, 3):
                self.f1 = str(round(float(self.f1)-0.1, 2))
        elif label == "f2_increase":
            if self.check_if_in_range(str(float(self.f2)+0.1), 0, 3):
                self.f2 = str(round(float(self.f2)+0.1, 2))
        elif label == "f2_decrease":
            if self.check_if_in_range(str(float(self.f2)-0.1), 0, 3):
                self.f2 = str(round(float(self.f2)-0.1, 2))
        elif label == "po_increase":
            if self.check_if_in_range(str(float(self.po)+15), 0, 360):
                self.po = str(round(float(self.po)+15, 1))
        elif label == "po_decrease":
            if self.check_if_in_range(str(float(self.po)-15), 0, 360):
                self.po = str(round(float(self.po)-15, 1))
        
        self.frequency1_set.SetLabel(self.f1)
        self.frequency2_set.SetLabel(self.f2)
        self.phase_offset_set.SetLabel(self.po)

    # Callback for recieving message from serial
    def on_serial(self, text):
        # Update encoder readings
        a = text.split(',')
        if (len(a) == 3):
            self.frequency1_val.SetLabel(a[0])
            self.frequency2_val.SetLabel(a[1])
            self.phase_offset_val.SetLabel(a[2])

    # Callback for closing the window
    def on_close(self, evt):
        # Shutdown serial read thread before closing
        if self.ser_rd.alive:
            self.ser_rd.stop_reader()
        evt.Skip()

if __name__ == '__main__':
    app = wx.App()
    frame = MyFrame()
    app.MainLoop()