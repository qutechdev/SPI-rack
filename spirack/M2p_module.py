import numpy as np
from spi_rack import *
from chip_mode import BICPINS_MODE, BICPINS_SPEED
from time import sleep

#global values
gainlist = [1,10,100,1000,10000]

class M2p_module(object):
    """M2p module interface class

    This class does the low level interfacing with the M2p V-measure module.
    It requires an SPI Rack object and module number at initialization.

    If the remote control is enabled, it also allows the user to set the gain and coupling settings remotely.
    It is also posible to readback a clip condition and the frontpanel switch settings in either remote and local mode.
    In remote mode the module ignores the front switch settings until remote control is disabled again.
    The remote mode condition is indicated with a LED on the frontpanel.

    Attributes:
        module: the module number set by the user (must coincide with hardware)
        remote_settings: contains a byte with the remote settigns (IQ gain and filter)
    """

    def __init__(self, spi_rack, module):
        self.module = module
        self.spi_rack = spi_rack

    def set_remote(self):
        """
        Set the module to the remote mode and set the gain and coupling in accordance to the fronpanel switch settings.
        If the module is already in remote mode the function exits without changing settings
        """
               
        # check if module is already in remote mode, if so do nothing
        set_mode = self.get_operating_mode()
        if (set_mode == 1):
            print("Already in remotemode, no mode change made" )
            return #exit do nothing
       
        # Get gainswitch and coupleswitcht setting
        gain = self.get_gainswitch()
        couple = self.get_couple_switchsetting()
        self.set_gain(gain,couple)
        
        dummy_data = bytearray([0,0])
        r_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, dummy_data)
        
        #set remote bit keep the others
        commandlow  = r_data[0]|0b10000000
        commandhigh = r_data[1]|0b10000000
        s_data = bytearray([commandhigh, commandlow,])
        self.spi_rack.write_data(self.module, 5, BICPINS_MODE, BICPINS_SPEED, s_data)
        self.set_gain(gain,couple)
         
    def set_local(self):
        """
        Set the module to local mode. The gain- and coupling settings reverts to the frontpanel switch setting.
        If the remotely set gain is different from the front panel setting there migh be a jump in output due to a differnce in gain.
        """

        # read last high and low byte setting
        dummy_data = bytearray([0,0])
        r_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, dummy_data)
        
        #reset remote bit keep the others
        commandlow  = r_data[0] & 0b01111111
        commandhigh = r_data[1] & 0b01111111  

        #write settings with reset remote bit
        s_data = bytearray([commandlow,commandhigh])
        self.spi_rack.write_data(self.module, 5, BICPINS_MODE, BICPINS_SPEED, s_data)

    def set_gain(self, gainlevel, couplemode):
        """
        Set gain and coupling mode 
        Posible gain settings : 1, 10, 100, 1000, 10.000
        Coupling DC=0 AC=1
        gainlevel and couplemode parameters are checked for valid values.
        When one of the parameters is out of range or otherwise invalid no change in settings will be made
        The remote/local setting will not ne changed, so if current setting is local NO CHANGE in gain or coupling will happen.
        But the last written values can be readback
        """
        
        # check for valid gain and couple values       
        if self.gainvalid(gainlevel) == False or self.couplemodevalid(couplemode) == False:
            print("Invalid gain or couplemode, no settings changed" )
            return
        
        # read high byte for remote bit and couple bit  
        dummy_data = bytearray([0,0])
        r_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, dummy_data)
        commandhigh = r_data[1]
       
        #commandhigh = 0b10000001
        # set gain bits
        if gainlevel == 1:
            commandlow  = 0b10010000
        if gainlevel == 10:
            commandlow  = 0b10010001
        if gainlevel == 100:
            commandlow  = 0b10010010
        if gainlevel == 1000:
            commandlow  = 0b10010110
        if gainlevel == 10000:
            commandlow  = 0b10011010
       
        # (re)setcouple bit while keeping remote bit setting
        if couplemode == 0:
            commandlow = commandlow | 0b00010000  # DC
        else:
            commandlow = commandlow & 0b11101111  # AC

        s_data = bytearray([commandhigh,commandlow])
        self.spi_rack.write_data(self.module, 5, BICPINS_MODE, BICPINS_SPEED, s_data)
        
    def gainvalid(self, gain):
        # Check gain parameter for valid values
        # return True (valid) or False (invalid)
        if gain in gainlist:
            return True
        else:
            return False
            
    def couplemodevalid(self, couplemode):
        # Check couple parameter for valid values
        # return True (valid) or False (invalid)
        if couplemode <0 or couplemode>1:
            return False
        else:
            return True
    
    def get_operating_mode(self):
        """
        read the operating status of the module.
        Either 1 = Remote mode or 0 = local mode
        """
    
        mode = 0
        dummy_data = bytearray([0,0])
        r_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, dummy_data)
       
        if r_data[0] & 0b10000000:
            mode = 1
        else:
            mode = 0
      
        return(mode)

    def get_lastgainset(self):
        """
        Get lastgain switch setting command.
        This in NOT the same as the gain settings of the switch panel!
        This is the last gain set in remote mode.
        posible results: 1, 10, 100, 1000, 10000
        """
        commandhigh = 0b00000000
        commandlow = 0b00000000
        s_data = bytearray([commandhigh, commandlow])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, s_data)
        gain = -1
        if r_data[1] & 0b00001111 == 0:
            gain = 1
        if r_data[1] & 0b00000001 == 0b00000001:
            gain = 10
        if r_data[1] & 0b00000010 == 0b00000010 :
            gain = 100
        if r_data[1] & 0b00000110 == 0b00000110:
            gain = 1000
        if r_data[1] & 0b00001010 == 0b00001010:
            gain = 10000
        return (gain)


    def get_gainswitch(self):
        """
        Get the frontpanel gain switch setting.
        posible results: 1, 10, 100, 1000, 10000
        """
        commandhigh = 0b00000000
        commandlow = 0b00000000
        s_data = bytearray([commandhigh, commandlow])
        r_data = self.spi_rack.read_data(self.module, 4, BICPINS_MODE, BICPINS_SPEED, s_data)

        gain = -1
        if r_data[1] & 0b00001111 == 0:
            gain = 1
        if r_data[1] & 0b00000001 == 0b00000001:
            gain = 10
        if r_data[1] & 0b00000010 == 0b00000010 :
            gain = 100
        if r_data[1] & 0b00000110 == 0b00000110:
            gain = 1000
        if r_data[1] & 0b00001010 == 0b00001010:
            gain = 10000

        return (gain)

    def get_lastcouple_switchsetting(self):
        """
         Get the last frontpanel couple mode (AC/DC) switch setting command.
         This in NOT the same as the couple mode settings on the front panel!
         posible results: 0 = DC 1=AC
        """
        dummy_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, dummy_data)
        mode = -1
        if r_data[1] & 0b00010000:
            mode = 1
        else:
            mode = 0
        return (mode)
        
    def get_couple_switchsetting(self):
        """
         Get the frontpanel couple mode (AC/DC) switch setting.
         posible results: 0 = DC 1=AC
        """
        dummy_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 4, BICPINS_MODE, BICPINS_SPEED, dummy_data)
        mode = -1
        if r_data[1] & 0b00010000:
            mode = 1
        else:
            mode = 0
        return (mode)
       
    def get_clipped_latched(self):
        # Return:
        # 0 if none clipped
        # 1 if pos clipped
        # 2 if neg clipped
        # 3 if pos and neg clipped (is posible when clipped is latched)
        clipped = -1
        dummy_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 7, BICPINS_MODE, BICPINS_SPEED, dummy_data)
 
        if (((r_data[0] & 0b01000000) == 0) & ((r_data[0] & 0b10000000) == 0)):
                clipped = 0
        elif (r_data[0] & 0b11000000 == 0b11000000):
                clipped = 3
        elif (r_data[0] & 0b01000000):
                clipped = 2
        elif (r_data[0] & 0b10000000):
                clipped = 1

        return (clipped)

    def get_clipped_unlatched(self):
        # Return:
        # 0 if none clipped
        # 1 if pos clipped
        # 2 if neg clipped
        # 3 if pos and neg clipped (is posible when clipped is latched)
        
        clipped = -1
        dummy_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 7, BICPINS_MODE, BICPINS_SPEED, dummy_data)

        if (r_data[0] & 0b00110000) == 0b00110000:
            clipped = 0
        elif (((r_data[0] & 0b00010000) == 0) & ((r_data[0] & 0b00100000) == 0)):
            clipped = 3
        elif (r_data[0] & 0b00010000) == 0:
            clipped = 1
        elif (r_data[0] & 0b00100000) == 0:
            clipped = 2                

        return (clipped)