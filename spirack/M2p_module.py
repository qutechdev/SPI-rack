import numpy as np
from .chip_mode import BICPINS_MODE, BICPINS_SPEED


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

    #allowed gain values
    gainlist = [1,10,100,1000,10000]


    def __init__(self, spi_rack, module):
        self.module = module
        self.spi_rack = spi_rack
        
        # Upon start-up copy the existing local settings 
        # and set module to local operation
        initial_gain = self.get_gain_local()
        initial_coupling = self.get_coupling_local()
        self.remote_set_gain_and_coupling(gain = initial_gain, coupling = initial_coupling)
        self.set_local()

    def set_remote(self):
        """
        Set the module to the remote mode and set the gain and coupling in accordance to the fronpanel switch settings.
        If the module is already in remote mode the function exits without changing settings
        """
               
        # check if module is already in remote mode, if so do nothing
        set_mode = self.get_operating_mode()
        if (set_mode == 1):
            print("Already in remote mode. No mode change made" )
            return #exit do nothing
       
        # Get the local gain and coupling settings (from the front panel) ... 
        gain = self.get_gain_local()
        couple = self.get_coupling_local()
        # ... and use them initially when switching to remote mode
        self.remote_set_gain_and_coupling(gain,couple)
        
        dummy_data = bytearray([0,0])
        r_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, dummy_data)
        
        #set remote bit to 1 (keeping other setings)
        high_byte  = r_data[0]|0b10000000
        #low_byte = r_data[1]|0b10000000
        low_byte = r_data[1]
        
        #write new settings (with high remote bit)
        w_data = bytearray([high_byte,low_byte])
        self.spi_rack.write_data(self.module, 5, BICPINS_MODE, BICPINS_SPEED, w_data)
        self.remote_set_gain_and_coupling(gain,couple)
         
    def set_local(self):
        """
        Set the module to local mode. The gain- and coupling settings revert to the front-panel switch setting.
        If the remotely set gain is different from the front panel setting there might be a jump in output due to a differnce in gain.
        """

        # read last high and low byte setting
        dummy_data = bytearray([0,0])
        r_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, dummy_data)
        
        #reset remote bit to 0 (keeping other setings)
        high_byte  = r_data[0] & 0b01111111
        #low_byte = r_data[1] & 0b01111111
        low_byte = r_data[1]

        #write new settings (with 0-set remote bit)
        w_data = bytearray([high_byte,low_byte])
        self.spi_rack.write_data(self.module, 5, BICPINS_MODE, BICPINS_SPEED, w_data)

    def remote_set_gain_and_coupling(self, gain, coupling):
        """
        Set gain and coupling mode 
        Posible gain settings : 1, 10, 100, 1000, 10.000
        Coupling 'DC'=1 'AC'=0
        
        gainlevel and couplemode parameters are checked for valid values.
        When one of the parameters is out of range or otherwise invalid no change in settings will be made
        
        The remote/local selection is maintained, so under a 'local' setting NO CHANGE in gain or coupling will happen.
        But the last written values can be readback
        """
        
        # check for valid gain and coupling values       
        if self._is_gain_valid(gain) == False or self._is_couplemode_valid(coupling) == False:
            print("Invalid gain or coupling, no settings changed" )
            return
        
        # read to preserve the value of the high byte for the upcoming 'write'
        dummy_data = bytearray([0,0])
        r_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, dummy_data)
        high_byte = r_data[0]
       
        # config gain bits
        if gain == 1:
            low_byte  = 0b10010000
        elif gain == 10:
            low_byte  = 0b10010001
        elif gain == 100:
            low_byte  = 0b10010010
        elif gain == 1000:
            low_byte  = 0b10010110
        else:
            # gain == 10000:
            low_byte  = 0b10011010
       
        # config coupling bit (while keeping other settings)
        if coupling == 'DC':
            low_byte = low_byte | 0b00010000  # For DC, set control bit to 1
        else:
            low_byte = low_byte & 0b11101111  # For AC, set control bit to 0

        w_data = bytearray([high_byte,low_byte])
        self.spi_rack.write_data(self.module, 5, BICPINS_MODE, BICPINS_SPEED, w_data)
        
    def _is_gain_valid(self, gain):
        # Check gain parameter for valid values
        # return True (valid) or False (invalid)
        if gain in self.gainlist:
            return True
        else:
            return False
            
    def _is_couplemode_valid(self, couplemode):
        # Check couple parameter for valid values
        # return True (valid) or False (invalid)
        if couplemode in ['AC','DC']:
            return True
        else:
            return False
    
    def get_operating_mode(self):
        """
        read the operating status of the module.
        Modes can be 'remote' or 'local'
        """
    
        mode = ''
        dummy_data = bytearray([0,0])
        r_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, dummy_data)
       
        if r_data[0] & 0b10000000:
            mode = 'remote'
        else:
            mode = 'local'
      
        return(mode)

    def get_gain_setting(self):
        """
        Get lastgain switch setting command.
        This in NOT the same as the gain settings of the switch panel!
        This is the last gain set in remote mode.
        Posible results: 1, 10, 100, 1000, 10000
        """
        if self.get_operating_mode()=='local':
            return self.get_gain_local()
        else:
            return self.get_gain_remote()

    def get_gain_remote(self):
        s_data = bytearray([0, 0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, s_data)
        gain = -1
        if r_data[1] & 0b00001111 == 0:
            gain = 1
        if r_data[1] & 0b00001111 == 0b00000001:
            gain = 10
        if r_data[1] & 0b00001111 == 0b00000010:
            gain = 100
        if r_data[1] & 0b00001111 == 0b00000110:
            gain = 1000
        if r_data[1] & 0b00001111 == 0b00001010:
            gain = 10000
        return (gain)


    def get_gain_local(self):
        """
        Get the front-panel gain switch setting.
        Posible results: 1, 10, 100, 1000, 10000
        """
        s_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 4, BICPINS_MODE, BICPINS_SPEED, s_data)

        gain = -1
        if r_data[1] & 0b00001111 == 0:
            gain = 1
        elif r_data[1] & 0b00001111 == 0b00000001:
            gain = 10
        elif r_data[1] & 0b00001111 == 0b00000010:
            gain = 100
        elif r_data[1] & 0b00001111 == 0b00000110:
            gain = 1000
        elif r_data[1] & 0b00001111 == 0b00001010:
            gain = 10000

        return (gain)

    def get_coupling_setting(self):
        """
         Get the last frontpanel couple mode (AC/DC) switch setting command.
         This in NOT the same as the couple mode settings on the front panel!
         posible results: 'DC'=1 AC='0'
        """
        if self.get_operating_mode()=='local':
            return self.get_coupling_local()
        else:
            return self.get_coupling_remote()
        
    def get_coupling_remote(self):
        dummy_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 6, BICPINS_MODE, BICPINS_SPEED, dummy_data)
        mode = -1
        if r_data[1] & 0b00010000:
            mode = 'DC'
        else:
            mode = 'AC'
        return (mode)
        
    def get_coupling_local(self):
        """
         Get the frontpanel couple mode (AC/DC) switch setting.
         posible results: 'DC'=1 AC='0'
        """
        dummy_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 4, BICPINS_MODE, BICPINS_SPEED, dummy_data)
        mode = -1
        if r_data[1] & 0b00010000:
            mode = 'DC'
        else:
            mode = 'AC'
        return (mode)

    def get_clipped_unlatched(self):
        # Inverted polarity
        # Return:
        # 0 if none clipped
        # 1 if pos clipped
        # 2 if neg clipped
        # 3 if pos and neg clipped (is posible when clipped is latched)
        
        clipped = ''
        dummy_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 3, BICPINS_MODE, BICPINS_SPEED, dummy_data)

        if (r_data[0] & 0b00110000) == 0b00110000:
            clipped = 'not clipped'
        elif (((r_data[0] & 0b00010000) == 0) & ((r_data[0] & 0b00100000) == 0)):
            clipped = 'pos and neg clipped'
        elif (r_data[0] & 0b00010000) == 0:
            clipped = 'pos clipped'
        elif (r_data[0] & 0b00100000) == 0:
            clipped = 'neg clipped'

        return (clipped)
       
    def get_clipped_latched(self):
        # Regular polarity (non-inverted)
        # Return:
        # 0 if none clipped
        # 1 if pos clipped
        # 2 if neg clipped
        # 3 if pos and neg clipped (is posible when clipped is latched)
        clipped = ''
        dummy_data = bytearray([0,0])
        r_data = self.spi_rack.read_data(self.module, 4, BICPINS_MODE, BICPINS_SPEED, dummy_data)
 
        if (((r_data[0] & 0b01000000) == 0) & ((r_data[0] & 0b10000000) == 0)):
                clipped = 'not clipped'
        elif (r_data[0] & 0b11000000 == 0b11000000):
                clipped = 'pos and neg clipped'
        elif (r_data[0] & 0b01000000):
                clipped = 'neg clipped'
        elif (r_data[0] & 0b10000000):
                clipped = 'pos clipped'

        return (clipped)
