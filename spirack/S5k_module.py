"""AWG module S5k interface

SPI Rack interface code for the S5k AWG module.

Example use : ::
    S5k = S5k_module(SPI_rack1, 3)

Todo:
    *Add readback from DAC ICs
"""

from .spi_rack import SPI_rack
from .chip_mode import AD9106_MODE, AD9106_SPEED, AD9106_RD_SPEED, LMK01010_MODE, LMK01010_SPEED, BICPINS_SPEED, BICPINS_MODE
import numpy as np

class S5k_module(object):
    """S5k module interface class

    This class does the low level interfacing with the S5k module. When creating
    an instance it requires a SPI_rack object passed as a parameter.

    The DACs can run in multiple modes: DC-value, sawtooth, noise and AWG. Each
    mode has its own parameters and requirements described by the function.

    Attributes:
        module (int): module number set by the user (must coincide with hardware)
        reference (str): set to either internal or external reference
        DAC_waveform_mode (list(str)): list containing the current functions settings per DAC
        DAC_DC_val (list(int)): list containing the DC values per DAC
        DAC_dgain (list(int)): list containing the (digital) gain settings per DAC
        DAC_doffset (list(int)): list containing the (digital) offsets per DAC
        DAC_clock_div (list(int)): list containing the clock division per DAC (IC)
        module_running (bool): True or False if the module is running
    """

    # Maps the DACs, as numbered at module front plate, to the DAC IC SPI address
    # and internal DAC (within the 4-channel DAC IC).
    # DAC:[DAC_IC,DAC_internal]
    # So DAC 1 maps to DAC IC with SPI addres 3 and internal DAC 4
    DAC_mapping = {1:[3,4], 2:[3,3], 3:[3,1], 4:[3,2], 5:[1,4], 6:[1,3], 7:[1,1], 8:[1,2],
                   9:[4,4], 10:[4,3], 11:[4,1], 12:[4,2], 13:[0,4], 14:[0,3], 15:[0,1], 16:[0,2]}

    def __init__(self, spi_rack, module):
        """Inits S5k module class

        Initialises the S5k module. At initialization the clock source will be
        set to internal and the clock to all DAC ICs will be activated at the
        undivided clock speed.

        Args:
            spi_rack (SPI_rack object): SPI_rack class object via which the communication runs
            module (int): module number set at hardware
        """
        self.spi_rack = spi_rack
        self.module = module

        self.write_LMK_data(0, (1<<31)) #Reset clock distribution
        self.write_LMK_data(0, 1<<16 | 0<<17 | 0b0<<8) #Enable channel 0, undivided
        self.write_LMK_data(1, 1<<16 | 0<<17 | 0b0<<8) #Enable channel 1, undivided
        self.write_LMK_data(6, 1<<16 | 0<<17 | 0b0<<8) #Enable channel 6, undivided
        self.write_LMK_data(7, 1<<16 | 0<<17 | 0b0<<8) #Enable channel 7, undivided
        
        self.DAreg = AD9106_registers          #DAreg contains all AD9106 register addresses
        self.DAC_waveform_mode = 16*[None]
        self.DAC_DC_val = 16*[None]
        self.DAC_dgain = 16*[None]
        self.DAC_doffset = 16*[None]
        self.DAC_clock_div = 16*[1]

        self.module_running = False
        self.reference = None
        self.set_clock_source('internal')
        self.run_module(False)
        
        self.DAC_limits = {1:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      2:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      3:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      4:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      5:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      6:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      7:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      8:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      9:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      10:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      11:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      12:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      13:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      14:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      15:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752},
                      16:{'Vmax':2.875,'Vmin':-2.875,'Vscale':5.752}}

    def set_waveform_mode(self, DAC, waveform):
        """Sets the selected DAC to a certain waveform mode

        Changes the the mode of the selected DAC to either: DC, sawtooth,
        noise or AWG.

        Args:
            DAC (int: 1-16): DAC for which to change the mode
            waveform (string): waveform type to set
        """
        # input checks
        possible_values = {'DC': 1, 'sawtooth': 0b10001, 'noise': 0b100001, 'AWG': 0}
        if waveform not in possible_values:
            raise ValueError('Value {} does not exist. Possible values are: {}'.format(waveform, possible_values))

        self.DAC_waveform_mode[DAC-1] = waveform

        DAC_IC = S5k_module.DAC_mapping[DAC][0]        # The DAC group (of 4 DACs)
        DAC_internal = S5k_module.DAC_mapping[DAC][1]  # The DAC number in the group

        if DAC_internal == 1:
            data = self.read_AD9106(self.DAreg.WAV2_1CONFIG, DAC_IC)
            data &= 0xFF00
            data |= possible_values[waveform]
            register = self.DAreg.WAV2_1CONFIG
        elif DAC_internal == 2:
            data = self.read_AD9106(self.DAreg.WAV2_1CONFIG, DAC_IC)
            data &= 0x00FF
            data |= possible_values[waveform]<<8
            register = self.DAreg.WAV2_1CONFIG
        elif DAC_internal == 3:
            data = self.read_AD9106(self.DAreg.WAV4_3CONFIG, DAC_IC)
            data &= 0xFF00
            data |= possible_values[waveform]
            register = self.DAreg.WAV4_3CONFIG
        else:
            data = self.read_AD9106(self.DAreg.WAV4_3CONFIG, DAC_IC)
            data &= 0x00FF
            data |= possible_values[waveform]<<8
            register = self.DAreg.WAV4_3CONFIG

        self.write_AD9106(register, data, DAC_IC)

    def get_waveform_mode(self, DAC):
        """Returns the waveform mode of the selected DAC

        Args:
            DAC (int: 1-16): DAC for which the waveform is checked
        """
        # input checks
        DAC_list = list(range(1,17,1))
        if DAC not in DAC_list:
            raise ValueError('DAC {} does not exist. Possible values are: {}'.format(DAC_list))

        # software value
        print("The waveform mode (in SW) is",self.DAC_waveform_mode[DAC])
        
        # hardware value
        possible_values = {0:'DC', 1:'sawtooth', 2:'noise', 3:'AWG'}
        if waveform not in possible_values:
            raise ValueError('Value {} does not exist. Possible values are: {}'.format(waveform, possible_values))

        DAC_IC = S5k_module.DAC_mapping[DAC][0]        # The DAC group (of 4 DACs)
        DAC_internal = S5k_module.DAC_mapping[DAC][1]  # The DAC number in the group

        if DAC_internal == 1:
            data = self.read_AD9106(self.DAreg.WAV2_1CONFIG, DAC_IC)
            data &= 0x3000
        elif DAC_internal == 2:
            data = self.read_AD9106(self.DAreg.WAV2_1CONFIG, DAC_IC)
            data &= 0x0030
        elif DAC_internal == 3:
            data = self.read_AD9106(self.DAreg.WAV4_3CONFIG, DAC_IC)
            data &= 0x3000
        else:
            data = self.read_AD9106(self.DAreg.WAV4_3CONFIG, DAC_IC)
            data &= 0x0030

        print("The waveform mode (in HW) is",possible_values[data])
        return possible_values[data]

    def set_sawtooth_parameters(self, DAC, sawtooth_type, stepsize):
        """Set the parameters for sawtooth mode

        Set the type of sawtooth and the stepsize of the sawtooth for a
        specific DAC. DAC needs to be in sawtooth mode to output. The sawtooth
        is generated by a 2^14 counter running at clk/stepsize. The counter is
        fixed but the stepsize can be adjusted from 1 to 63.

        Args:
            DAC (int: 1-16): DAC of which sawtooth settings to change
            sawtooth_type (str): type of sawtooth to output
            stepsize (int: 1-63): clock cycles per counter step
        """
        if self.DAC_waveform_mode[DAC-1] != 'sawtooth':
            print('DAC {} is not set to sawtooth mode!'.format(DAC))

        # input checks
        possible_values = {'ramp_up': 0, 'ramp_down': 1, 'triangle': 2, 'no_wave': 3}
        if sawtooth_type not in possible_values:
            raise ValueError('Value {} does not exist. Possible values are: {}'.format(sawtooth_type, possible_values))

        # Decode which IC and internal DAC is being addressed
        DAC_IC = S5k_module.DAC_mapping[DAC][0]
        DAC_internal = S5k_module.DAC_mapping[DAC][1]

        # Depending on the internal DAC, choose correct registers
        if DAC_internal == 1:
            data = self.read_AD9106(self.DAreg.SAW2_1CONFIG, DAC_IC)
            data &= 0xFF00
            data |= (stepsize<<2) | possible_values[sawtooth_type]
            register = self.DAreg.SAW2_1CONFIG
        elif DAC_internal == 2:
            data = self.read_AD9106(self.DAreg.SAW2_1CONFIG, DAC_IC)
            data &= 0x00FF
            data |= (stepsize<<10) | (possible_values[sawtooth_type]<<8)
            register = self.DAreg.SAW2_1CONFIG
        elif DAC_internal == 3:
            data = self.read_AD9106(self.DAreg.SAW4_3CONFIG, DAC_IC)
            data &= 0xFF00
            data |= (stepsize<<2) | possible_values[sawtooth_type]
            register = self.DAreg.SAW4_3CONFIG
        else:
            data = self.read_AD9106(self.DAreg.SAW4_3CONFIG, DAC_IC)
            data &= 0x00FF
            data |= (stepsize<<10) | (possible_values[sawtooth_type]<<8)
            register = self.DAreg.SAW4_3CONFIG

        self.write_AD9106(register, data, DAC_IC)

    def set_DAC_scale(self, DAC, Vmax=2.875, Vmin=-2.875):
        """
        "Informs" the SW about the max and min levels for the DAC.
        The levels themselves are deterined by the HW used,
        and should be known to the user.
        """
        # input checks
        if Vmax > 2.875:
            print('The given max value is too high. Will be recorded as {}'.format(2.875))
            Max_level=2.875
        if Vmax < 0:
            print('The given max value of {} is negative. Please enter a positive value'.format(Vmax))
            Max_level=2.875
        else:
            Max_level=Vmax
        
        # input checks
        if Vmin < -2.875:
            print('The given min value is too low. Will be recorded as {}'.format(-2.875))
            Min_level=-2.875
        if Vmin > 0:
            print('The given min value of {} is positive. Please enter a negative value'.format(Vmin))
            Min_level=-2.875
        else:
            Min_level=Vmin
        
        DAC_limits[DAC-1]['Vmax'] = Max_level
        DAC_limits[DAC-1]['Vmin'] = Min_level
        DAC_limits[DAC-1]['Vscale'] = Max_level - Min_level + 0.002   # 5.75V; need to slightly stretch the scale to avoid special cases at ±2.875

    def set_DC_value(self, DAC, value):
        """Set the DC value

        This sets the DC value for when the DAC is in DC mode. This value is
        also output when the DAC is in any other mode, but the module is not
        running.

        Args:
            DAC (int: 1-16): DAC of which DC value to change
            value (float): Voltage within range (by default ±2.875V)
        """
        if self.DAC_waveform_mode[DAC-1] != 'DC':
            #raise ValueError('DAC {} needs to be set to DC to set DC value'.format(DAC))
            print("\nWarning! DAC",DAC-1,"is in",self.DAC_waveform_mode[DAC-1],"mode and not currently configured to a DC output.",
            "\nIt will only show this DC level when the module is NOT running/triggered!")

        # The S5k scale is set by default to ±2.875V
        # The 'set_DAC_scale' only needs to run if the S5k is a customiz build with a different scale
        #self.set_DAC_scale(DAC, Vmax=2.875, Vmin=-2.875)
        
        # input checks
        if value > self.DAC_limits[DAC-1]['Vmax']:
            print('Warning! Value too high, set to: {}'.format(self.DAC_limits[DAC-1]['Vmax']))
            value = self.DAC_limits[DAC-1]['Vmax']
        elif value < self.DAC_limits[DAC-1]['Vmin']:
            print('Warning! Value too low, set to: {}'.format(self.DAC_limits[DAC-1]['Vmin']))
            value = self.DAC_limits[DAC-1]['Vmin']

        self.DAC_DC_val[DAC-1] = value
        # Calculate code (as int) from voltage (as float)
        # For the default range of ±2.875V, the resolution is ~712 codes per Volt 
        # (so 1 code is ~1.4mV ),
        Vscale = self.DAC_limits[DAC-1]['Vscale']
        intval = int((-4096/Vscale * value) - 7) #Calculate int value from float
        data = intval << 4

        DAC_IC = S5k_module.DAC_mapping[DAC][0]
        DAC_internal = S5k_module.DAC_mapping[DAC][1]

        if DAC_internal == 1:
            register = self.DAreg.DAC1_CST
        elif DAC_internal == 2:
            register = self.DAreg.DAC2_CST
        elif DAC_internal == 3:
            register = self.DAreg.DAC3_CST
        else:
            register = self.DAreg.DAC4_CST

        self.write_AD9106(register, data, DAC_IC)

    def get_DC_value(self, DAC):
        """Returns the DC value for the given DAC
        """
        print("\nget_DC_value:")
        
        # software value
        print("    For DAC",DAC,"the DC value (in SW) is:",self.DAC_DC_val[DAC-1])

        # hardware value
        DAC_IC = S5k_module.DAC_mapping[DAC][0]
        DAC_internal = S5k_module.DAC_mapping[DAC][1]
        print("\nget_DC_value:",
        "\nDAC",DAC,"is produced by AWG (DAC_IC) number",DAC_IC,"channel",DAC_internal,".")
        
        if DAC_internal == 1:
            register = self.DAreg.DAC1_CST
        elif DAC_internal == 2:
            register = self.DAreg.DAC2_CST
        elif DAC_internal == 3:
            register = self.DAreg.DAC3_CST
        else:
            register = self.DAreg.DAC4_CST
        
        DAC_raw_data = self.read_AD9106(register, DAC_IC)
        print("    For DAC",DAC,": raw data is",DAC_raw_data,"(bin ",bin(DAC_raw_data),")")
        DAC_DC_code = DAC_raw_data >> 4
        print("    For DAC",DAC,": DC data is",DAC_DC_code)

        dc_level = DAC_DC_code*self.DAC_limits[DAC-1]['Vscale']/(-4096)
        print("    For DAC",DAC,"the DC value (in HW) is:",round(dc_level,3))
        return dc_level

    def set_digital_gain(self, DAC, gain):
        """Set the digital gain

        Sets the digital gain of the DAC.

        Args:
            DAC (int: 1-16): DAC of which DC value to change
            gain (float): value between -1.99 and 2.0
        """
        # input checks
        if gain > 2.0:
            print('Warning! Value too high. Gain is set to 2.0')
            gain = 2.0
        elif gain < -1.99:
            print('Warning! Value too low. Gain is set to -1.99')
            gain = -1.99
        
        gain = -gain # Compensate for inverting amplifier
        self.DAC_dgain[DAC-1] = gain
        data = int(gain*(1<<4))    # multiplying by 16 is the same as shifting 4 bits to the left to align with the register field

        DAC_IC = S5k_module.DAC_mapping[DAC][0]
        DAC_internal = S5k_module.DAC_mapping[DAC][1]

        if DAC_internal == 1:
            register = self.DAreg.DAC1_DGAIN
        elif DAC_internal == 2:
            register = self.DAreg.DAC2_DGAIN
        elif DAC_internal == 3:
            register = self.DAreg.DAC3_DGAIN
        else:
            register = self.DAreg.DAC4_DGAIN

        self.write_AD9106(register, data, DAC_IC)

    def set_digital_offset(self, DAC, offset):
        """Set the digital offset

        Sets the digital offset of the DAC. 
        The user should know the voltage increment of one code step.

        Args:
            DAC (int: 1-16): DAC of which DC value to change
            offset (float): value between -2.85 and 2.85
        """
        
        # The S5k scale is set by default to ±2.875V
        # The 'set_DAC_scale' only needs to run if the S5k is a customiz build with a different scale
        #self.set_DAC_scale(DAC, Vmax=2.875, Vmin=-2.875)
        
        # input checks
        if offset > self.DAC_limits[DAC-1]['Vmax']:
            print('Warning! Value too high, set to: {}'.format(self.DAC_limits[DAC-1]['Vmax']))
            offset = self.DAC_limits[DAC-1]['Vmax']
        elif offset < self.DAC_limits[DAC-1]['Vmin']:
            print('Warning! Value too low, set to: {}'.format(self.DAC_limits[DAC-1]['Vmin']))
            offset = self.DAC_limits[DAC-1]['Vmin']

        self.DAC_doffset[DAC-1] = offset
        intval = int((-4096/Vdif * offset) - 7)
        data = intval << 4

        DAC_IC = S5k_module.DAC_mapping[DAC][0]
        DAC_internal = S5k_module.DAC_mapping[DAC][1]

        if DAC_internal == 1:
            register = self.DAreg.DAC1DOF
        elif DAC_internal == 2:
            register = self.DAreg.DAC2DOF
        elif DAC_internal == 3:
            register = self.DAreg.DAC3DOF
        else:
            register = self.DAreg.DAC4DOF

        self.write_AD9106(register, data, DAC_IC)

    def run_module(self, run):
        """Starts the module

        Set all DAC outputs to run and start outputting a trigger. Can also
        be done by an external trigger in on the front of the module.

        Args:
            run (bool): Set module running True or False
        """
        # input checks
        if run not in range(2):
            raise ValueError("The 'run' parameter of {} is illegal. Possible values are 0 and 1.".format(run))
        self.module_running = run

        for i in [0, 1, 3, 4]:
            self.write_AD9106(self.DAreg.PAT_STATUS, run, i)

        # Check to see if internal (on-board) oscillator needs to be disabled.
        # Disabling is done using the 'external' clock option.
        if self.reference == 'internal':
            reference = 1
        else:
            reference = 0

        self.spi_rack.write_data(self.module, 5, BICPINS_MODE, BICPINS_SPEED, bytearray([(run<<7) | 2 | reference]))

    def get_run_status(self):
        """
        """
        print("The S5k run status is",self.module_running,"\n")
        #print("The trigger status is",)

    def upload_waveform(self, DAC, waveform, start_addr, set_pattern_length=True):
        """Upload waveform to selected DAC

        Args:
            DAC (int: 1-16): DAC which to upload to
            waveform (int array): integer array containing values between -2048 and 2047
            start_addr (int): location of first address to upload to
            set_pattern_length (bool): set the pattern length of the DAC to the uploaded waveform length
        """
        if len(waveform) + start_addr > 4096:
            raise ValueError('Waveform length + starting address exceeds RAM size! Max 4096.')

        # Make sure values don't exceed min or max
        new_waveform = np.zeros_like(waveform)
        np.clip(waveform, a_min=-2048, a_max=2047, out=new_waveform)

        if not np.array_equal(waveform, new_waveform):
            print("Warning! Values in the waveform exceed the minimum ({}) or ".format(-2048),
                  "maximum values ({}) and have been clipped.".format(2047))

        DAC_IC = S5k_module.DAC_mapping[DAC][0]

        # Shift into correct place for registers
        new_waveform = new_waveform<<4
        # Reverse waveform order, register in IC gets updated in Reverse
        new_waveform = np.flipud(new_waveform)

        # Create new array to fit all the bytes
        # Waveform data is 16-bit, but SPI data is 8-bit
        ordered_data = np.zeros(2*len(new_waveform), dtype=np.uint8)
        ordered_data[0::2] = (new_waveform >> 8) & 0xFF
        ordered_data[1::2] = new_waveform & 0xFF

        # Can send maximum of 60 data bytes (+ 2 registers bytes) at a timeout
        # Split up data array into groups of ~60 for maximum data transfer
        s_data = np.split(ordered_data, np.arange(60, len(ordered_data), 60))

        # Start addres. Is traversed in reverse direction so need to start
        # at the end
        start_addr += (0x6000 + len(new_waveform) - 1)

        self.run_module(False)
        self.write_AD9106(self.DAreg.PAT_STATUS, 1<<2, DAC_IC)

        # Write data points in groups of 30 (2 bytes each)
        for group in s_data:
            addr_bytes = bytearray([(start_addr >> 8) & 0xFF, start_addr & 0xFF])
            self.spi_rack.write_data(self.module, DAC_IC, AD9106_MODE, AD9106_SPEED,
                                     addr_bytes + group.tobytes())
            start_addr -= int(len(group)/2)

        self.write_AD9106(self.DAreg.PAT_STATUS, 0, DAC_IC)

        if set_pattern_length is True:
            self.set_pattern_length_DAC(DAC, len(waveform))

    def set_RAM_address(self, DAC, start_pos, stop_pos):
        """Set addresses for AWG mode

        Sets the start and stop address for the selected DAC in AWG mode. When the
        DAC is running it will output data from this address range.

        Args:
            DAC (int: 1-16): DAC of which to set the addresses
            start_pos (int): start address of waveform
            stop_pos (int): stop address of waveform
        """
        #start and stop both 12 bit
        if stop_pos > 4096:
            raise ValueError('Stop address {} is larger than max value 4096!'.format(stop_pos))
        elif start_pos > stop_pos:
            raise ValueError('Start address {} is larger than stop address {}!'. format(start_pos, stop_pos))

        DAC_IC = S5k_module.DAC_mapping[DAC][0]
        DAC_internal = S5k_module.DAC_mapping[DAC][1]

        if DAC_internal == 1:
            start_register = self.DAreg.START_ADDR1
            stop_register = self.DAreg.STOP_ADDR1
        elif DAC_internal == 2:
            start_register = self.DAreg.START_ADDR2
            stop_register = self.DAreg.STOP_ADDR2
        elif DAC_internal == 3:
            start_register = self.DAreg.START_ADDR3
            stop_register = self.DAreg.STOP_ADDR3
        else:
            start_register = self.DAreg.START_ADDR4
            stop_register = self.DAreg.STOP_ADDR4

        data = start_pos << 4
        self.write_AD9106(start_register, data, DAC_IC)
        data = stop_pos << 4
        self.write_AD9106(stop_register, data, DAC_IC)

    def set_pattern_length_DAC(self, DAC, length):
        # input checks
        if length > 65535:
            raise ValueError('Pattern length {} not allowed. Needs to be under or equal to 65536'.format(length))
        DAC_IC = S5k_module.DAC_mapping[DAC][0]
        self.write_AD9106(self.DAreg.PAT_PERIOD, length, DAC_IC)

    def set_pattern_length_trigger(self, length, S5k_version = 1.0):
        # Possible versions are:
        #     Version 1.0: jumper-given addressing, trigger length has a 14-bit limit
        #     Version 1.1: slot-based addressing, trigger length has a 16-bit limit
        # The module version is written on a sticker on the module side / front
        if S5k_version == 1.0:
            if length not in range(10, 4095):
                raise ValueError('Value {} not allowed. Needs to be between 10 and 4095'.format(length))
        elif S5k_version == 1.1:
            if length not in range(10, 16383):
                raise ValueError('Value {} not allowed. Needs to be between 10 and 16383'.format(length))
        else:
            raise ValueError('S5k module version {} is not recognized. Can be 1.0 or 1.1'.format(S5k_version))

        b1 = (length>>8) & 0xFF
        b2 = length & 0xFF
        s_data = bytearray([b1, b2])

        self.spi_rack.write_data(self.module, 7, BICPINS_MODE, BICPINS_SPEED, s_data)

    def set_clock_source(self, source):
        possible_values = {'internal':0, 'external':1}
        if source not in possible_values:
            raise ValueError('Value {} does not exist. Possible values are: {}'.format(source, possible_values))

        self.reference = source
        self.write_LMK_data(14, (1<<30) | (possible_values[source]<<29) |(1<<27))
        self.sync_clock()

    def sync_clock(self):
        # Check to see if internal (on-board) oscillator needs to be disabled.
        # Disabling is done using the 'external' clock option.
        if self.reference == 'internal':
            reference = 1
        else:
            reference = 0

        # Toggles the sync pin on the clock distribution/divider IC to sync
        # up all the clocks. Necessary after any clock change
        self.spi_rack.write_data(self.module, 5, BICPINS_MODE, BICPINS_SPEED,
                                 bytearray([(self.module_running<<7) | 2 | reference]))
        self.spi_rack.write_data(self.module, 5, BICPINS_MODE, BICPINS_SPEED,
                                 bytearray([(self.module_running<<7) | 0 | reference]))
        self.spi_rack.write_data(self.module, 5, BICPINS_MODE, BICPINS_SPEED,
                                 bytearray([(self.module_running<<7) | 2 | reference]))

    def set_clock_division(self, DAC, divisor):
        allowed_values = [1] + list(range(2, 512, 2))
        if divisor not in allowed_values:
            raise ValueError('Allowed values are: 1, 2, 4, 6, 8, ..., 510')

        if divisor == 1:
            data = 1<<16 | 0<<17 | 0<<8
        else:
            data = 1<<16 | 1<<17 | int((divisor/2))<<8

        DAC_IC = S5k_module.DAC_mapping[DAC][0]
        # Provide the correct LMK register for each DAC_IC (each AWG chip) with its own SPI address
        LMK_reg = {0:6, 1:7, 3:0, 4:1}
        self.write_LMK_data(LMK_reg[DAC_IC], data)

        # Update the values in the DAC_clock_div array for all affected DACs
        for i in range(0, 16, 4):
            if DAC-1 in range(i, i+4):
                self.DAC_clock_div[i:i+4] = 4*[divisor]     # A sub-array of 4 consecutive DACs is updated with the same 'divisor'

        # Update the clock to the BIC to be the slowest used clock at the moment
        # This is the clock with the largest divider (max_divisor).
        max_divisor = max(self.DAC_clock_div)
        if max_divisor == 1:
            data = 1<<16 | 0<<17 | 0<<8
        else:
            data = 1<<16 | 1<<17 | int((max_divisor/2))<<8
        self.write_LMK_data(3, data)

        # Synchronise all the clocks
        self.sync_clock()

    def get_all_AWG_registers(self, DAC):
        """read back all 68 registers from one of the AD9106 chips.
           The function self-calculates which AWG chip is referenced,
           based on the given DAC channel.
           Note:
            This is the same as the existing 'get_all_DAC_registers' function.
            This function is preferred since the name is clearer.
        """
        self.get_all_DAC_registers(DAC)

    def get_all_DAC_registers(self, DAC):
        """read back all 68 registers from the AD9106 chip of a requested DAC
           Note:
            This in fact reads the registers from an AWG chip, so 4 DACs.
            Keeping this function for back compatibility.
        """
        DAC_IC = S5k_module.DAC_mapping[DAC][0]        # i.e. the AWG chip
        Di = S5k_module.DAC_mapping[DAC][1]            # a.k.a. DAC_internal
        
        regs = list(dir(self.DAreg)[0:68])
        regs.remove('SRAM_DATA')
        
        print("\nDAC",DAC,"is produced by AWG (DAC_IC) number",DAC_IC,"channel",Di,".",
        "The AWG register data is:")
        for register in regs:
            reg_addr = self.DAreg.__dict__[register]
            DAC_raw_data = self.read_AD9106(reg_addr, DAC_IC)
            print("    Register",register,"data: ",DAC_raw_data)

    def get_one_DAC_registers(self, DAC):
        """read back from the AD9106 
           only the registers referring to a particular DAC channel
        """
        DAC_IC = S5k_module.DAC_mapping[DAC][0]
        Di = S5k_module.DAC_mapping[DAC][1]            # a.k.a. DAC_internal
        
        regs = list(dir(self.DAreg)[0:68])             # all AWG registers
        DAC_regs = regs[0:3]+[regs[8-Di]]+[regs[8]]+[regs[13-Di]]\
        +regs[13:18]+[regs[23-Di]]+[regs[25-int((Di+1)/2)]]+regs[25:26]\
        +[regs[29-int((Di+1)/2)]]+regs[29:30]+[regs[35-Di]]+[regs[39-Di]]\
        +[regs[41-int((Di+1)/2)]]+regs[41:42]+[regs[47-Di]]+regs[47:49]\
        +regs[62-4*Di : 66-4*Di]+regs[66:70]
        regs.remove('SRAM_DATA')
        
        print("\nFor DAC",DAC,", produced by DAC_IC",DAC_IC,", the register data is:")
        for register in DAC_regs:
            reg_addr = self.DAreg.__dict__[register]
            DAC_raw_data = self.read_AD9106(reg_addr, DAC_IC)
            print("    Register",register,"data: ",DAC_raw_data)

    def reset_DACs(self):
        """
        """        
        # This operation must start with resetting the RAM
        # because with the reversed order
        # some registers would get set again upon calling 'upload_waveform'
        reset_waveform = [0]*4096
        for DAC_NUM in [1,5,9,13]:   # Covering all DAC_ICs, but referring indirectly via DACs and not DAC_ICs
            self.upload_waveform(DAC = DAC_NUM, waveform = reset_waveform, start_addr = 0, set_pattern_length = True)

        for DAC_IC in list(range(4)):
            # registers for all 4 channels
            self.write_AD9106(self.DAreg.PAT_PERIOD , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.PAT_STATUS, 0x0000, DAC_IC)

            # registers for 2 channels
            self.write_AD9106(self.DAreg.WAV2_1CONFIG , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.WAV4_3CONFIG , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.SAW2_1CONFIG , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.SAW4_3CONFIG , 0x0000, DAC_IC)

            # registers for 1 channel
            self.write_AD9106(self.DAreg.DAC1_CST , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC2_CST , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC3_CST , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC4_CST , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC1_DGAIN , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC2_DGAIN , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC3_DGAIN , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC4_DGAIN , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC1DOF , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC2DOF , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC3DOF , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.DAC4DOF , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.START_DLY1 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.START_DLY2 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.START_DLY3 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.START_DLY4 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.START_ADDR1 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.START_ADDR2 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.START_ADDR3 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.START_ADDR4 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.STOP_ADDR1 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.STOP_ADDR2 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.STOP_ADDR3 , 0x0000, DAC_IC)
            self.write_AD9106(self.DAreg.STOP_ADDR4 , 0x0000, DAC_IC)

    def write_LMK_data(self, register, data):
        b1 = (data>>24) & 0xFF
        b2 = (data>>16) & 0xFF
        b3 = (data>>8) & 0xFF
        b4 = (data&0xFF) | (register & 0xF)

        s_data = bytearray([b1, b2, b3, b4])
        self.spi_rack.write_data(self.module, 2, LMK01010_MODE, LMK01010_SPEED, s_data)

    def write_AD9106(self, register, data, SPI_addr):
        b1 = (register>>8) & 0xFF
        b2 = register & 0xFF
        b3 = (data>>8) & 0xFF
        b4 = data&0xFF
        s_data = bytearray([b1, b2, b3, b4])
        self.spi_rack.write_data(self.module, SPI_addr, AD9106_MODE, AD9106_SPEED, s_data)

        #update
        s_data = bytearray([0, self.DAreg.RAMUPDATE, 0, 1])
        self.spi_rack.write_data(self.module, SPI_addr, AD9106_MODE, AD9106_SPEED, s_data)

    def read_AD9106(self, register, SPI_addr):
        b1 = 1<<7
        b2 = register
        s_data = bytearray([b1, b2, 0, 0])
        r_data = self.spi_rack.read_data(self.module, SPI_addr, AD9106_MODE, AD9106_RD_SPEED, s_data)

        return int.from_bytes(r_data[2:4], byteorder='big')

class AD9106_registers:
    SPICONFIG = 0x00
    POWERCONFIG = 0x01
    CLOCKCONFIG = 0x02
    REFADJ = 0x03
    DAC4AGAIN = 0x04
    DAC3AGAIN = 0x05
    DAC2AGAIN = 0x06
    DAC1AGAIN = 0x07
    DACxRANGE = 0x08      # Gain range control for all 4 outputs
    DAC4RSET = 0x09       # Settings for the value of Rset resistor
    DAC3RSET = 0x0A       # Settings for the value of Rset resistor
    DAC2RSET = 0x0B       # Settings for the value of Rset resistor
    DAC1RSET = 0x0C       # Settings for the value of Rset resistor
    CALCONFIG = 0x0D      # Used for calibration
    COMPOFFSET = 0x0E     # Used for calibration
    RAMUPDATE = 0x1D
    PAT_STATUS = 0x1E     # Starts/stops pattern generation, and flag for pattern status
    PAT_TYPE = 0x1F       # Determines whether the pattern runs continuously or a fixed number of times
    PATTERN_DLY = 0x20    # Delay from trigger to start
    DAC4DOF = 0x22        # Digital offset
    DAC3DOF = 0x23        # Digital offset
    DAC2DOF = 0x24        # Digital offset
    DAC1DOF = 0x25        # Digital offset
    WAV4_3CONFIG = 0x26   # Mode select (DC / Sawtooth / noise / DDS), and waveform source
    WAV2_1CONFIG = 0x27   # Mode select (DC / Sawtooth / noise / DDS), waveform source, and channel summing options
    PAT_TIMEBASE = 0x28
    PAT_PERIOD = 0x29
    DAC4_3PATx = 0x2A     # Number of pattern repetitions 
    DAC2_1PATx = 0x2B     # Number of pattern repetitions
    DOUT_START_DLY = 0x2C # Controls for the DOUT marker signal
    DOUT_CONFIG = 0x2D    # Controls for the DOUT marker signal
    DAC4_CST = 0x2E       # Value for 'constant' mode
    DAC3_CST = 0x2F       # Value for 'constant' mode
    DAC2_CST = 0x30       # Value for 'constant' mode
    DAC1_CST = 0x31       # Value for 'constant' mode
    DAC4_DGAIN = 0x32
    DAC3_DGAIN = 0x33
    DAC2_DGAIN = 0x34
    DAC1_DGAIN = 0x35
    SAW4_3CONFIG = 0x36   # Configuration (type & samples/step) for a sawtooth mode
    SAW2_1CONFIG = 0x37   # Configuration (type & samples/step) for a sawtooth mode
    DDS_TW32 = 0x3E       # Tuning word for DDR mode, part 2
    DDS_TW1 = 0x3F        # Tuning word for DDR mode, part 1
    DDS4_PW = 0x40        # Phase offset for DDR mode
    DDS3_PW = 0x41        # Phase offset for DDR mode
    DDS2_PW = 0x42        # Phase offset for DDR mode
    DDS1_PW = 0x43        # Phase offset for DDR mode
    TRIG_TW_SEL = 0x44    # Sets the trigger delay to equal the first-repetition delay ('start delay')
    DDSx_CONFIG = 0x45    # Controls for DDS mode
    TW_RAM_CONFIG = 0x47  # Controls for DDS mode
    START_DLY4 = 0x50
    START_ADDR4 = 0x51
    STOP_ADDR4 = 0x52
    DDS_CYC4 = 0x53
    START_DLY3 = 0x54
    START_ADDR3 = 0x55
    STOP_ADDR3 = 0x56
    DDS_CYC3 = 0x57
    START_DLY2 = 0x58
    START_ADDR2 = 0x59
    STOP_ADDR2 = 0x5A
    DDS_CYC2 = 0x5B
    START_DLY1 = 0x5C
    START_ADDR1 = 0x5D
    STOP_ADDR1 = 0x5E
    DDS_CYC1 = 0x5F
    CFG_ERROR = 0x60      # Flags for various errors for the user
    SRAM_DATA = 0x6000
