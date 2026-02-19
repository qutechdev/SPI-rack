"""ADC module D4a interface

SPI Rack interface code for the D4 module.

Example:
    Example use: ::
        D4a = spirack.D4a_modules(SPI_Rack1, 5)

Code version: 10
"""

from .chip_mode import AD7175_MODE, AD7175_SPEED                 # for ADC
from .chip_mode import HCT595_MODE, HCT595_SPEED                 # for SR (shift register)
from .chip_mode import BICPINS_SPEED                             # for GPIO
import time

class D4a_module(object):
    """D4a module interface class

    This class does the low level interfacing with the D4a module. When creating
    an instance, it requires a SPI_rack class passed as a parameter.

    The module contains two independent 24-bit analog to digital converters. They
    can be individually configured and triggered. The filter settings determine
    the data rate and resolution. For an overview of settings, see the website.

    Attributes:
        module (int): module number set by the user (must coincide with hardware)
        filter_setting (int): current filter setting
        filter_type (string): filter type, either sinc3 or sinc5
    """
	
    def __init__(self, spi_rack, module, switch_delay = 0.1, initialize = 1):
        """Inits D4a module class

        The D4a_module class needs an SPI_rack object at initiation. All
        communication will run via that class. At initialization the ADC filters
        will be set to 'sinc3' at 16.67 SPS.

        Args:
            spi_rack (SPI_rack object): SPI_rack class object via which the communication runs
            module (int): module number set on the hardware
        """
        self.file = os.path.abspath(__file__)
        print("D4a initialized.\nFile path is", self.file,"\n")      # debug only
        
        self.module = module
        self.spi_rack = spi_rack

        self.reg = AD7175_registers

        self.filter_setting = None
        self.filter_type = None

        # remote_settings is a byte containing settings 
        # for overload (OL), feedback (FB) and the muxes (1P4T switches).
		# The bit order corresponds to the connectivity of the Shift-Regsiter outputs
        # Bits are:
        # 
        #      7 (msb)       6          5        4           3         2         1      0 (lsb)
        # 
        #      SWB_A0     SWB_A1_0    B_OL    SWB_A1_1   SWT_A1_1   SWT_A0    SWT_A1_0   T_OL
        #     "Bottom"    "Bottom"  "Bottom"  "Bottom"    "Top"     "Top"      "Top"     "Top"
        #        1           0          0        0          0         1          0         0
        # 
        self.remote_settings = 0x42   # 0b 0100 0010
        self.spi_rack.write_data(self.module, 2, HCT595_MODE, HCT595_SPEED, bytearray([self.remote_settings]))
        
        self.bic_sync_value = 0     # initializing to no sync given
        
        self._default_setup()       # cont. mode, single cycle, chnn 0 and set-up 0, AIN3-vs-AIN2

        for adc in range(0, 2):
        
            # Check communication with the ADC
            print("\nChecking communication with ADC",adc,":")
            Received_ID = self.get_ADC_ID(adc)
            print("                   ADC ID is",Received_ID,
            ",",hex(Received_ID),".\n",
            "      The expected value is 3294 , 0xCDX.")
            
            # Set filter to sinc3 and 20
            self.set_filter(adc, 'sinc3', 20)
            
            # Disable sync mode
            self.configure_sync_mode(adc, 0)
            
            # Turn off over-load indicators
            self.config_led(adc, 0)
        
        self.SR_OEb         = 0     # initializing the SR output_enable\ to 'enabled'

    def continuous_conversion_trig_and_read(self, adc):
        """Perform a conversion

        Performs a conversion on the given ADC. It will both trigger the ADC and
        wait for the result. Because of this it will block all other operations.

        Args:
            adc (int:0-1): ADC to perform the conversion
        """
        self.launch_continuous_conversion(adc)
        return self.get_finalized_result(adc)

    def single_conversion_trig_and_read(self, adc):
        """Perform a true-single conversion

        Performs a conversion on the given ADC. It will both trigger the ADC for one
        conversion only (single- vs. continuous-mode ) and wait for its result.
        Because of this it will block all other operations.

        Args:
            adc (int:0-1): ADC to perform the conversion
        """
        self.launch_one_conversion(adc)
        return self.get_finalized_result(adc)

    def launch_continuous_conversion(self, adc):
        """Triggers a continuous conversion

        Triggers a conversion on the given ADC. Does not wait for the result. This
        should be used if multiple devices/adcs need to convert in parallel. After the conversion
        is done it will immediately continue doing conversions and updating the
        output.

        Args:
            adc (int:0-1): ADC to perform the conversion
        """
        self._write_data_16(adc, self.reg.adcMODE_REG,
                                (0<<self.reg.MODE) |
                                (1<<self.reg.SING_CYC))

    def launch_one_conversion(self, adc):
        """Triggers a single conversion

        Args:
            adc (int:0-1): ADC to perform the conversion
        """
        self._write_data_16(adc, self.reg.adcMODE_REG,
                                (1<<self.reg.MODE) |
                                (1<<self.reg.SING_CYC))

    def get_finalized_result(self, adc):
        """Returns the result of a conversion

        Returns the result from an executed conversion. The function will wait until the
        result is present, therefore it will block all other operations.

        It will return the last conversion result. If the time between the launching the conversion
        and reading the result is too long, the result may be of a second/third conversion. (in 
        continuous mode, where the ADC keeps converting and updating the data output)

        Args:
            adc (int:0-1): ADC to readout
        Returns:
            ADC measured voltage (float)
        """
        self.wait_for_data(adc)
        raw_res = self.force_immediate_read_out(adc)
        checked_res = self.overload_check(adc, raw_res)
        return checked_res
        
    def wait_for_data(self, adc):
        """Pauses further execution until current conversion is completed
        """
        running = True
        while running:
            status = self._read_data(adc, self.reg.STATUS_REG, 1)
            # if new data available:
            if (status[0]&0x80) == 0:
                running = False
        return

    def force_immediate_read_out(self, adc): 
        """Reads the ADC data output register, regardless of its RDY/complete status
		
		Get raw data, shift it to correct place, and convert to voltage
        Args:
            adc (int:0-1): ADC to readout
        Returns:
            ADC measured voltage (float)
        force_read_out = force_get_results
        """
        raw_data = self._read_data(adc, self.reg.DATA_REG, 3)
        raw_data = raw_data[1:]
        raw_data_val = raw_data[0] << 16 | raw_data[1] << 8 | raw_data[2]
        return self.calculate_ADC_voltage_from_ADC_binary(raw_data_val)


    def calculate_ADC_voltage_from_ADC_binary(self, bin_data):
        """

        """
        return (bin_data * 2 * 0.875 / 2**22) - 3.5     # D4a v2-1 using a 3.5V reference


    def overload_check(self, adc, raw_res):
        """

        """
        if (raw_res > 3.999999 or raw_res < -3.994):
		    # Indicate overload condition to user:
		    # print warning, turn on LED until reset by user, and replace result
            print("    Warning: result", raw_res,"is outside the supported range.\n    Data will be replaced by '999'")
            self.config_led(adc, 1)
            return 999
        else:
            return raw_res

    
    def config_led(self, led, on_off):
        """
        """
        if led not in range(2):
            raise ValueError('LED number {} does not exist. Possible values are 0 and 1.'.format(adc))
        if on_off not in range(2):
            raise ValueError('Value {} is not legal for LED setting. Requiring a Bool value.'.format(on_off))

        if (led==0):
            # Setting the 'Top' mux
            self.remote_settings &= 0xfe       # reset SR bit 0 (T_OL)
            mux_new_settings = on_off<<0       # shift new val to bit 0
        elif (led==1):
            # Setting the 'Bottom' mux
            self.remote_settings &= 0xDf       # reset SR bit 5 (B_OL)
            mux_new_settings = on_off<<5       # shift new val to bit 5
        self.remote_settings |= mux_new_settings
        self.spi_rack.write_data(self.module, 2, HCT595_MODE, HCT595_SPEED, bytearray([self.remote_settings]))

    def select_measurement_type(self, adc, measurement_type):
        """Sets the D4a input-path mux 
        (connects the selected signal to the selected ADC)
        and updates the 'remote_settings' byte
        
		Options are 0 - GND, 1 - cal ref level , 2 - internal header pin, 3 - front-panel connector
        
        Formerly called 'config_mux' (a.k.a. 'connect_signals_to_adc')
        """
        # input checks
        if adc not in range(2):
            raise ValueError('ADC number {} does not exist. Possible values are 0 and 1.'.format(adc))
        
        self.measurement_types = ['single-ended calibration','differential calibration',
                             'single-ended measurement','grounding',
                             'back-panel single-ended measurement','back-panel complementary single-ended measurement',
                             'back-panel differential measurement',
                             'test_msb_is_1','test_lsb_is_1']
        if measurement_type not in self.measurement_types:
            raise ValueError('Measurement type \'{}\' does not exist. \nPossible values are {}.'.format(measurement_type,self.measurement_types))
        
        print("select_measurement_type:",
              "\n         Instruction was given for ADC",adc,
              "\n         Initial SR remote_settings is",self.remote_settings,
              "\n         (as bits:",bin(self.remote_settings),")")                  # debug
        
        if (adc==0):
            # Setting the 'Top' mux
            if (measurement_type=='test_lsb_is_1'):
                self.remote_settings = 0x01          # testing only (used for debug)
            if (measurement_type=='test_msb_is_1'):
                self.remote_settings = 0x08          # testing only (used for debug)
            if (measurement_type=='single-ended calibration'):
                #SWT_A0,A1_0,A1_1: 0 0 0
                self.remote_settings &= 0xfb         # config SWT_A0 (bit 2)
                self.remote_settings &= 0xfd         # config SWT_A1_0 (bit 1)
                self.remote_settings &= 0xf7         # config SWT_A1_1 (bit 3)
            if (measurement_type=='differential calibration'):
                #SWT_A0,A1_0,A1_1: 0 0 1
                self.remote_settings &= 0xfb         # config SWT_A0 (bit 2)
                self.remote_settings &= 0xfd         # config SWT_A1_0 (bit 1)
                self.remote_settings |= 0x08         # config SWT_A1_1 (bit 3)
            if (measurement_type=='single-ended measurement'):
                #SWT_A0,A1_0,A1_1: 0 1 0
                self.remote_settings &= 0xfb         # config SWT_A0 (bit 2)
                self.remote_settings |= 0x02         # config SWT_A1_0 (bit 1)
                self.remote_settings &= 0xf7         # config SWT_A1_1 (bit 3)
            if (measurement_type=='grounding'):
                #SWT_A0,A1_0,A1_1: 1 0 0
                self.remote_settings |= 0x04         # config SWT_A0 (bit 2)
                self.remote_settings &= 0xfd         # config SWT_A1_0 (bit 1)
                self.remote_settings &= 0xf7         # config SWT_A1_1 (bit 3)
            if (measurement_type=='back-panel single-ended measurement'):
                #SWT_A0,A1_0,A1_1: 1 1 0
                self.remote_settings |= 0x04         # config SWT_A0 (bit 2)
                self.remote_settings |= 0x02         # config SWT_A1_0 (bit 1)
                self.remote_settings &= 0xf7         # config SWT_A1_1 (bit 3)
            if (measurement_type=='back-panel complementary single-ended measurement'):
                #SWT_A0,A1_0,A1_1: 1 0 1
                self.remote_settings |= 0x04         # config SWT_A0 (bit 2)
                self.remote_settings &= 0xfd         # config SWT_A1_0 (bit 1)
                self.remote_settings |= 0x08         # config SWT_A1_1 (bit 3)
            if (measurement_type=='back-panel differential measurement'):
                #SWT_A0,A1_0,A1_1: 1 1 1
                self.remote_settings |= 0x04         # config SWT_A0 (bit 2)
                self.remote_settings |= 0x02         # config SWT_A1_0 (bit 1)
                self.remote_settings |= 0x08         # config SWT_A1_1 (bit 3)
        if (adc==1):
            # Setting the 'Bottom' mux
            if (measurement_type=='test_lsb_is_1'):
                self.remote_settings = 0x10          # testing only (used for debug)
            if (measurement_type=='test_msb_is_1'):
                self.remote_settings = 0x80          # testing only (used for debug)
            if (measurement_type=='single-ended calibration'):
                #SWB_A0,A1_0,A1_1: 0 0 0
                self.remote_settings &= 0x7f         # config SWB_A0 (bit 7)
                self.remote_settings &= 0xbf         # config SWB_A1_0 (bit 6)
                self.remote_settings &= 0xef         # config SWB_A1_1 (bit 4)
            if (measurement_type=='differential calibration'):
                #SWB_A0,A1_0,A1_1: 0 0 1
                self.remote_settings &= 0x7f         # config SWB_A0 (bit 7)
                self.remote_settings &= 0xbf         # config SWB_A1_0 (bit 6)
                self.remote_settings |= 0x10         # config SWB_A1_1 (bit 4)
            if (measurement_type=='single-ended measurement'):
                #SWB_A0,A1_0,A1_1: 0 1 0
                self.remote_settings &= 0x7f         # config SWB_A0 (bit 7)
                self.remote_settings |= 0x40         # config SWB_A1_0 (bit 6)
                self.remote_settings &= 0xef         # config SWB_A1_1 (bit 4)
            if (measurement_type=='grounding'):
                #SWB_A0,A1_0,A1_1: 1 0 0
                self.remote_settings |= 0x80         # config SWB_A0 (bit 7)
                self.remote_settings &= 0xbf         # config SWB_A1_0 (bit 6)
                self.remote_settings &= 0xef         # config SWB_A1_1 (bit 4)
            if (measurement_type=='back-panel single-ended measurement'):
                #SWB_A0,A1_0,A1_1: 1 1 0
                self.remote_settings |= 0x80         # config SWB_A0 (bit 7)
                self.remote_settings |= 0x40         # config SWB_A1_0 (bit 6)
                self.remote_settings &= 0xef         # config SWB_A1_1 (bit 4)
            if (measurement_type=='back-panel complementary single-ended measurement'):
                #SWB_A0,A1_0,A1_1: 1 0 1
                self.remote_settings |= 0x80         # config SWB_A0 (bit 7)
                self.remote_settings &= 0xbf         # config SWB_A1_0 (bit 6)
                self.remote_settings |= 0x10         # config SWB_A1_1 (bit 4)
            if (measurement_type=='back-panel differential measurement'):
                #SWB_A0,A1_0,A1_1: 1 1 1
                self.remote_settings |= 0x8f         # config SWB_A0 (bit 7)
                self.remote_settings |= 0x4f         # config SWB_A1_0 (bit 6)
                self.remote_settings |= 0x1f         # config SWB_A1_1 (bit 4)
        
        # finaly, write the data
        self.spi_rack.write_data(self.module, 2, HCT595_MODE, HCT595_SPEED, bytearray([self.remote_settings]))


    def select_ADC_inputs(self, adc, pos_input_name, neg_input_name):
        """Set the ADCs to read from desired line
		
 		This updates the config resgisters on the ADC chip itself.

        Args:
            adc (int:0-1): ADC which receives the signal to sample
            pos_input_name: the D4a internal signal used as the non-inverting signal
            neg_input_name: the D4a internal signal used as the inverting signal
			(see numbering of connectors on front-panel)
        """
        # input checks
        if (pos_input_name == neg_input_name):
            raise ValueError('Cannot set both phases to the same value. Requested set for pos_input_line and neg_input_line was {}'.format(pos_input_name))
        accepted_names = ['REF-', 'REF+', 'POS', 'NEG', 'gnd']
        if pos_input_name not in accepted_names:
            raise ValueError('pos_input_line {} does not exist.'.format(pos_input_name))
        if neg_input_name not in accepted_names:
            raise ValueError('neg_input_line {} does not exist.'.format(neg_input_name))
        
        # choosing the non-inverting ("positive") signal
        if (pos_input_name=='REF-'):
            # Set the ADC positive to read REF-
            self.read_modify_write_CH0_AINPOS0(adc, 22)
        elif (pos_input_name=='REF+'):
            # Set the ADC positive to read REF+
            self.read_modify_write_CH0_AINPOS0(adc, 21)
        elif (pos_input_name=='NEG'):
            # Set the ADC positive to read from AIN2
            self.read_modify_write_CH0_AINPOS0(adc, 2)
        elif (pos_input_name=='POS'):
            # Set the ADC positive to read from AIN3
            self.read_modify_write_CH0_AINPOS0(adc, 3)
        elif (pos_input_name=='gnd'):
            # Set the ADC positive to read from AIN4
            self.read_modify_write_CH0_AINPOS0(adc, 4)
		
        # choosing the inverting ("negative") signal
        if (neg_input_name=='REF-'):
            # Set the ADC negative to read REF-
            self.read_modify_write_CH0_AINNEG0(adc, 22)
        elif (neg_input_name=='REF+'):
            # Set the ADC negative to read REF+
            self.read_modify_write_CH0_AINNEG0(adc, 21)
        elif (neg_input_name=='NEG'):
            # Set the ADC negative to read from AIN2
            self.read_modify_write_CH0_AINNEG0(adc, 2)
        elif (neg_input_name=='POS'):
            # Set the ADC negative to read from AIN3
            self.read_modify_write_CH0_AINNEG0(adc, 3)
        elif (neg_input_name=='gnd'):
            # Set the ADC negative to read from AIN4
            self.read_modify_write_CH0_AINNEG0(adc, 4)
			
        return

    def read_modify_write_CH0_AINPOS0(self, adc, adc_ain_pin):
        """
        """
        CH0_REG_data = self._read_data(adc, self.reg.CH0_REG, 2)
        CH0_REG_data = CH0_REG_data[1:]
		
        # AINPOS0 - update
        byte0 = CH0_REG_data[0]&0xfc               # clear 2 lsbs
        byte0_new_2msbs = ((adc_ain_pin&0x18)>>3)  # calculate new 2 lsbs
        byte0 |= byte0_new_2msbs                   # place new lsbs into the 2 lsb positions
        byte1 = CH0_REG_data[1]&0x1f               # clear 3 msbs
        byte1_new_3lsbs = (adc_ain_pin&0x07)<<5    # calculate new 3 msbs
        byte1 |= byte1_new_3lsbs                   # place new msbs into the 3 msb positions

        wr_data = byte0<<8 | byte1
        self._write_data_16(adc, self.reg.CH0_REG, wr_data)

    def read_modify_write_CH0_AINNEG0(self, adc, adc_ain_pin):
        """
        """
        CH0_REG_data = self._read_data(adc, self.reg.CH0_REG, 2)
        CH0_REG_data = CH0_REG_data[1:]
		
        # AINNEG0 - update
        byte0 = CH0_REG_data[0]
        byte1 = CH0_REG_data[1]&0xe0               # clear 5 lsbs
        byte1_new_5lsbs = (adc_ain_pin&0x1f)       # calculate new 5 lsbs
        byte1 |= byte1_new_5lsbs                   # place new lsbs into the 5 lsb positions

        wr_data = byte0<<8 | byte1
        self._write_data_16(adc, self.reg.CH0_REG, wr_data)

    def set_single_cycle(self, adc, sing_cyc):
        """
        """
        # input checks
        if adc not in range(2):
            raise ValueError('ADC number {} does not exist. Possible values are 0 and 1.'.format(adc))
        if sing_cyc not in [0,1]:
            raise ValueError('The bit value {} does not exist. Possible values are 0 or 1'.format(sing_cyc))
        
        ADCMODE_data = self._read_data(adc, self.reg.adcMODE_REG, 2)
        ADCMODE_data = ADCMODE_data[1:]
		
        byte0 = ADCMODE_data[0] & 0xdf          # clear existing ref buffer setting
        buf_bit = (sing_cyc & 0xff) << 5        # calculate new bit
        byte0 |= buf_bit                        # place new bits into position
        byte1 = ADCMODE_data[1]

        wr_data = byte0<<8 | byte1
        self._write_data_16(adc, self.reg.adcMODE_REG, wr_data)

    def set_ref_input_buffers(self, adc, buf_setting):
        """
        """
        # input checks
        if adc not in range(2):
            raise ValueError('ADC number {} does not exist. Possible values are 0 and 1.'.format(adc))
        if buf_setting not in ['enabled','disabled']:
            raise ValueError("ADC number {} does not exist. Possible values are 'enabled','disabled'".format(buf_setting[0],buf_setting[1]))
        
        SETUPCON0_data = self._read_data(adc, self.reg.SETUPCON0_REG, 2)
        SETUPCON0_data = SETUPCON0_data[1:]
		
        if (buf_setting=='enabled'):
            wr_bit = 1
        elif (buf_setting=='disabled'):
            wr_bit = 0
        else:
            print("Error")

        byte0 = SETUPCON0_data[0] & 0xf3          # clear existing ref buffer setting
        buf_bits = ((wr_bit*3) & 0xff) << 2       # calculate 2 new bits
        byte0 |= buf_bits                         # place new bits into position
        byte1 = SETUPCON0_data[1]

        wr_data = byte0<<8 | byte1
        self._write_data_16(adc, self.reg.SETUPCON0_REG, wr_data)

    def set_anlg_input_buffers(self, adc, buf_setting):
        """
        """
        # input checks
        if adc not in range(2):
            raise ValueError('ADC number {} does not exist. Possible values are 0 and 1.'.format(adc))
        if buf_setting not in ['enabled','disabled']:
            raise ValueError('ADC number {} does not exist. Possible values are {} and {}.'.format(buf_setting[0],buf_setting[1]))
        
        SETUPCON0_data = self._read_data(adc, self.reg.SETUPCON0_REG, 2)
        SETUPCON0_data = SETUPCON0_data[1:]
		
        if (buf_setting=='enabled'):
            wr_bit = 1
        elif (buf_setting=='disabled'):
            wr_bit = 0
        else:
            print("Error")

        byte0 = SETUPCON0_data[0] & 0xfc          # clear existing ref buffer setting
        buf_bits = ((wr_bit*3) & 0xff)            # calculate 2 new bits
        byte0 |= buf_bits                         # place new bits into position
        byte1 = SETUPCON0_data[1]

        wr_data = byte0<<8 | byte1
        self._write_data_16(adc, self.reg.SETUPCON0_REG, wr_data)

    def save_remote_settings(self):
        """
        """
        initial_settings = self.remote_settings
        filter_setting = self.filter_setting
        filter_type = self.filter_type
        return([initial_settings, filter_setting, filter_type])

    def restore_remote_settings(self, adc, initial_conditions):
        """
        """
        
		# Restore remote settings
        initial_settings = initial_conditions[0]
        self.remote_settings = initial_settings
        self.spi_rack.write_data(self.module, 2, HCT595_MODE, HCT595_SPEED, bytearray([self.remote_settings]))
		
        # Restore filter settings
        filter_setting = initial_conditions[1]
        filter_type = initial_conditions[2]
        self.set_filter(adc, filter_type, filter_setting)

    def offset_calibration_with_module_reference(self, adc):
        """Offset voltage calibration routine

        Calibrates the offset of the given ADC input.  
        This uses the internal GND, so no need to short on front-panel.

        Args:
            adc (int:0-1): ADC to calibrate
        """

        # Save initial remote settings
        initial_settings = self.remote_settings
        # Save filter settings
        filter_setting = self.filter_setting
        filter_type = self.filter_type

        # Set calibration conditions
        #     Setting input to GND
        self.select_measurement_type(0, 'grounding')
        #     Set to best performance for offset calibration
        self.set_filter(adc, 'sinc3', 20)

        # Calibrate
        #     6 is 'System offset calibration' - valid for the present channel and configuration
        self._write_data_16(adc, self.reg.adcMODE_REG,
                                (6<<self.reg.MODE) |
								(1<<self.reg.SING_CYC))
        self.wait_for_data(adc)
        #     0 is 'Continuous conversion mode'
        self._write_data_16(adc, self.reg.adcMODE_REG,
                                (0<<self.reg.MODE) |
								(1<<self.reg.SING_CYC))

        # Restore filter settings
        self.set_filter(adc, filter_type, filter_setting)
		# Restore remote settings
        self.remote_settings = initial_settings
        self.spi_rack.write_data(self.module, 2, HCT595_MODE, HCT595_SPEED, bytearray([self.remote_settings]))

    def offset_calibration_with_external_reference(self, adc):
        """Offset voltage calibration routine

        Calibrates the offset of the given ADC input. To run this routine, put
        a short or 50 Ohm short on the input of the given ADC.

        Args:
            adc (int:0-1): ADC to calibrate
        """
        print('Make sure that ADC{} inputs are terminated with a short or 50 Ohm to GND'
              'while running this calibration!'.format(adc+1))
        
        # Save initial conditions
        initial_conditions = self.save_remote_settings()

        # Set calibration conditions
        #     Setting input to 'single-ended measurement'
        self.select_measurement_type(adc, 'single-ended measurement')
        # set to best performance for offset calibration
        self.set_filter(adc, 'sinc3', 20)

        # Calibrate
        #     6 is 'System offset calibration' - valid for the present channel and configuration
        self._write_data_16(adc, self.reg.adcMODE_REG,
                                (6<<self.reg.MODE) |
								(1<<self.reg.SING_CYC))
        self.wait_for_data(adc)
        #     0 is 'Continuous conversion mode'
        self._write_data_16(adc, self.reg.adcMODE_REG,
                                (0<<self.reg.MODE) |
                                (1<<self.reg.SING_CYC))

		# Restore initial conditions
        self.restore_remote_settings(adc, initial_conditions)

    def gain_calibration_with_module_reference(self, adc):
        """Gain calibration routine

        Calibrates the gain of the given ADC input.  
        This uses the in-module CAL_REF, so no need for connection on front-panel.
        Sets the input-path mux to the 'CAL_REF' position,
        and then restores its initial connection.
        
        Args:
            adc (int:0-1): ADC to calibrate
        """
        # Save initial remote settings
        initial_settings = self.remote_settings
        # Save filter settings
        filter_setting = self.filter_setting
        filter_type = self.filter_type

        # Set calibration conditions
        #     Setting input to CAL_REF
        self.select_measurement_type(0, 'single-ended calibration')

        # set to best performance for offset calibration
        self.set_filter(adc, 'sinc3', 20)

        self._write_data_16(adc, self.reg.adcMODE_REG,
                                (7<<self.reg.MODE) |
                                (1<<self.reg.SING_CYC))
        self.wait_for_data(adc)

        self._write_data_16(adc, self.reg.adcMODE_REG,
                                (0<<self.reg.MODE) |
                                (1<<self.reg.SING_CYC))
        #self.add_to_GAIN0(adc, -3700)  # was -10900 in D4    TEMP COMMENTED

        # Restore filter settings
        self.set_filter(adc, filter_type, filter_setting)
		# Restore remote settings
        self.remote_settings = initial_settings
        self.spi_rack.write_data(self.module, 2, HCT595_MODE, HCT595_SPEED, bytearray([self.remote_settings]))

    def gain_calibration_with_external_reference(self, adc):
        """Gain calibration routine

        Calibrates the gain of the given ADC input. To run this routine, apply
        4V on the input of the given ADC using a D5a.

        Args:
            adc (int:0-1): ADC to calibrate
        """
        print('Make sure that ADC {} input is set to the experiment full-scale (using a precise voltage source)!\n',
        'For the general case use the D4a maximum scale of 3.5V.'.format(adc))   # prev. we printed 'adc' without adc+1
        
        # Save initial conditions
        initial_conditions = self.save_remote_settings()

        # Set calibration conditions
        #     Setting input to 'single-ended measurement'
        self.select_measurement_type(adc, 'single-ended measurement')

        # set to best performance for offset calibration
        self.set_filter(adc, 'sinc3', 20)

        self._write_data_16(adc, self.reg.adcMODE_REG,
                                (7<<self.reg.MODE) |
                                (1<<self.reg.SING_CYC))
        self.wait_for_data(adc)

        self._write_data_16(adc, self.reg.adcMODE_REG,
                                (0<<self.reg.MODE) |
                                (1<<self.reg.SING_CYC))
        #self.add_to_GAIN0(adc, -3700)  # was -10900 in D4    TEMP COMMENTED

		# Restore initial conditions
        self.restore_remote_settings(adc, initial_conditions)

    def set_GAIN0(self, adc, gain_setting):
        """Set the GAIN resgiter for set-up 0
        """
        self._write_data_24(adc, self.reg.GAIN0_REG, gain_setting)
        return

    def add_to_GAIN0(self, adc, gain_incr):
        """Add to the GAIN resgiter of set-up 0
        """
        value = self.read_GAIN0(adc)
        self._write_data_24(adc, self.reg.GAIN0_REG, value+gain_incr)
        return

    def read_GAIN0(self, adc):
        """Read the GAIN resgiter for set-up 0
        """
        if adc not in range(2):
            raise ValueError('ADC number {} does not exist. Possible values are 0 and 1.'.format(adc))
        rdata = self._read_data(adc, self.reg.GAIN0_REG, 3)                   # DEBUG
        print("    rdata is",rdata)                                           # DEBUG
        rdata = self._read_data(adc, self.reg.GAIN0_REG, 3)[1:]
        print("    The final rdata is",rdata[0], rdata[1], rdata[2], rdata)   # DEBUG
        value = rdata[0]<<16 | rdata[1]<<8 | rdata[2]
        return value

    def set_OFFSET0(self, adc, gain_setting):
        """Set the OFFSET resgiter for set-up 0
        """
        self._write_data_24(adc, self.reg.OFFSET0_REG, gain_setting)
        return

    def read_OFFSET0(self, adc):
        """Read the OFFSET resgiter for set-up 0
        """
        if adc not in range(2):
            raise ValueError('ADC number {} does not exist. Possible values are 0 and 1.'.format(adc))
        rdata = self._read_data(adc, self.reg.OFFSET0_REG, 3)[1:]
        value = rdata[0]<<16 | rdata[1]<<8 | rdata[2]
        return value

    def _default_setup(self):
        # Basic configuration
        for adc in range(0, 2):

            # adcMODE_REG:
			#     Mode - set continuous conversion mode
            #     Single cycle - only output data at filter settling rate
            #     Also implcitly disabling internal ref and setting the clock source to internal
            self._write_data_16(adc, self.reg.adcMODE_REG,
								(0<<self.reg.MODE) |
								(1<<self.reg.SING_CYC))

            # IFMODE_REG:
            self._write_data_16(adc, self.reg.IFMODE_REG, (1<<self.reg.DOUT_RESET))

            # GPIOCON_REG:
            #     Disable ext_synch
            self._write_data_16(adc, self.reg.GPIOCON_REG,
                                (0<<self.reg.SYNC_EN) |
                                (0<<self.reg.IP_EN1) |
                                (0<<self.reg.IP_EN0) |
                                (0<<self.reg.OP_EN1) |
                                (0<<self.reg.OP_EN0))
            
            # CH0_REG:
            self._write_data_16(adc, self.reg.CH0_REG,
                                (1<<self.reg.CH_EN) |
                                (0<<self.reg.SETUP_SEL) |
                                (self.reg.AIN3<<self.reg.AINPOS) |
                                (self.reg.AIN2<<self.reg.AINNEG))

            # CH1,2,3_REG:
            self._write_data_16(adc, self.reg.CH1_REG,
                                (0<<self.reg.CH_EN) |
                                (0<<self.reg.SETUP_SEL) |
                                (self.reg.AIN1<<self.reg.AINPOS) |
                                (self.reg.AIN4<<self.reg.AINNEG))

            self._write_data_16(adc, self.reg.CH2_REG,
                                (0<<self.reg.CH_EN) |
                                (0<<self.reg.SETUP_SEL) |
                                (self.reg.AIN2<<self.reg.AINPOS) |
                                (self.reg.AIN4<<self.reg.AINNEG))

            self._write_data_16(adc, self.reg.CH3_REG,
                                (0<<self.reg.CH_EN) |
                                (0<<self.reg.SETUP_SEL) |
                                (self.reg.AIN3<<self.reg.AINPOS) |
                                (self.reg.AIN4<<self.reg.AINNEG))

            # SETUPCON0_REG:
            self._write_data_16(adc, self.reg.SETUPCON0_REG,
                                (1<<self.reg.BI_UNIPOLAR) |
                                (1<<self.reg.REFBUF0P) |
                                (1<<self.reg.REFBUF0M) |
                                (0<<self.reg.AINBUF0P) |
                                (0<<self.reg.AINBUF0M))

            # OFFSET 0,1,2,3_REG:
            self._write_data_24(adc, self.reg.OFFSET0_REG, 8388150)
            self._write_data_24(adc, self.reg.OFFSET1_REG, 8388150)
            self._write_data_24(adc, self.reg.OFFSET2_REG, 8388150)
            self._write_data_24(adc, self.reg.OFFSET3_REG, 8388150)

            # GAIN 0,1,2,3_REG:
            #     Set the gain such that +-3.5V is full scale. Can be overwritten by user's calibration
            self._write_data_24(adc, self.reg.GAIN0_REG, 5699250)
            self._write_data_24(adc, self.reg.GAIN1_REG, 5699250)
            self._write_data_24(adc, self.reg.GAIN2_REG, 5699250)
            self._write_data_24(adc, self.reg.GAIN3_REG, 5699250)

    def set_clock_mode(self, adc, clockmode):
        """Selects the Clock Mode for the ADC

        """
        d_clock_modes = {'IntOsc':0b00, 'IntOscToOut':0b01, 'ExtClk':0b10, 'ExtOsc':0b11}
        if clockmode not in d_clock_modes:
            raise ValueError('Value {} does not exist. Possible values are: {}'.format(clockmode, d_clock_modes.keys()))
        self._write_data_16(self, adc, self.reg.adcMODE_REG,
                            (d_clock_modes[clockmode]<<self.reg.CLOCKSEL))
    
    def set_filter(self, adc, filter_type, filter_setting):
        """Sets the ADC filter

        The two filter parameters determine the filter response (cutoff frequency),
        the 50 Hz rejection and the resolution. See the filter table on the website
        to determine which setting is correct for your application.

        Args:
            adc (int:0-1): ADC inside the module which needs to be set
            filter_type (string): set to sinc3 or sinc5
            filter_setting (int:0-20): the desired filter setting
        """
        filter_values = {'sinc3': 3, 'sinc5': 0}
        if filter_type not in filter_values:
            raise ValueError('Value {} does not exist. Possible values are: {}'.format(filter_type, filter_values))
        if filter_setting not in range(21):
            raise ValueError('Value {} not allowed. Possible values are from 0 to 20.'.format(filter_setting))

        self._write_data_16(adc, self.reg.FILTCON0_REG,
                            (filter_values[filter_type]<<self.reg.ORDER0) |
                            (filter_setting<<self.reg.ODR))

        self.filter_setting = filter_setting
        self.filter_type = filter_type

    def get_ADC_ID(self, adc):

        # From register ID REGISTER: 
        #     extract ID
        ADC_ID_raw = self._read_data(adc, self.reg.ID_REG, 2)
        ADC_ID_chopped = ADC_ID_raw[1:]
        ADC_ID = (ADC_ID_chopped[0]<<8)|(ADC_ID_chopped[1])
        return ADC_ID

    def get_ADC_status(self, adc):
        AdcState = {}

        # From register ID_REG: 
        #     extract ID
        ID_REG_data = self.get_ADC_ID(adc)
        print("Reading ID_REG:")
        print("    ID is ", ID_REG_data)
        AdcState['ID_REG']= ID_REG_data
        
        # From register CH0_REG: 
        #     extract CH_EN0
        CH0_REG_data = self._read_data(adc, self.reg.CH0_REG, 2)
        CH0_REG_data = CH0_REG_data[1:]
        CH_EN0 = (CH0_REG_data[0]>>7)&0x01
        AINPOS0 = ((CH0_REG_data[1]&0xe0)>>5)|((CH0_REG_data[0]&0x03)<<3)
        AINNEG0 = (CH0_REG_data[1]&0x1f)
        # prints for debug
        print("Reading ADC{} CH0_REG:".format(adc))
        print("    CH0_REG data is ", CH0_REG_data, " (or integer value of ", int.from_bytes(CH0_REG_data, byteorder='big', signed=False),")")
        print("    CH0_REG's CH_EN0 is:", CH_EN0, ", AINPOS0 is:", AINPOS0, ", AINNEG0 is:", AINNEG0)
        AdcState['CH_EN0']=CH_EN0
        AdcState['AINPOS0']=AINPOS0
        AdcState['AINNEG0']=AINNEG0

        # From register adcMODE (p. 53): 
        #     extract REF_EN, SING_CYC, MODE, CLOCKSEL
        adcMODE_REG_data = self._read_data(adc, self.reg.adcMODE_REG, 2)
        adcMODE_REG_data = adcMODE_REG_data[1:]
        REF_EN = (adcMODE_REG_data[0]>>7)&0x01
        SING_CYC = (adcMODE_REG_data[0]>>5)&0x01
        MODE = (adcMODE_REG_data[1]>>4)&0x07
        CLOCKSEL = adcMODE_REG_data[1]&0x0c
        # prints for debug
        print("Reading ADC{} MODE_REG:".format(adc))
        print("    MODE_REG data is ", adcMODE_REG_data, " (or integer value of ", int.from_bytes(adcMODE_REG_data, byteorder='big', signed=False),")")
        print("    adcMODE_REG's Internal_REF is:",REF_EN," SING_CYC is:", SING_CYC, ", MODE is:",MODE, ", CLOCKSEL is:",CLOCKSEL)
        AdcState['Internal_REF EN']=REF_EN
        AdcState['SING_CYC']=SING_CYC
        AdcState['MODE']=MODE
        AdcState['CLOCKSEL']=CLOCKSEL

        # From register FILTCON0:
        #     extract ODR, Sync_Filter
        FILTCON0_REG_data = self._read_data(adc, self.reg.FILTCON0_REG, 2)
        FILTCON0_REG_data = FILTCON0_REG_data[1:]
        Sync_Filter = FILTCON0_REG_data[1]>>5&0x03  # Order (type) of digital filter used
        ODR = FILTCON0_REG_data[1]&0x1f             # 'ODR' is Output Data Rate
        # prints for debug
        print("Reading ADC{} FILTCON0_REG:".format(adc))
        print("    FILTCON0_REG data is ", FILTCON0_REG_data, " (or integer value of ", int.from_bytes(FILTCON0_REG_data, byteorder='big', signed=False),")")
        print("    FILTCON0_REG's Sync_Filter is:", Sync_Filter, ", ODR code is:",ODR)
        AdcState['Output Data Rate']=ODR
        AdcState['Sync_Filter']=Sync_Filter

		# From register GPIOCON:
        #     extract EXT_ENABLE, OP1_EN, GP_DATA1
        GPIO_data = self._read_data(adc, self.reg.GPIOCON_REG, 2)
        GPIO_data = GPIO_data[1:]
        EXT_ENABLE = (GPIO_data[0]>>3)&0x01
        OP_EN1 = (GPIO_data[1]>>3)&0x01
        GP_DATA1 = (GPIO_data[1]>>1)&0x01
        # prints for debug
        print("Reading ADC{} GPIOCON:".format(adc))
        print("    GPIOCON is ", GPIO_data, " (or integer value of ", int.from_bytes(GPIO_data, byteorder='big', signed=False),")")
        print("    GPIOCON_REG's EXT_ENABLE is:", EXT_ENABLE, ", OP_EN1 is:", OP_EN1, ", GP_DATA1 is:",GP_DATA1)
        AdcState['EXT_ENABLE']=EXT_ENABLE
        AdcState['OP_EN1']=OP_EN1
        AdcState['GP_DATA1']=GP_DATA1

		# From register SETUPCON0 (p. 59):
        #     extract REFBUF0+, REFBUF0-, REF_SEL0
        GPIO_data = self._read_data(adc, self.reg.SETUPCON0_REG, 2)
        GPIO_data = GPIO_data[1:]
        REFBUF0plus = (GPIO_data[0]>>3)&0x01
        REFBUF0minus = (GPIO_data[0]>>2)&0x01
        ANLGBUF0plus = (GPIO_data[0]>>1)&0x01
        ANLGBUF0minus = GPIO_data[0]&0x01
        REF_SEL0 = (GPIO_data[1]>>4)&0x03
        # prints for debug
        print("Reading ADC{} SETUPCON0:".format(adc))
        print("    SETUPCON0 is ", GPIO_data, " (or integer value of ", int.from_bytes(GPIO_data, byteorder='big', signed=False),")")
        print("    SETUPCON0_REG's REFBUF0+ is:", REFBUF0plus, ", REFBUF0- is:", REFBUF0minus, ", REF_SEL0 is:" ,REF_SEL0 , "ANLGBUF0+ is:", ANLGBUF0plus, ", ANLGBUF0- is:", ANLGBUF0minus)
        AdcState['REFBUF0+']=REFBUF0plus
        AdcState['REFBUF0-']=REFBUF0minus
        AdcState['ANLGBUF0+']=ANLGBUF0plus
        AdcState['ANLGBUF0-']=ANLGBUF0minus
        AdcState['REF_SEL0']=REF_SEL0

        # From register OFFSET0:
        # prints for debug
        print("Reading ADC{} OFFSET0:".format(adc))
        OFFSET0 = self.read_OFFSET0(adc)
        print("    OFFSET0 is:", OFFSET0)
        AdcState['OFFSET0']=OFFSET0
        
        # From register GAIN0:
        # prints for debug
        print("Reading ADC{} GAIN0:".format(adc))
        GAIN0 = self.read_GAIN0(adc)
        print("    GAIN0 is:", GAIN0)
        AdcState['GAIN0']=GAIN0
        
        return AdcState
        
        
    def set_output_enable(self, SR_OEb):
        """
        Sets the output_enable\ signal into the SR.        
        Args:
            
        """
        if SR_OEb not in range(0, 2):
            raise ValueError('The output_enable given {} is not a legal value. Possible values are in {}'.format(SR_OEb, range(0, 2)))

        # The output byte holds two bits:
        # The 'SR_OEb' - given hereby explicitly by the user
        # The trig/sync - the existing bic_sync_value kept in memory
        output_byte = SR_OEb<<7|self.bic_sync_value
        print("The output_byte from BIC to module is",output_byte)
        
        self.spi_rack.write_data(self.module, 5, 0, BICPINS_SPEED, bytearray([output_byte]))
        self.SR_OEb = SR_OEb                     # need to keep this value for sync_signal setting
        

    def set_sync_signal(self, user_sync_value):
        """
        Sets the sync signal from the BIC to the ADCs
        to 0 or 1, as configured.
        Note 1: one sync signal serves both ADC0, ADC1.
        Note 2: sync\ low freezes operation. Operation resumes at sync\ rising edge,
                and continues while high value maintained.
        Note 3: the BIC_TRIG is OR'ed with the front-panel wired TRIG. Therefore
                a 'high' setting from a front-panel TRIG is enough to keep the sync at 'high'
                even with the BIC_TRIG low.
        
        Args:
            
        """
        if user_sync_value not in range(0, 2):
            raise ValueError('The sync_value given {} is not a legal value. Possible values are in {}'.format(user_sync_value, range(0, 2)))

        print("sync_signal from user is ",bytearray([user_sync_value]))
        
        # The output byte holds two bits:
        # The 'SR_OEb' - the existing SR_OEb kept in memory
        # The trig/sync - the new user_sync_value given here explicitly by the user
        output_byte = self.SR_OEb<<7|user_sync_value
        print("The output_byte from BIC to module is",output_byte)
        
        # Write to SPI addr 5 - the GPIO output direction
        self.spi_rack.write_data(self.module, 5, 0, BICPINS_SPEED, bytearray([output_byte]))
        
        self.bic_sync_value = user_sync_value             # need to keep this value for SR_OEb setting

    def read_unified_sync_signal(self):
        """
        Note: there is only one sync signal in the module, not per ADC
                
        Args:
            
        """
        s_data = bytearray([0])
        r_data = self.spi_rack.read_data(self.module, 4, 0, BICPINS_SPEED, s_data)
        lsb_value = int.from_bytes(r_data, byteorder='little', signed=False)&0x1
        return lsb_value

    def configure_sync_mode(self, adc, ext_enable):
        """Wait for external trigger to perform a conversion

        Configures the given ADC to depend on external 'enable' (a.k.a. synch or mask) to perform a new convertion.
        A call to get the result needs to follow to access the data.

        Args:
            adc (int:0-1): ADC to perform the conversion
            ext_enable (int:0-1): 1 - external trigger mode; 0 - regular operation (SW)
        """
        # input checks
        if adc not in range(0, 2):
            raise ValueError('ADC {} does not exist. Possible values are: {}'.format(adc, range(0, 2)))
        if ext_enable not in range(0, 2):
            raise ValueError('ext_enable {} is not a legal value. Possible values are in {}'.format(ext_enable, range(0, 2)))

        gpio_desired = (ext_enable<<self.reg.SYNC_EN)|(ext_enable<<self.reg.OP_EN1)|(ext_enable<<self.reg.GP_DATA1)
        self._write_data_16(adc, self.reg.GPIOCON_REG, gpio_desired)
        gpio_actual = self._read_data(adc, self.reg.GPIOCON_REG, 2)
        gpio_actual = gpio_actual[1]<<8|gpio_actual[2]
        if (gpio_actual != gpio_desired):
            raise ValueError('D4a set_ext_trigger error: failed to configure desired trigger mode {} to ADC{}'.format(ext_enable, adc))

    def _read_data(self, adc, reg, num_bytes):
        """
        Read a given number of bytes (num_bytes) from given adc register
        """
        s_data = bytearray([reg | (1<<6)] + num_bytes*[0])
        r_i_data = self.spi_rack.read_data(self.module, adc, AD7175_MODE, AD7175_SPEED, s_data)

        return r_i_data

    def _write_data_8(self, adc, reg, data):
        s_data = bytearray([reg, data])
        self.spi_rack.write_data(self.module, adc, AD7175_MODE, AD7175_SPEED, s_data)

    def _write_data_16(self, adc, reg, data):
        s_data = bytearray([reg, data>>8, data&0xFF])
        self.spi_rack.write_data(self.module, adc, AD7175_MODE, AD7175_SPEED, s_data)

    def _write_data_24(self, adc, reg, data):
        s_data = bytearray([reg, data>>16, (data>>8)&0xFF, data&0xFF])
        self.spi_rack.write_data(self.module, adc, AD7175_MODE, AD7175_SPEED, s_data)

    def _reset_ADC(self, adc):
        s_data = bytearray([self.reg.GAIN0_REG, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff])
        self.spi_rack.write_data(self.module, adc, AD7175_MODE, AD7175_SPEED, s_data)


class AD7175_registers:
    """AD7175 register class

    A list of all the register names with values and all bits with corresponding
    locations in the registers.
    """
    # adc register locations
    STATUS_REG = 0x00
    adcMODE_REG = 0x01
    IFMODE_REG = 0x02
    REGCHECK_REG = 0x03
    DATA_REG = 0x04
    GPIOCON_REG = 0x06
    ID_REG = 0x07
    CH0_REG = 0x10
    CH1_REG = 0x11
    CH2_REG = 0x12
    CH3_REG = 0x13
    SETUPCON0_REG = 0x20
    SETUPCON1_REG = 0x21
    SETUPCON2_REG = 0x22
    SETUPCON3_REG = 0x23
    FILTCON0_REG = 0x28
    FILTCON1_REG = 0x29
    FILTCON2_REG = 0x2A
    FILTCON3_REG = 0x2B
    OFFSET0_REG = 0x30
    OFFSET1_REG = 0x31
    OFFSET2_REG = 0x32
    OFFSET3_REG = 0x33
    GAIN0_REG = 0x38
    GAIN1_REG = 0x39
    GAIN2_REG = 0x3A
    GAIN3_REG = 0x3B

    # Status Register bits
    nRDY = 7
    adc_ERROR = 6
    CRC_ERROR = 5
    REG_ERROR = 4
    CHANNEL = 0

    # adc Mode Register bits
    REF_EN = 15
    HIDE_DELAY = 14
    SING_CYC = 13
    DELAY = 8
    MODE = 4
    CLOCKSEL = 2

    # IFMODE Register bits
    ALT_SYNC = 12
    IOSTRENGTH = 11
    DOUT_RESET = 8
    CONTREAD = 7
    DATA_STAT = 6
    REG_CHECK = 5
    CRC_EN = 2
    WL16 = 0

    # GPIOCON Register bits
    MUX_IO = 12
    SYNC_EN = 11
    ERR_EN = 9
    ERR_DAT = 8
    IP_EN1 = 5
    IP_EN0 = 4
    OP_EN1 = 3
    OP_EN0 = 2
    GP_DATA1 = 1
    GP_DATA0 = 0

    # Channel Registers bits
    CH_EN = 15
    SETUP_SEL = 12
    AINPOS = 5
    AINNEG = 0

    # Setup Configuration Register bits
    BI_UNIPOLAR = 12
    REFBUF0P = 11
    REFBUF0M = 10
    AINBUF0P = 9
    AINBUF0M = 8
    REF_SEL = 4

    # Filter Configuration Register bits
    SINC3_MAP0 = 15
    ENHFILTEN = 11
    ENHFILT = 8
    ORDER0 = 5
    ODR = 0

    # adc register values
    AIN0 = 0
    AIN1 = 1
    AIN2 = 2
    AIN3 = 3
    AIN4 = 4
    REFP = 21
    REFN = 22
