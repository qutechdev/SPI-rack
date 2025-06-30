from .spi_rack import SPI_rack
from .chip_mode import MCP320x_MODE, MCP320x_SPEED, BICPINS_SPEED

class P2d_module(object):
    """P2d module interface class

    This class does the low level interfacing with the P2d module.
    It requires an SPI Rack object and module number at initialization.

    Attributes:
        module: the module number set by the user (must coincide with hardware)
        remote_settings: contains a byte with the remote settings
    """
    possible_channels = [1, 2, 3, 4]
    possible_states = ['ON', 'OFF']
    possible_limits = ['LOW', 'HIGH']

    def __init__(self, spi_rack, module, Chn1_state = 'off', Chn2_state = 'off', Chn3_state = 'off', Chn4_state = 'off'):
        """Inits P2d module class

        The P2d module needs an SPI_rack class for communication. Clears the RF clipped
        bit a startup.

        Args:
            spi_rack: SPI_rack class object via which the communication runs
            module: module number set on the hardware
            Chn1_state, Chn2_state, Chn3_state, Chn4_state: control for channel being on or off
        Example:
            P2d = P2d_module(SPI_Rack_1, 4)
        """
        # Set module number for Chip Select
        self.module = module
        # Receive the spi_rack object to use
        self.spi_rack = spi_rack
        self.remote_settings = 0x0A
        initial_states = [Chn1_state, Chn2_state, Chn3_state, Chn4_state]
        for i in self.possible_channels:
            chn_num = i-1
            self.calc_settings_for_enabling_outputs(self.possible_channels[chn_num], initial_states[chn_num])
        self.spi_rack.write_data(self.module, 2, 0, BICPINS_SPEED, bytearray([self.remote_settings]))

    def input_checks_channel_num(self, channel_num):
        """
        This function is used interally.
        """
        if channel_num not in self.possible_channels:
            raise ValueError('Value {} does not exist. Possible values are: {}'.format(channel_num, possible_channels))
        else:
            return 1

    def input_checks_channel_state(self, state):
        """
        This function is used interally.
        """
        if state.upper() not in self.possible_states:
            raise ValueError('Value {} does not exist. Possible values are: {}'.format(state, possible_states))
        else:
            return 1

    def input_checks_current_limit(self, limit):
        """
        This function is used interally.
        """
        if limit.upper() not in self.possible_limits:
            raise ValueError('Value {} does not exist. Possible values are: {}'.format(limit, possible_limits))
        else:
            return 1

    def read_adc_K1K2K5K6(self, channel):
        """Reads the ADC for MCX inputs K1, K2, K5, K6

        Reads a given channel from an MCP3202 ADC. Output needs to be converted to ... 

        The function is used internally.

        Args:
            channel (int: 0-1): the ADC channel to be read
        Returns:
            12-bit ADC data (int)
        """
        s_data = bytearray([1, 160|(channel<<6), 0])
        r_data = self.spi_rack.read_data(self.module, 3, MCP320x_MODE, MCP320x_SPEED, s_data)
        return [r_data, (r_data[1]&0xF)<<8 | r_data[2]]        # (r_data[0]&0xF)<<8 | r_data[1] or (r_data[1]&0xF)<<8 | r_data[2]

    def read_adc_K8K11(self, channel):
        """Reads the ADC for MCX inputs K8, K11

        Reads a given channel from an MCP3202 ADC. Output needs to be converted to ... 
        It is very similar to 'read_adc_K1K2K5K6' except using a different SPI address.
        
        The function is used internally.

        Args:
            channel (int: 0-1): the ADC channel to be read
        Returns:
            12-bit ADC data (int)
        """
        s_data = bytearray([1, 160|(channel<<6), 0])
        r_data = self.spi_rack.read_data(self.module, 0, MCP320x_MODE, MCP320x_SPEED, s_data)
        return [r_data, (r_data[1]&0xF)<<8 | r_data[2]]        # (r_data[0]&0xF)<<8 | r_data[1] or (r_data[1]&0xF)<<8 | r_data[2]

    def get_output_current(self, channel):
        """
        Args:
            channel (int: 0-3): the ADC channel to be read
        Returns:
            The measured current in Amperes
        Example:
            I_meas = P2d.get_output_current(2)
        """
        self.input_checks_channel_num(channel)

        chn_to_enbl_posn = {1:0, 2:2, 3:4, 4:6}    # positions of enable bits in settings byte
        bit_pos = chn_to_enbl_posn[channel]
        if not(self._read_bit_in_byte(self.remote_settings, bit_pos)):
            raise ValueError('Channel {} is not active. Please enable the channel before attemping to read its output.'.format(channel))
        
        # for 'north' 2 outputs - K2, K6
        if channel in [1,2]:
            ADC_raw_data = self.read_adc_K1K2K5K6(channel-1)[0]
            ADC_processed_data = self.read_adc_K1K2K5K6(channel-1)[1]
            ADC_input_voltage = ADC_processed_data*5/4096  # [V]
            Vlim = ADC_input_voltage/16.667      # [V], between the LT3040 output and the OPA2192 Amp output - 50/3
            
            if (channel == 1):
                res_select = (self.remote_settings&0x02)>>1  #for channel 1, checking LIM1
            else:    # channel is 2
                res_select = (self.remote_settings&0x08)>>3  #for channel 2, checking LIM2
                
            if (res_select == 0):
                selected_R = 1200      # R is 1.2K [Ohm]
            else:
                selected_R = 12000     # R is 12K [Ohm]
            Ilim = Vlim / selected_R   #[A]
            Iout = Ilim*400

        # for 'south' 2 outputs - K4, K10
        if channel in [3,4]:
            ADC_raw_data = self.read_adc_K8K11(channel-3)[0]
            ADC_processed_data = self.read_adc_K8K11(channel-3)[1]
            ADC_input_voltage = ADC_processed_data*3.3/4096  # [V]
			
            if (channel == 3):
                res_select = (self.remote_settings&0x20)>>5  #for channel 3, checking LIM3
            else:    # channel is 4
                res_select = (self.remote_settings&0x80)>>7  #for channel 4, checking LIM4
                
            if (res_select == 0):
                selected_R = 15000 # [A], R is 15K
            else:
                selected_R = 1500 # [A], R is 1.5K
			
            Iout = ADC_input_voltage/(2*selected_R)
            
        # clipping / open-circuit warning for all channels
        
        # If the ADC output data is unrealistically low
        if (ADC_processed_data == 0) or (ADC_processed_data == 1):
            print("\033[31mMeasured current is zero: no load connected (output is open) or the input and load levels are the same (likely 0V). Data is invalid\033[39m")
        
        # If the ADC output data is unrealistically high
        if  (ADC_processed_data == 4094) or (ADC_processed_data == 4095):
            print("\033[31mADC IS CLIPPED. Data is invalid\033[39m")
            
        return Iout

    def read_overcurrents(self):
        """Reads the SN74HC165
        Returns:
            4 bits for FLT1, FLT2, FLT3, FLT4
        Example:
            P2d.read_overcurrents()
        """
        s_data = bytearray([0xFF])
        fault_logics = {1:'FAULT',0:'CORRECT'}
        r_data = self.spi_rack.read_data(self.module, 1, 2, 21, s_data)
        FLT1 = (r_data[0])&0x01
        FLT2 = (r_data[0]>>1)&0x01
        FLT3 = (r_data[0]>>2)&0x01
        FLT4 = (r_data[0]>>3)&0x01
        print("\Fault 1 status is",fault_logics[FLT1])
        print("\Fault 2 status is",fault_logics[FLT2])
        print("\Fault 3 status is",fault_logics[FLT3])
        print("\Fault 4 status is",fault_logics[FLT4])
        return [FLT4, FLT3, FLT2, FLT1]

    def set_I_limit(self, channel, limit):
        """Sets a High or Low limit to the current out of a P2d channel

        Args:
            channel (int: 0-3): the ADC channel
            limit (string: 'HIGH'/'LOW'): the desired limit for the channel
        """
        self.input_checks_channel_num(channel)
        self.input_checks_current_limit(limit)
        upprcs_limit = limit.upper()              # upper case to match the look-up table

        # per-channel controls are ch1 - LIM1, ch2 - LIM2, ch3 - ILIM3, ch4 - ILIM 4
        # limits are:
        #     Channels 1, 2: 'LOW' - 10mA, 'HIGH' - 100mA
        #     Channels 3, 4: 'LOW' - 100uA, 'HIGH' - 1000uA (1mA)

        chn_to_lim_posn = {1:1, 2:3, 3:5, 4:7}    # positions of limit bits in settings byte
        if channel in (1,2):
            lim_to_bit = {'LOW':1, 'HIGH':0}
        else:
            lim_to_bit = {'LOW':0, 'HIGH':1}

        self.remote_settings = self._configure_bit_in_byte(bit_pos = chn_to_lim_posn[channel], bit_value = lim_to_bit[upprcs_limit], orig_byte = self.remote_settings)
        
        self.spi_rack.write_data(self.module, 2, 0, BICPINS_SPEED, bytearray([self.remote_settings]))
        print("Setting channel",channel,"to '",limit,"' limit - done\n")

    def enable_outputs(self, channel_num, state):
        """enables or disables one output channel in the P2d

        Args:
            channel (int: 0-3): the ADC channel
            state (string: 'ON'/'OFF'): the desired state for the channel
        """
        self.calc_settings_for_enabling_outputs(channel_num, state)
        self.spi_rack.write_data(self.module, 2, 0, BICPINS_SPEED, bytearray([self.remote_settings]))

    def calc_settings_for_enabling_outputs(self, channel_num, state):
        """
        This function is used internally.
        """
        self.input_checks_channel_num(channel_num)
        self.input_checks_channel_state(state)
        state = state.upper()                         # upper case to match the look-up table

        # states are active - 'ON', inactive - 'OFF'
        # per-channel controls are ACT1, ACT2, EN3, EN4
        state_text_to_Bool = {'ON':1, 'OFF':0}
        chn_num_to_bit_posn = {1:0, 2:2, 3:4, 4:6}    # positions in the settings byte
        
        bit_pos = chn_num_to_bit_posn[channel_num]
        
        state_Bool = state_text_to_Bool[state]

        self.remote_settings = self._configure_bit_in_byte(bit_pos = bit_pos, bit_value = state_Bool, orig_byte = self.remote_settings)

    def get_P2d_settings(self):
        """
        This functions returns the status of all 4 channel in the P2d. It is based
        on reading an array variable in the P2d SW code.
        
        Returns:
            A copy of the SW array. It also prints the data with interpretation of the fields.
        Example:
            P2d.get_P2d_settings()
        """
        
        chnn_enabled = {0:'OFF', 1:'ON'}
        high_current_chnn_limits = {1:'LOW', 0:'HIGH'}   # K1, K2, K5, K6
        low_current_chnn_limits = {1:'HIGH', 0:'LOW'}    # K7, K8, K10, K11
        
        print("\nP2d remote settings are:",
        "\n    The complete byte is", self.remote_settings,bin(self.remote_settings),"of type",type(self.remote_settings),
        "\n    ACT1 is", (self.remote_settings&0x01),"",chnn_enabled[self.remote_settings&0x01],
        "\n    LIM1 is", (self.remote_settings&0x02)>>1,"",high_current_chnn_limits[(self.remote_settings&0x02)>>1],
        "\n    ACT2 is", (self.remote_settings&0x04)>>2,"",chnn_enabled[(self.remote_settings&0x04)>>2],
        "\n    LIM2 is", (self.remote_settings&0x08)>>3,"",high_current_chnn_limits[(self.remote_settings&0x08)>>3],
        "\n    EN3 is",  (self.remote_settings&0x10)>>4,"",chnn_enabled[(self.remote_settings&0x10)>>4],
        "\n    ILIM3 is",(self.remote_settings&0x20)>>5,"",low_current_chnn_limits[(self.remote_settings&0x20)>>5],
        "\n    EN4 is",  (self.remote_settings&0x40)>>6,"",chnn_enabled[(self.remote_settings&0x40)>>6],
        "\n    ILIM4 is",(self.remote_settings&0x80)>>7,"",low_current_chnn_limits[(self.remote_settings&0x80)>>7])
        
        return self.remote_settings

    def get_repeated_readings(self, channel, num_readings):
        P2d_repeated_readings = [None]*num_readings
        for i in range(num_readings):
            P2d_repeated_readings[i] = self.get_output_current(channel = channel)
        return P2d_repeated_readings

    def _configure_bit_in_byte(self, bit_pos, bit_value, orig_byte):
        """
        This function is used internally.

        Returns:
            The original byte with the bit updated
        """
        possible_positions = [0, 1, 2, 3, 4, 5, 6, 7]
        if bit_pos not in possible_positions:
            raise ValueError('Bit position {} does not exist in a byte. Possible values are 0 through 7')
        
        possible_values = [0, 1]
        if bit_value not in possible_values:
            raise ValueError('Bit value {} does not exist. Possible values are: {}'.format(bit_value, possible_values))

        if bit_value == 0:
            update_vector = 255-(2**bit_pos)
            new_byte = orig_byte & update_vector
        else:
            update_vector = bit_value<<bit_pos
            new_byte = orig_byte | update_vector
        return new_byte

    def _read_bit_in_byte(self, byte, bit_pos):
        """
        This function is used internally.

        Returns:
            The bit value in the specified position
        """
        possible_positions = [0, 1, 2, 3, 4, 5, 6, 7]
        if bit_pos not in possible_positions:
            raise ValueError('Bit position {} does not exist in a byte. Possible values are 0 through 7')

        bit_val = (byte >> bit_pos) & 1
        print("for byte",byte,"and bit_pos",bit_pos,"the value is",bit_val)
        return bit_val