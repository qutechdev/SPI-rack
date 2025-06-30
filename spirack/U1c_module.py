"""DAC module U1c interface

SPI Rack interface code for the D5a module.

Example:
    Example use: ::
        U1c = spirack.U1c_module(SPI_Rack1, 2, 0.1, 1)

Attributes:
    
"""

import logging
import time
import numpy as np
import os                  # to get path to file
import binascii            # for hexlify
import csv                 # for reading, writing user data in csv format
from copy import deepcopy  # to generate switch-row and switch-grid states
import winsound            # for audio notification
import pandas as pd        # for ordered printing of settings from csv


from .chip_mode import HCT595_MODE, HCT595_SPEED, BICPINS_MODE, BICPINS_SPEED

logger = logging.getLogger(__name__)

class U1c_module(object):
    """U1c module interface class

    ... 

    Attributes:
        
    """

    def __init__(self, spi_rack, module, switch_delay = 0.1, initialize = 1):
        """Initializes U1c module class

        The U1c_module class needs an SPI_rack object at initiation. All
        communication will run via that class.

        Args:
            spi_rack (SPI_rack object): SPI_rack class object via which the communication runs
            module (int): module number set on the hardware
            switch_delay (float): lag time between interim updates 
									when transitioning from one state to another;
									meant to let all stransients settle
            initialize (bool): determines whether to disengage (switch off) all connections
        """
        self.spi_rack = spi_rack
        self.module = module
        self.initialized = 0
        
        self.switch_delay = switch_delay
        self.switch_cfg_file = 'U1c_user_config.csv'
        self.settings_current = [
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0']
        ]
        self.settings_default = [
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0'],
            ['0','0','0','0','0','0','0','0']
        ]        
        self.settings_all_on = [               
            ['1','1','1','1','1','1','1','1'],
            ['1','1','1','1','1','1','1','1'],
            ['1','1','1','1','1','1','1','1'],
            ['1','1','1','1','1','1','1','1'],
            ['1','1','1','1','1','1','1','1'],
            ['1','1','1','1','1','1','1','1'],
            ['1','1','1','1','1','1','1','1'],
            ['1','1','1','1','1','1','1','1']
        ]
        
        if (initialize==1):
            self.disconnect_all()    # disconnect_all() also sets to 1 the setting self.initialized 
        updt_data = self.convert_settings_to_bytearray(self.settings_default)
        self.mux8by8_write_user_data(3, updt_data)
        self.enable_SR_output(value = 'enable')

    def enable_SR_output(self, value = 'disable'):
        """
        OE\ has inverse logic, so
        at value = 0 the output is active ('enable')
        at value = 1 the output is inactive ('disable', chosen as default)
        """
        if value=='enable':
            data = 0
        elif value=='disable':
            data = 1
        else:
            raise ValueError("Value",value,"is illegal. Only 'enable' and 'disable' are allowed.")
        # SPI address 5 for writing. SPI mode is 'don't care'
        self.spi_rack.write_data(self.module, 5, 0, BICPINS_SPEED, bytearray([data<<4]))

    def mux8by8_write_user_data(self,address,data):
        """
        """
        # send data via controller
        #print("Data for updated state into the shift-register is",data)
        self.spi_rack.write_data(self.module, address, HCT595_MODE, HCT595_SPEED, data)
        
    def mux8by8_write_complex_data(self,address):
        """
        """
        # Preparing the data as bytearray
        data = bytearray([0xf0,0xf0,0xe7,0xc3,0x81,0x42,0x24,0xff])

        # send data via controller
        self.spi_rack.write_data(self.module, address, HCT595_MODE, HCT595_SPEED, data)

    def mux8by8_write_simple_data(self,address):
        """
        """
        # Preparing the data as bytearray
        data = bytearray([255,0,255,0,255,0,255,0])

        # send data via controller
        self.spi_rack.write_data(self.module, address, HCT595_MODE, HCT595_SPEED, data)

    def provide_cfg_manually(self):
        """
        """
        new_cfg_file = input("Provide_cfg_manually: enter the switch configuration file here: ")
        fname = os.getcwd()+'\\'+new_cfg_file

        file_type = fname[-3:]
        file_found = os.path.isfile(fname)
        if (file_type != 'csv'):
            print("Could not update the configuration file. Please select a .csv format")
        elif (file_found==0):
            print("Could not update the configuration file. The file was not found in the run directory: ",os.getcwd(),"\n")
        else:
            self.switch_cfg_file = new_cfg_file

    def provide_cfg_programmatically(self,new_cfg_file):
        """
        """
        fname = os.getcwd()+'\\'+new_cfg_file

        file_type = fname[-3:]
        file_found = os.path.isfile(fname)
        if (file_type != 'csv'):
            print("Could not update the configuration file. Please select a .csv format")
        elif (file_found==0):
            print("Could not update the configuration file. The file was not found in the run directory: ",os.getcwd(),"\n")
        else:
            self.switch_cfg_file = new_cfg_file

    def which_cfg_file(self):
        """
        """
        return self.switch_cfg_file
		
    def check_sample_is_legal(self, sample_num):
        if (sample_num <1 or sample_num >8):
            raise ValueError('Sample number',sample_num,'is illegal. Sample number must be between 1 and 8.')
        return 1
		
    def check_instrument_is_legal(self, instrument_num):
        if (instrument_num <1 or instrument_num >8):
            raise ValueError('Instrument number',instrument_num,'is illegal. Instrument number must be between 1 and 8.')
        return 1

    def edit_connection_in_file(self, sample_num, instrument_num, new_val):
        # input checks
        self.check_sample_is_legal(sample_num)
        self.check_instrument_is_legal(instrument_num)
			
        f1 = open('U1c_user_config.csv', 'r')   # ver3 comment - make this generally the current cfg file
        # identify the delimiter
        csv_info = csv.Sniffer().sniff(f1.read(1024))
        f1.seek(0)
        #print("Debug: csv has delimiter char ",csv_info.delimiter)
        rows_reader=csv.reader(f1, delimiter=csv_info.delimiter)
        rows = []
        for row_num,row in enumerate(rows_reader):
            if (row_num==sample_num):
                #print("row_num is ",row_num,"and sample_num is ",sample_num)
                row[instrument_num]=new_val
            #print("row is ",row)
            rows.append(row)
        rows[sample_num][instrument_num]=new_val   # ver3 comment - is this not unnecessary?
        #print("\nrows is ",rows,"\n")
        f1.close()

        f2 = open('U1c_user_config.csv', 'w', newline="")
        writer = csv.writer(f2,delimiter=csv_info.delimiter)
        for row in rows:
            #print("row is ",row)
            writer.writerow(row)
        f2.close()

    def reset_cfg_file(self):
        """
        Write 0 to all connections in the current cfg file.
        This is useful during production testing - 
        to ensure you start with a blank cfg file
        regardless of previous testing sessions.
        """
        for row_num in range(1,9):
            for col_num in range(1,9):
                self.edit_connection_in_file(row_num, col_num, 0)

    def fully_set_cfg_file(self):
        """
        Write 1 to all connections in the current cfg file.
        This is useful for testing purpoes.
        """
        for row_num in range(1,9):
            for col_num in range(1,9):
                self.edit_connection_in_file(row_num, col_num, 1)

    def audio_notification_connection(self, sample_num = 1, instrument_num = 1):
        """
        Potentially, this funtcion can replace the generic winsound.Beep.
        This function beeps as many time as the sample- / instrument-number, 
        giving the user audio information on the switch state.
        However, if you use this function, it's recommended to remove the 
        default winsound.Beep calls.
        """
        # Make a number of beeps for sample number
        for i in range(sample_num):
            winsound.Beep(800,20)       # freq 800 Hz, duration 50 mS
        # Make a number of beeps for instrument number
        for i in range(instrument_num):
            winsound.Beep(4000,20)      # freq 1600 Hz, duration 50 mS
        return

    def auto_iterate_all_connections(self, measurement_delay_sec = 1):
        """
        Use this function for a functional test, 
        looping over all possible connections.
        """
		# initialization
        self.reset_cfg_file()
        self.update_state()
        prev_row = 1
        prev_col = 1
        print("auto_iterate_all_connections: initialization done")
		
        # Iterate over all connections, one at a time:
        #     Coarse steps over samples (MSB)
        #     Fine steps over instruments (LSB)
        for row_num in range(1,9):
            for col_num in range(1,9):
                # first break the previous connection - EITHER
                #self.edit_connection_in_file(sample_num = prev_row, instrument_num = prev_col, new_val=0)
                # OR 
                self.reset_cfg_file()
                print("\nauto-iterate: configured connection between Sample",prev_row,"to Instrument",prev_col," - off")
                self.update_state()
                print("auto-iterate:  connection between Sample",prev_row,"and Instrument",prev_col,"is broken")
                
                # ... then define the new connection ... 
                self.edit_connection_in_file(sample_num = row_num, instrument_num = col_num, new_val=1)
                print("\nauto-iterate: configured connection between Sample",row_num," to Instrument",col_num," - on")
                prev_row = row_num
                prev_col = col_num
                # ... and apply it.
                self.update_state()
                print("auto-iterate: connection between Sample",row_num," and Instrument",col_num,"is made")
                #self.print_current_state()
                time.sleep(measurement_delay_sec)
				
		# clean-up: reset the U1c after loop is completed
        self.reset_cfg_file()
        self.update_state()
		
        print("auto-iterate: finished checking all connections")
        return
    
    def print_2D_cfg(self, list_in_2d):
        for element in list_in_2d:
            print(element)
        print("\n")
        #print(list_in_2d)
        #df = pd.DataFrame(list_in_2d)
        #print(df.to_string(index=False , header=False))
    
    def print_cfg_from_cfg_file(self):
        self.print_2D_cfg(self.read_new_config_from_csv())
        return

    def print_current_state(self):
        if (self.initialized==1):
            print("Current state is: ")
            print("            instruments number --> \nsample number\n     |\n     |\n     V\n\n")
            self.print_2D_cfg(self.settings_current)
        else:
            print("State was not initialized - cannot reliably determine current state. Possibly the module code was re-set without a HW change.")

    def read_new_config_from_csv(self):
        """
		Reads a csv file with the requested configuration. The file is assumed, and not checked, to be in a csv format and the proper structure.
		
		Returns: an array of original rows from the file.
        """
        fpath = os.getcwd()
        fname = fpath+'\\'+self.switch_cfg_file

        fwrapper = open(fname,'r')
        input_lines = []
        for index,row in enumerate(fwrapper, start =1):
            input_lines.append(row)
        #print("Debug: fwrapper is",fwrapper,"and the requested config is",input_lines,"\n")
        fwrapper.close()
        return input_lines
        
    def extract_settings_from_csv_input(self,input_lines):
        """
		Reads an array of 9 lines, and extracts the contents of lines 2-9 (removing header, delimiters, etc).
		
		Returns: an array of 8 rows (samples), each one with 8 values (8 instrument connections).
        """
        settings_requested = []

        # Read 8 rows (2-9) from .csv file.
        # Each row correponds to a sample.
        for i in range(1,9):
            line = input_lines[i].rstrip('\n')
            #print("For row ",i+1," the stripped line is ",line)
            line_without_commas = list(filter (lambda symb: symb != ",",line))
            line_without_commas = list(filter (lambda symb: symb != ";",line_without_commas))
            #line_extract = ''.join(line_without_commas[7:])  # if better to make a string
            del line_without_commas[:7]
            #print("For row ",i+1," line_without_commas is ",line_without_commas," in type ",type(line_without_commas),"\n")
            settings_requested.append(line_without_commas)
        #print("Debug (extract_from_csv): settings_requested is "); self.print_2D_cfg(settings_requested)  # same-line for readability
        return(settings_requested)
       
    def adapt_settings_to_hardware_mapping(self,settings_initial):
        """
		Adapts a vector of connection flags from a user-readable order to the HW layout order.
        """
        settings_updated = deepcopy(settings_initial)
        settings_updated[0],settings_updated[1]=settings_initial[1],settings_initial[0][::-1]
        settings_updated[2],settings_updated[3]=settings_initial[3],settings_initial[2][::-1]
        settings_updated[4],settings_updated[5]=settings_initial[5],settings_initial[4][::-1]
        settings_updated[6],settings_updated[7]=settings_initial[7],settings_initial[6][::-1]
        #for index,row in enumerate(settings_desired):
        #    print("Debug: the adapted settings desired are",index,row,"\n")
        return(settings_updated)

    
    def get_transitional_rows(self,row_desired = [],row_current = [],transitional_rows=[[]]):
        """
		Receives initial and final states for a selected row:
		    row_current - the initial state of the row
			row_desired - the final state of the row
			transitional_rows - an array of rows, initially containing the current row
		Returns all interim versions of the row from the initial row (as received) to the final row.
			
		Returns: list of rows.
        """
        if row_desired == row_current:
            return transitional_rows # no change to transitional_rows
        else:
            for lit_index,literal in enumerate(row_desired):
                #print(lit_index,literal,row_current[lit_index])
                if literal != row_current[lit_index]:
                    # add a new interim row state, 
					# and update the literal towards the desired state
                    transitional_rows.append(deepcopy(transitional_rows[-1]))
                    transitional_rows[-1][lit_index]=row_desired[lit_index]
                    #print("Debug: transitional_rows after append and edit",transitional_rows,"\n")
                    # call get_transition_rows() to continue with next literals
                    return self.get_transitional_rows(row_desired,transitional_rows[-1],transitional_rows)

    def get_interim_states_from_interim_rows(self, row_num, interim_rows=[[]], interim_states = []):
        """
		Receive a row number and all interim versions of that row. Returns the full-grid states for those rows.
		
		Returns: an array of full states.
        """
        #print("\nDebug (get_interim_states): For row number",row_num," - \n    The given interim_states is:")
        #self.print_array_of_grid_states(interim_states)
        #print("    ... the given interim_rows are", interim_rows,"\n")
        if len(interim_rows) == 0:
            # only 1 element means current and desired rows are the same, so no change from current
            #print("    ... and the final interim_states is unchanged.\n")
            return interim_states
        else:
            for interim_index,interim_row in enumerate(interim_rows):
                # make a new interim state, 
				# with one literal updated w.r.t the previous state
				# in one row only 
				# towards the desired state
                interim_states.append(deepcopy(interim_states[-1]))
                interim_states[-1][row_num]=interim_row
                #print("Debug: interim_states after append and edit",interim_states,"\n")
            #print("    ... and the final interim_states is")
            #self.print_array_of_grid_states(interim_states)
            return interim_states

    def convert_settings_to_bytearray(self,next_settings):
        """
		The SPI write function takes a bytearray, 
		so it's needed to convert a list (of binaries) or str (of binaries) to int.
		Receives a single grid state (full set of 64 connections); returns a bytearray.
		
		Returns: a bytearray ready to send to the Shift-Register
        """
        sr_data = bytearray()    # data for the Shift-Register
        for row_index,row in enumerate(next_settings):
            #print(row_index,row)
            #current_byte = int(''.join(map(str, row)), 2) # method num. 1
            current_byte = sum(int(bit)<<bit_index for bit_index,bit in enumerate(reversed(row))) # alternative method - equivalent
            sr_data.append(current_byte)
        #print("Debug: full settings for next state are",sr_data,"\n")
        return(sr_data)
    
    def print_array_of_grid_states(self, array_of_states):
        if len(array_of_states) == 0:
            # no transition - maintain current state
            print("Array of states is empty (no states given)")
            return 0
        else:
            for trns_num,trns_state in enumerate(array_of_states):
                print("State number ",trns_num,":")
                self.print_2D_cfg(trns_state)
            return 1

    def calc_transition_states(self, settings_desired, transitional_states):
        """
		Calculate interim (transitional) states.
        """
        for row_index,row in enumerate(settings_desired):
            # For a single row at a time, calculate the transitions 
            # e.g. transitional_rows = get_transitional_rows(['0','0','1','1'],['0','0','0','0'],[['0','0','0','0']])
            transitional_rows = self.get_transitional_rows(settings_desired[row_index], self.settings_current[row_index], [self.settings_current[row_index]])
            #print("Debug (update_state) ZZZ: for row number",row_index,"the initial transitional_rows are",transitional_rows)
            del transitional_rows[0]  # current row was used just as a starting point
            #print("Debug (update_state) AAA: for row number",row_index,"transitional_rows as used for state calculation are",transitional_rows)
            
            # Now use the transitional rows to make the full transitional grid states
            transitional_states = self.get_interim_states_from_interim_rows(row_index, transitional_rows, transitional_states)   # using '=' and not 'append' since the append action is already done in 'get_interim_states'
        #print("Debug (update_state) ABAB: initial transitional_states for this update is:")
        #self.print_array_of_grid_states(transitional_states)
        return

    def pass_state_to_HW_module(self, state):
        """
		
        """
        state_adapted = self.adapt_settings_to_hardware_mapping(state)
        updt_data = self.convert_settings_to_bytearray(state_adapted)
        """
        print("Debug (update_state) CCC: next transitional state is")
        self.print_2D_cfg(state)
        print("and next state data to write is ",updt_data)
        """
        self.mux8by8_write_user_data(3,updt_data)
        #M8x8_desk.mux8by8_write_user_data(4,bytes.fromhex('00 00 00 00 00 00 00 00'))
        time.sleep(self.switch_delay)

    def update_state(self):
        """
		
        """
		# Obtain the current settings
		# (it is the starting point for state transitions).
        transitional_states = [deepcopy(self.settings_current)]
        #print("Debug (update_state): the initial transitional_states is ",transitional_states,"\n")

        # Read input and produce the requested settings (a 2D grid of binary literals)
        input_lines = self.read_new_config_from_csv()
        settings_desired = self.extract_settings_from_csv_input(input_lines)
        
        # Calculate interim (transitional) states
        self.calc_transition_states(settings_desired, transitional_states)
        del transitional_states[0]  # current grid state was used just as a starting point. No need to actually 'transition' to it
        #"""
        #print("Debug (update_state) BBB: processing of transitional_states is done. transitional_states for this update is:")
        #self.print_array_of_grid_states(transitional_states)
        #"""
        
        # Now make the transitions
		# ver3 comment - should there be an 'if' for no change?
        for state in transitional_states:
            self.pass_state_to_HW_module(state)

        # when done (i.e. HW is in final desired state), make a beep
        winsound.Beep(800,150)
        winsound.Beep(1600,150)

        #print("Debug (update_state) DDD: Original state was",self.settings_current)        
        # ... and update the local data.
		# Re-read the original file, avoiding the re-map for HW.
        self.settings_current = self.extract_settings_from_csv_input(input_lines)
        #print("Debug (update_state) EEE: After the transision, the state is",self.settings_current)
		
        #self.initialized = 1   # The new state is known only if the initial state was known, so we do not set this to '1'

    def connect_all(self):
        """
		Based on update_state().
		Identical to update_state() except 3 locations marked with CHANGE FROM UPDATE STATE
		It's probably possible to write a utility function that will serve both connect_all() and update_state()
        """
		# Obtain the current settings
		# (it is the starting point for state transitions).
        transitional_states = [deepcopy(self.settings_current)]
        #print("Debug (update_state): the initial transitional_states is ",transitional_states,"\n")

        # CHANGE FROM UPDATE STATE - no need to read the input_lines
        settings_desired = self.settings_all_on
        
        # Calculate interim (transitional) states
        for row_index,row in enumerate(settings_desired):
            # e.g. transitional_rows = get_transitional_rows(['0','0','1','1'],['0','0','0','0'],[['0','0','0','0']])
            transitional_rows = self.get_transitional_rows(settings_desired[row_index], self.settings_current[row_index], [self.settings_current[row_index]])
            #print("Debug (update_state) ZZZ: for row number",row_index,"the initial transitional_rows are",transitional_rows)
            del transitional_rows[0]  # current row was used just as a starting point
            #print("Debug (update_state) AAA: for row number",row_index,"transitional_rows as used for state calculation are",transitional_rows)
            
            # Now use the transitional rows to make the full transitional grid states
            transitional_states = self.get_interim_states_from_interim_rows(row_index, transitional_rows, transitional_states)   # using '=' and not 'append' since the append action is already done in 'get_interim_states'
        #print("Debug (update_state) ABAB: initial transitional_states for this update is:")
        #self.print_array_of_grid_states(transitional_states)
        del transitional_states[0]  # current grid state was used just as a starting point. No need to actually 'transition' to it
        #"""
        #print("Debug (update_state) BBB: processing of transitional_states is done. transitional_states for this update is:")
        #self.print_array_of_grid_states(transitional_states)
        #"""
        
        # Now make the transitions
		# ver3 comment - should there be an 'if' for no change?
        for state in transitional_states:
            state_adapted = self.adapt_settings_to_hardware_mapping(state)
            updt_data = self.convert_settings_to_bytearray(state_adapted)
            """
            print("Debug (update_state) CCC: next transitional state is")
            self.print_2D_cfg(state)
            print("and next state data to write is ",updt_data)
            """
            self.mux8by8_write_user_data(3,updt_data)
            #M8x8_desk.mux8by8_write_user_data(4,bytes.fromhex('00 00 00 00 00 00 00 00'))
            time.sleep(self.switch_delay)

        # when done (i.e. HW is in final desired state), make a beep
        winsound.Beep(800,150)
        winsound.Beep(1600,150)
        
        # ... and update the local data.
        # CHANGE FROM UPDATE STATE - no need to read the input_lines
        self.settings_current = self.settings_all_on
        # CHANGE FROM UPDATE STATE - it is now a known state
        self.initialized = 1   
        
        self.enable_SR_output(value = 'enable')
        
        print("Connect all: done")

    def disconnect_all(self):
        """
		Is it possible to replace this with a direct 
		mux8by8_write_user_data(self,address,data), with data of 0's?
        """
		# Obtain the current settings
		# (it is the starting point for state transitions).
        # In case we have no info on current state (SW reset without HW reset),
        # write all 0s as the initial transition state, and make sure it gets written 
        # i.e. del transitional_states[0] is commented out
        if (self.initialized != 1):
            self.settings_current = deepcopy(self.settings_default)
		# else - the current settings are valid, and we can use them.
		# Notes: 
		#       1. The initial setting cannot be 'settings_all_on', 
		#       since that will turn 'on' switches (close switches) in the interim states,
		#       even those that weren't originally on.
		#       2. After this check we can procees safely to use the self.settings_current 
		#       to calculate the necessary transitions.

        # calculate interim (transitional) states
        transitional_states = [self.settings_current]
        #print("Debug (update_state): the initial transitional_states is ",transitional_states,"\n")
            
		# Produce the requested settings (a 2D grid of binaries)
        settings_desired = self.settings_default
        
        # Calculate interim (transitional) states
        self.calc_transition_states(settings_desired, transitional_states)
        #print("Debug (update_state): with all transitional rows processed, transitional_states is ") # continues on next command
        #for trns_num,trns_stt in enumerate(transitional_states):
        #    print("Transition state number ",trns_num,":")
        #    self.print_2D_cfg(trns_stt)
        
        # DO NOT DELETE automatically transitional_states[0]  ! ! !
        if (self.initialized==1 and transitional_states[0]==self.settings_current):
            del transitional_states[0]  # current grid state was used just as a starting point
        
        # Now make the transitions
        for state in transitional_states:
            self.pass_state_to_HW_module(state)
			
        # when done (i.e. HW is in final desired state), make a beep
        winsound.Beep(800,150)
        winsound.Beep(1600,150)
        
        # ... and update the local data
        self.settings_current = self.settings_default
        self.initialized = 1
        
        self.enable_SR_output(value = 'enable')
        
        print("Disconnect all: done")
		