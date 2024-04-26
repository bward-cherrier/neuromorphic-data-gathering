# -------------------------------------------------------------------------------------------------------------------------------------------------------
# Created By  : George Brayshaw
# Created Date: 03/May/2022
# version ='1.3'
# python version = '3.8.10'
# ------------------------------------------------------------------------------------------------------------------------------------------------------
"""This file provides a data processing class for manipulating neuroTac data

File contains 2 dependancies NOT included in a base Python 3.8.10 installation (numpy and nums_from_string).
Class has been tested on python 3.8.10 but should work on anything >3.6 (anyone willing to test this please report back).

TODO: Refactor messy code throughout
"""
# ------------------------------------------------------------------------------------------------------------------------------------------------------
import pickle
import numpy as np
import os
import nums_from_string     # pip install nums_from_string
import time
import torch  # Used to create torch tensor for integration with lava
import lava.lib.dl.slayer as slayer  # Used for creating the Events object


class DataProcessor():
    """ Class to process neuroTac data. Use the .load_data method to load from a previously saved file
    Arguments
    ---------
    data:   numpy array of lists
                neuroTac sample to process
    AER:    Bool (default=False)
                Bool to indicate if loaded data is in AER format
    """

    def __init__(self, data=None, AER=False):

        self.data = data
        # Flag to indicate if data has been converted to AER and thus made unavailable for most  operations
        self.__AER = AER

    @classmethod
    def load_data(cls, path: str, AER: bool = False):
        """ Class method to load data in from a file

        path (string): Path string to the location of data if load_data = True

        """
        with(open(path, "rb")) as openfile:
            try:
                data = pickle.load(openfile)
            except EOFError:
                print(EOFError)

        return cls(data, AER=AER)

    def data_show(self):
        """" Function to print the current data stored in the class
        """
        print(self.data)

    def remove_empty(self):
        """" Function to remove empty lists from input data
        """
        if not self.__AER:

            new_list = np.empty_like(self.data)

            for row in range(self.data.shape[0]):
                for column in range(self.data.shape[1]):
                    # Use list(set()) to remove duplicates in list (sets cannot contain dupes)
                    if self.data[row, column] != []:
                        new_list[row, column] = self.data[row, column]

            self.data = new_list

        else:
            raise ValueError(
                'Data is in AER format and cannot perform remove_empty operation')

    # Function is currently bugged
    def remove_duplicates(self):
        """ Function to remove duplicate lists from input data
        """
        if not self.__AER:
            new_list = np.empty_like(self.data)

            for row in range(self.data.shape[0]):
                for column in range(self.data.shape[1]):
                    # Use list(set()) to remove duplicates in list (sets cannot contain dupes)
                    new_list[row, column] = (list(set(self.data[row, column])))

            self.data = new_list

        else:
            raise ValueError(
                'Data is in AER format and cannot perform remove_duplicates operation')

    def remove_cuttoff(self, cuttoff, remove_dup=False, remove_em=False):
        """ Function to remove events after a specified cuttoff point
        Arguments
        ---------
        cuttoff:        int
                            Cuttoff point after which all entries in nested list should be removed
        remove_dup:     Bool (default=False)
                            Bool to indicate if you wish to remove duplicate spikes before processing
        remove_em:      Bool (default=False)
                            Bool to indicate if you wish to remove empty pixels before processing                        
        """
        if not self.__AER:
            if remove_dup:
                self.remove_duplicates()
            if remove_em:
                self.remove_empty()

            new_list = np.empty_like(self.data)

            for row in range(self.data.shape[0]):
                for column in range(self.data.shape[1]):
                    # Create new list for each pixel, only containing values lower than cuttoff
                    new_list[row, column] = [
                        element for element in self.data[row, column] if element <= cuttoff]

            self.data = new_list

        else:
            raise ValueError(
                'Data is in AER format and cannot perform remove_cuttoff operation')

    def pixel_reduction(self, x_reduce_l, x_reduce_r, y_reduce_t, y_reduce_b):
        """ Function to reduce the number of pixels in output image from the neuroTac

        Arguments
        ----------
        x_reduce_l:   int
                        Total number of pixels to remove from the left of the array
        x_reduce_r:   int
                        Total number of pixels to remove from the right of the array
        y_reduce_t:   int
                        Total number of pixels to remove from the top of the array
        y_reduce_b:   int
                        Total number of pixels to remove from the bottom of the array
        Returns
        -------
        reduced_image:  nested list (array of lists)
                            New cropped array of timestamps
        """
        if self.__AER is False:
            # Find shape of input data
            y_size, x_size = self.data.shape

            # Find number of pixels to crop from right and bottom
            x_r = x_size - x_reduce_r
            y_b = y_size - y_reduce_b

            # Create new array as a slice of old array
            reduced_image = self.data[y_reduce_t:y_b, x_reduce_l:x_r]

            self.data = reduced_image
            # return reduced_image

        else:
            raise ValueError(
                'Data is in AER format and cannot perform pixel_reduction operation')

    def __bitstring_to_bytes(self, s):
        """
        Private method used in the convert_to_aer function
        """
        v = int(s, 2)
        b = bytearray()
        while v:
            b.append(v & 0xff)
            v >>= 8
        return bytes(b[::-1])

    def convert_to_aer(self, ON_OFF=1):
        """ Function to create convert pickled neuroTac data into aer format .bin file

        Arguments
        ----------
        ON_OFF:     Integer
                        Input to state whether data contains OFF events. 0 = OFF events included. Default = 1
        """

        # Create a temporary list to contain information for all events
        temp_list = []

        # Cycle through each nested list and check if empty
        # Check row
        for y in range(self.data.shape[0]):
            # Check column
            for x in range(self.data.shape[1]):
                # Check if pixel (x,y) is empty
                if self.data[y, x]:
                    # Cycle through all events in this pixel
                    for spike in self.data[y, x]:
                        # Currently we only input ON events
                        temp_list.append([x, y, ON_OFF, spike])

        # Order by timestamp with earliest spike first
        sorted_list = sorted(temp_list, key=lambda x: int(x[3]), reverse=False)

        # Create a byte for entire datasample
        string = bytearray()

        # Convert to 40 bit binary number
        for element in sorted_list:

            # Add each of x,y,p & ts of the list to the byte array
            #print(f'element[0] = {element[0]}')
            if element[0] == 0:
                string.extend(bytearray([0]))
            else:
                string.extend(bytearray([element[0]]))

            #print(f'element[1] = {element[1]}')
            if element[1] == 0:
                string.extend(bytearray([0]))
            else:
                string.extend(bytearray([element[1]]))

            #print(f'element[2] = {element[2]}')
            if element[2] == 0:
                string.extend(bytearray([0]))
            else:
                string.extend(bytearray([element[2] << 7]))

            # Timestamp cannot be 0
            # If it's less than 1 byte add 1 byte padding
            if element[3] < 256:
                string.extend(bytearray([0]))
                string.extend(bytearray([element[3]]))
            # If it's less than 2 bytes add 1 bytes padding
            elif 256 <= element[3] <= 65536:
                string.extend(self.__bitstring_to_bytes(
                    '{0:016b}'.format(element[3])))
            # If it's larger than 2 bytes, no padding required

            # Function can only handle ts of under 65536 currently
            else:
                # string.extend((bytearray(['{0:024b}'.format(element[3])])))
                print("Function cannot handle timestamps > 65536 currently")
                return "Error"

        # Set AER flag to True so that other functions cannot use their operations
        self.__AER = True
        self.data = string

    def save_data(self, PATH):
        """ Function to save processed data
        Arguments
        ---------
        PATH:       string
                        Path to save location for data. NOTE: Currently you must specify the filename and type in this PATH string                      
        """
        with open(PATH, 'wb') as pickle_out:
            pickle.dump(self.data, pickle_out)
            pickle_out.close()

    def offset_values(self, offset, reduce=False):
        """ Function to offset input data to avoid negative spike times
        Arguments
        ---------
        offset:     int
                        offset to add to each event in ms    
        reduce:     bool
                        Set to true if data is already offset and should be clipped back rather than forwards. Default = False                  
        """
        data_y, data_x = self.data.shape

        # Loop through array
        for y in range(data_y):
            for x in range(data_x):
                temp_list = []
                # If list isn't empty
                if self.data[y, x]:
                    # If need to clip data back then remove all before offset and reduce all values by offset
                    if reduce:
                        temp_list = [(spike - offset)
                                     for spike in self.data[y, x] if spike >= offset]
                    # Else simply add the offset to each spike
                    else:
                        temp_list = [(spike + offset)
                                     for spike in self.data[y, x]]

                self.data[y, x] = temp_list

    def create_events(self, ON_OFF=1):
        """ Function to convert data to a lava SLAYER compatible Event object
        Arguments
        ---------
        ON_OFF:     int (default = 1)
                        Int either 0 or 1 to indicate the channel of the data. For some reason SLAYER reads in my data with 0 events (wanted 1 events)
        """
        if not self.__AER:
            # Convert to AER and then read back string - this avoids issues with sorting in timestamp order
            # Create a temporary list to contain information for all events
            temp_list = []

            # Debug
            #print(f"Maximum number of rows (y max) = {self.data.shape[0]}")
            #print(f"Maximum number of columns (x max) = {self.data.shape[1]}")

            # Cycle through each nested list and check if empty
            # Check row
            for y in range(self.data.shape[0]):
                # Check column
                for x in range(self.data.shape[1]):
                    # Check if pixel (x,y) is empty
                    if self.data[y, x]:
                        # Cycle through all events in this pixel
                        for spike in self.data[y, x]:
                            # Currently we only input ON events
                            temp_list.append([x, y, ON_OFF, spike])

            # Order by timestamp with earliest spike first
            sorted_list = sorted(
                temp_list, key=lambda x: int(x[3]), reverse=False)
            #print(f"Total number of events = {len(sorted_list)}")

            # Move sorted list elements into arrays
            x_array = []
            y_array = []
            ts_array = []

            #print(f"Length of sorted list = {len(sorted_list)}")
            for event in range(len(sorted_list)):
                x_array.append(sorted_list[event][0])
                y_array.append(sorted_list[event][1])
                ts_array.append(sorted_list[event][3])

            # Should this be zeros or ones?
            channel_array = np.zeros(len(x_array))

            # Combine arrays into Event object
            # CHWT format
            td_event = slayer.io.Event(
                x_array, y_array, channel_array, ts_array, 1000)   # np.multiply(ts_array, 1000)
            self.data = td_event

            return td_event

        else:
            raise ValueError(
                'Data is in AER format and cannot perform create_tensor operation')


# Testing of the class
if __name__ == "__main__":

    path_to_data = "./Artificial Dataset 9Texture No. 10.pickle"
    OUTPUT_PATH = "./test.bin"

    # Import data to the DataProcessor class
    proc = DataProcessor(load_data=True, path=path_to_data)
