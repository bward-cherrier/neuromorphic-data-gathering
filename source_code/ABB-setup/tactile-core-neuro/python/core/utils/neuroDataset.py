# -------------------------------------------------------------------------------------------------------------------------------------------------------
# Created By  : George Brayshaw
# Created Date: 25/04/2023
# version ='1.0'
# python version = '3.8.10'
# ------------------------------------------------------------------------------------------------------------------------------------------------------
"""This file provides a PyTorch dataloader class for use with LavaDL

Class has been tested on python 3.8.10 but should work on anything >3.6 (anyone willing to test this please report back).

NOTE: This dataloader is only useful for classification tasks. Regression tasks will require a different dataloader.
"""
# ------------------------------------------------------------------------------------------------------------------------------------------------------
import numpy as np
import pickle
import torch
import math
import glob
import matplotlib.pyplot as plt
from torch.utils.data import Dataset, DataLoader
import lava.lib.dl.slayer as slayer
from nums_from_string import get_nums


class neuroDataset(Dataset):
    def __init__(self, PATH,        # Path to folder containing the train and test subfolders
                 sampling_time=1,  # Samplet = 100us = 1ms time of neuroTac in ms, Default = 1ms
                 sample_length=5000,  # Max length of each sample in ms, Default = 5000ms
                 train=True,        # Bool for if wanting training or test data. Default = True
                 x_size=180,             # Size of the array in the x direction
                 y_size=240              # Size of the array in the y direction
                 ):

        super(neuroDataset, self).__init__()

        # Load data
        self.train = train
        self.PATH = PATH

        # Set path to whichever dataset you want
        if self.train is True:
            self.PATH = f"{PATH}/train/"
        else:
            self.PATH = f"{PATH}/test/"

        # Filenames of data samples
        self.samples = glob.glob(f'{self.PATH}/**/*.pickle')  # .bin for binary
        self.sampling_time = sampling_time
        self.num_time_bins = int(sample_length/sampling_time)
        self.x_size = x_size
        self.y_size = y_size

    # Function to retrieve spike data from index
    def __getitem__(self, index):
        """
        Private method called during training by the Lava training assistant. Used to retrieve a datapoint and it's corresponding label
        """
        filename = self.samples[index]

        # Get the folder name that contains the file for label
        label = int(filename.split('/')[-2])
        event = self.__create_events(filename)

        spike = event.fill_tensor(                      # torch.zeroes(channel, height, width, )
            torch.zeros(1, self.y_size, self.x_size,        # Swapped x and y sizes for batch sizes - self.x_size, self.y_size
                        self.num_time_bins, requires_grad=False),    # Change back to requires_grad = False if causing issues
            sampling_time=self.sampling_time)
        # print(spike)
        return spike.reshape(-1, self.num_time_bins), label

    def __len__(self):
        """ Private method that returns the length of the dataset
        """
        return len(self.samples)

    def __create_events(self, filename, ON_OFF=1):
        """ Private method to convert data to a lava SLAYER compatible Event object. Used in the __getitem__ method
        Arguments
        ---------
        ON_OFF:     int (default = 1)
                        Int either 0 or 1 to indicate the channel of the data. For some reason SLAYER reads in my data with 0 events (wanted 1 events)
        """
        with(open(filename, "rb")) as openfile:
            try:
                data = pickle.load(openfile)
            except EOFError:
                print(EOFError)

        # Convert to AER and then read back string - this avoids issues with sorting in timestamp order
        # Create a temporary list to contain information for all events
        temp_list = []

        # Cycle through each nested list and check if empty
        # Check row
        for y in range(data.shape[0]):
            # Check column
            for x in range(data.shape[1]):
                # Check if pixel (x,y) is empty
                if data[y, x]:
                    # Cycle through all events in this pixel
                    for spike in data[y, x]:
                        # Currently we only input ON events
                        temp_list.append([x, y, ON_OFF, spike])

        # Order by timestamp with earliest spike first
        sorted_list = sorted(
            temp_list, key=lambda x: int(x[3]), reverse=False)

        # Move sorted list elements into arrays
        x_array = []
        y_array = []
        ts_array = []

        #print(f"Length of sorted list = {len(sorted_list)}")
        for event in range(len(sorted_list)):
            x_array.append(sorted_list[event][0])
            y_array.append(sorted_list[event][1])
            ts_array.append(sorted_list[event][3])

        channel_array = np.zeros(len(x_array))

        # Combine arrays into Event object
        # CHWT format
        td_event = slayer.io.Event(
            x_array, y_array, channel_array, ts_array, self.num_time_bins)   # np.multiply(ts_array, 1000)

        return td_event
