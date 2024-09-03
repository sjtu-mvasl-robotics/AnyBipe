"""
@file Rate.py

© [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import time

class Rate:
    def __init__(self, frequency):
        """
        Constructor for the Rate class.
        
        Args:
            frequency (float): The frequency in Hz (cycles per second).
        """
        self.frequency = frequency
        self.period = 1.0 / frequency  # Calculate the time period for one cycle
        self.start_time = time.time()  # Get the current time

    def sleep(self):
        """
        Sleeps for the appropriate time to maintain the desired frequency.
        """
        end_time = time.time()  # Get the current time
        elapsed = end_time - self.start_time  # Calculate time elapsed since last sleep

        # Check if the elapsed time is less than the desired period
        # If so, sleep for the remaining time to achieve the desired frequency
        if elapsed < self.period:
            remaining_time = self.period - elapsed
            time.sleep(remaining_time)

        self.start_time = time.time()  # Update the start time for the next cycle

    def reset(self):
        """
        Reset the start time.
        """
        self.start_time = time.time()  # Reset the start time to the current time