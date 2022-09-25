#!/usr/bin/env python3

import pyfirmata
import time

if __name__ == '__main__':
    # Initiate communication with Arduino
    board = pyfirmata.Arduino('YOUR_PORT_HERE')
    print("Communication Successfully started")
    
    # Create bunch of useful variables
    button = board.digital[5]
    LED1 = board.digital[9]
    LED2 = board.digital[10]
    LED3 = board.digital[11]
    LED4 = board.digital[12]

    LEDs = [LED1, LED2, LED3, LED4] 
    LED_index = 0
    previous_button_state = 0
    
    # Start iterator to receive input data
    it = pyfirmata.util.Iterator(board)
    it.start()

    # Setup LEDs and button
    button.mode = pyfirmata.INPUT
    
    for LED in LEDs:
        LED.write(0)

    # The "void loop()"
    while True:
        # We run the loop at 100Hz
        time.sleep(0.01)
        
        # Get button current state
        button_state = button.read()
        
        # Check if button has been released
        if button_state != previous_button_state:
            if button_state == 0:
                print("Button released")

                # Power off current LED 
                # and power on next LED
                LEDs[LED_index].write(0)
                LED_index += 1
                if LED_index >= len(LEDs):
                    LED_index = 0
                LEDs[LED_index].write(1)
            
        # Save current button state as previous
        # for the next loop iteration
        previous_button_state = button_state