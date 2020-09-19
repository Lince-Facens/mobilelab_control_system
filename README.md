# mobilelab_control_system

Software for reading actuation commands from the AI system and translate it to electrical signals to control the vehicle actuators

## Inputs
* A0 - Steering Feedback (Analog) - The potentiometer value that indicates the wheel position
* A1 - Right Steering (Analog) - The right steering output from the data gatherer system
* A2 - Left Steering (Analog) - The left steering output from the data gatherer system
* A3 - Acceleration (Analog) - The acceleration from the data gatherer system
* A8 - Control Enabled (Analog) - Whether the controller is enabled (received from mobilelab_data_gatherer)
* A10 - Serial Rx (Digital) - Receiver for serial communication with the AI system 
* B15 - Reverse Acceleration (Digital) - Whether the acceleration is backwards

## Outputs
* A6 - Right Steering (PWM)
* A7 - Left Steering (PWM)
* B0 - Acceleration (PWM)
* A9 - Serial Tx (Digital)
* C13 - Indicator Led (Digital) - Used as a visual indicator
* B12 - Enable Left (Digital) - Whether the left steering is enabled
* B13 - Enable Right (Digital) - Whether the right steering is enabled
* B14 - Still alive (Digital) - Whether the system is alive or not
