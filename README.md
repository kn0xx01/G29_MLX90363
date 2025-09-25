This is a Arduino Leonardo Test Project for G29 and MLX90363.

Long Story Short: G29 Mainboard got mayhem.
- No forcefeedback
- No Calibration initial procedure
- No Steering - GHUB shows wheel at -450º
- Power Supply is good, tested, and used another supply to confirm.
- Diodes and Mosfets got replaced ( not all, lack of tools and skill to be honest) as per this video: https://www.youtube.com/watch?v=7N3erLtbl4U
- USB connection works, and registers Buttons and Pedals, gets detected in GHUB.
- Using Leonardo to make the Steering wheel work in games. G29 buttons and pedals USB + Leonardo USB for the Steering

Since its problem with Power to the DC Motors and G29 failsafe procedure. Until I properly replace all mosfets and diode and further testing
I started this hell of a journey to at least make the steering work ( initially without FFB or Motors code ).

Using some repos out in the wild for G29, specially:
https://github.com/popos123/Logitech-G29-Motherboard-Replacement

Thanks popos123! -  G29 Pin Outs, Base code of working and concept using 2 board and EMC Utility. 

MLX90363 cable pinouts to Leonardo R3.
// yellow - CS  -> Pin 10
// orange - SCK -> Pin 13 (hardware)
// blue   - MISO-> Pin 12 (hardware) 
// green  - MOSI-> Pin 11 (hardware)
// red    - 5V -> 5V
// black  - GND -> GND



this project is concept of ideias from AFFBWheel and FreeJoy, but not perfect
The code still has issues with Angle Drifting if turning to fast multiple times. Can sustain readings and angles for slower pace like ETS.
Needs SerialCOM command to "CALIBRATE_CENTER" then use Windows Joystick Panel (joy.cpl) to do the rest of the calibration.


Using USB from G29 + USB from Leonardo, it should appear 2 peripherals...
If G29 is connected to the PC, it wont let you calibrate Arduino Leonardo, for some reason. ( disconnect G29, calibrate Leonardo, reconnect G29 )


Repos used in this project:
MLX90363 extensive library : the amazing work of Robert's Smorgasbord on details of MLX90363 (library on description of video) - https://www.youtube.com/watch?v=HDymx1VU_a8
Joystick With FFB : This is a big library because of FFB https://github.com/YukMingLaw/ArduinoJoystickWithFFBLibrary.

The project is not optimized neither finished, there is still a lot to work out, specially with Leonardo storage limitations for coding.

I did learn alot with this project, but to be honest, 95% of code was from AI tools 
( i have no shame, because this is really out my league here )

The is ground base for further works, not intended for regular use yet.

