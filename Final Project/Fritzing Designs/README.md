## File names and what they contain:
* navigator-trial01: contains the original circuit (an XBee explorer breakout and two servos)
* navigator-trial02: contains only the XBee explorer breakout. This is assuming that I’m using the motor shield and thus no longer need an external circuit for the servos. This also means that I can use the same circuit for the wanderer. 
* controller-trial01: contains a nice looking schematic of the whole circuitry using the actual components.
* controller-trial02-assuming-using-wires: I got confused how to design the PCB without resorting to using two sides. Also, considering that I would like some flexibility in the locations of the buttons in the box that I’ll mount this circuit to, I figured that I’ll be using wires between the buttons/joysticks and the PCB rather than mounting the components onto it directly. So I replaced every component with female headers. 
* controller-trial03-weird schematic: But I still ran into the problem of trying to design the PCB on one board only, which led me to ponder: why not just use a separate line of female headers for ground and for the 5V, instead of doing it component by component. I suppose this is a rather peculiar way of designing a PCB though, but it simplified the problem and design process significantly.
* controller-trial04-extra-buttons-and-pots: I decided to add extra spaces for a button and two pots in case they might become handy as the construction process of robots progresses. Considering last Saturday’s discussions regarding the use of lights to put emphasis on each robot while it’s speaking, having the pots might help to dim the lights on and off on cue. Or I could use the button instead for this, but I only had one button space left, and I think fading the lights on/off might create a nicer effect during the performance.
* xbee-trial01-only-xbee: a tiny board which only has the XBee explorer breakout and space for the wires that need to be connected to the Arduino
* xbee-trial02-half-shield-effect: I just made a rectangle that only covers part of the Arduino, but added space for stackable headers to still be able to plug it into the Arduino easily. By covering up only part of the Arduino, I can still have access to the servo pins underneath and the VIN jumper for while I’m prototyping with my laptop.
## Questions for Michael:
* Is there a way to cut off part of the PCB, since the shield will create problems when trying to attach the servos below it?
* Does what I did for the controller PCB in the end (trials 3 and 4) make sense? Or what’s a better way to go about designing this particular circuit?