# Final Project Concept
#### Collaborator: 
Luīze Rieksta

#### Main themes: 
* symbiosis (specifically mutualism)
* helping each other
* age
* navigation
* communication

#### Characters (robots):
1. The Navigator: 
    * An old school boat which knows where it wants to go, but cannot move on its own. 
    * Has two flags to communicate using flag semaphore (each flag position corresponds to a different number or letter in the alphabet).
1. The Wanderer: 
    * A modern boat which can move around on its own, but does not know where it wants to go. 
    * Has a fan that just keeps blowing.
    * Can communicate by making inexplicable sounds.
   
#### Story:
* First, you see the Wanderer on stage, who is aimlessly wandering around.
* Then it suddenly spots the Navigator and the two boats talk to each other.
  * The Navigator talks with its flags.
  * The Wanderer talks using inexplicable sounds. 
  * Since each boat bot needs help, they make the effort to try to understand each other.
* The Navigator suggests to the Wanderer that it can use its fan to help the Navigator move. In return, the Navigator can direct the Wanderer where to go. Thus both their problems are solved and they sail off into the distance happily.

#### Moral/So What:
* We want to show the value in helping others solve their problems and in working together as team.
Setting:
#### Projections:
  * Water on the ground to simulate waves (need a projector hanging from the ceiling)
  * Constellations on the wall to simulate a starry night sky (need a projector facing a wall, preferably projected from behind)
  * It would be nice if we could somehow replicate the setup that TPO used for the Farfalle show so that we can seamlessly create the sea, horizon, and starry night sky
  * Somehow project letters onto the wall to translate one letter at a time what the Navigator is saying to the audience (maybe somehow incorporate this translation using the stars to make the letters)
#### Sounds:
  * Wanderer’s communication through inexplicable sounds that somehow make sense to convey what its thinking
  * Sea-scape ambient sounds, including the sounds of waves and the wind when the two boat bots move in sync
  * Possibly have a pre-recorded narration of what’s happening, or have someone narrate with a mic during the show

#### Technical specifications for the robots:
* Size:
  * Somewhere around the size of a tissue box for each robot
* Mechanical structure:
  * Laser cut the base for each robot (wood for the Navigator, acrylic for the Wanderer)
  * Flags for the Navigator that can rotate position easily
  * Fan for the Wanderer that’s strong enough to push the Navigator around
  * Sails for the Navigator and maybe the Wanderer
  * Caster wheels for both robots (four or five total)
  * Two regular wheels for the Wanderer
* Electronics:
  * 2 Xbee Explorers and 2 XBee Breakouts
  * Adafruit Bluefruit LE SPI (to control the robots remotely with a phone)
  * 2 Arduino Uno’s
  * 2 small servos (for the flags)
  * A very strong motor
  * Something that can play from a set of pre-recorded sounds
  * 2 projectors
  * A screen/wall/sheet to project onto
  * A mic for on the spot narration or a speaker for pre-recorded narration
  * Lots of batteries (need separate batteries to power 2 servos, 2 Arduinos, 2 regular 5V DC motors, and 1 powerful motor (the one from the drill, which I think is 7.2 V).
  * If I do decide to/manage to figure out how to use computer vision and fiducials for making the robots move automously:
    * 2 small web cams
    * 2 Raspberry Pi’s
