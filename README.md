# RemoteDMX_ESPNOW
This code aims to control a predefined set of scenes using DMX512 remotely. The remote communication is achieved with ESPNOW.

# Hardware
For the peripheral (which is actually the DMX controller), I have used the Sparkfun ESP32 Thing Plus WROOM: https://www.sparkfun.com/products/15663

I used it with its DMX feather shield: https://www.sparkfun.com/products/15110

I powered this with a USB 5V adapter, and placed it at the beginning of the DMX chain.

For the wireless controller, I used another ESP32 Thing Plus, but that could have been any ESP32 board, since all I needed was ESPNOW. I powered the device with a 3.7V LiPo battery. In my case, I used a 980mAh battery, which proved to be too low of a current for driving the controller when Wifi was turned on. Upon further research I would recommend using a battery with at least 2500mAh, although I didn't test it myself yet. The insufficient current of the battery I was using led me to an irregular and unreliable functioning, which I tried to fix by placing a 1000uF capacitor between GND and 3.3V, which seemed to improve performance but still didn't solve the unstability. Some times it would work just fine, while other times it would trigger the brown out detector and get caught in a never-ending rebooting loop. I then disabled the brown out detection, which seemed to fix the problem, but the device kept turning off unexepectedly.

I wanted to keep the controller minimal, so I only used one button and two knobs: in my setup, the button would allow to move either to the next scene or back to the last. In order to merge both functions in a single button, I have used Kimballa's debounce library: https://github.com/kimballa/button-debounce

A short button press would move to the next scene, while a long button press would take us back to the last. Defining presets of my fixtures as data arrays was a useful way of recalling different configurations for each scene, and especially useful for fixtures that had multiple control channels, such as LED lights or a laser.

As for the knobs, I mapped one to a fog machine and the other to the strobe effect of the lights I was using, but they could be mapped to any parameter.

Additionally, I added a switch to turn the controller on and off, and an additional LED attached to pin 5 to monitor internal working and power stability from the outside of the enclosure.

In this example configuration, which I used in a performance, I was controlling 2 Par LED fixtures, 1 laser, 1 fog machine and a 4-channel dimmer (which I didn't use in the end).
