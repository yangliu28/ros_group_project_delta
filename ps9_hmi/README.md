# ps9_hmi

This package includes a library that provides the functions of a human-machine interface to our baxter final project.
The human-machine interface works through the detection of a human hand that is used to place the colored block on the stool in front of baxter.
The hand can be left in front of baxter to postpone action until its removal, signaling baxter to begin its routine.

## Example usage

The library within provides a public function belonging to a HumanMachineInterface object.
The object requires a ros nodehandle to be constructed, but otherwise handles all other aspects of its functions.

The function get_human_hand() returns a boolen that acts as a flag for hand detection.
A true value returned represents that there is a hand in front of baxter that was detected by the kinect.
A false value means there was no hand present.

## Running tests/demos

A test file is included in the library that can be ran to test the HMI function, as well as aid in calibrating a kinect with baxter for use in the human machine interface.

The test file, when run, will continue to execute and update its progress, until it successfully detects a human hand.