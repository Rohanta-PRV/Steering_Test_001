
2026 Update: I have no idea where it went, but I lost the Nextion Editor app on my laptop entirely...[click to pay respect](https://media1.tenor.com/m/tnphPiAztMIAAAAC/skeleton.gif)
[here for free candi](https://media.tenor.com/6yfREp7Mt4gAAAAi/skull-rotating-skull.gif)

This is the roughest of the roughest documentation of the software/embedded systems work done on the steering wheel/dashboard of PRV25 by Pravega Racing. 
Read along to lose braincells =P

# NEXTION DISPLAY
## WHAT I DID

1) made a draft layout on canva, asked around what values was important, what must be added and where on the screen.
2) made the base black (#000000) background with a thin white grid pattern
3) Decided where the different values and their text would go with all drivers and others
4) included only icons for different sensor values. left space for the text and the values and progress bars to sit (values position could only be done on the nextion editor).
5) finally exported it out as png

6) installed and setup nextion editor
7) created the first page, called it the main page
8) placed the dashboard base image on the main page
9) Placed the text boxes with appropriate label in the font of choice (Avionics, or Raleway, or some other shit )
	1) Took note of all the textboxes' box IDs allowing them to get updated on the go.
10) placed the number value boxes and the progress bars
	1) must config their min and max values, and what it should default to
	2) Took note of all the box IDs to allow them to get updated in real time.

11) took a video/gif of the smoking pravega logo (<3 second one), extracted and downloaded all the frames of the video. (the longer the video, the more work u need to do, because it was 24 fps)
12) imported all the frames of that into pages before the main page.
	1) Maybe memory was getting full because of this.
	2) Problem: every bootup started normally, but stopped after around 2-3 seconds.
	3) But what if the memory of the values updating was causing the display to stop updating...
		1) not it, because the display kept incrementing the brake input percentage, albeit for a few seconds.
		2) Proving that the display was updating values, irrespective of the display storage.
		3) It was a software problem, idfk y it stopped working...fuckkk
13) ...
14) Once done, moved onto stm32cubeide to code the mc to read the CAN data.[[STEERING WHEEL DOCUMENTATION ROUGH#STM32CUBEIDE WORK]]



# STM32CUBEIDE WORK

## WHAT I DID

1) Went on to smt32cudeIDE, to code the mc to read the incoming CAN data, and acc to that, control the display by updating the certain value box (acc to box ID) for the specific parameter. started to pull my hair apart.
2) After successfully seeing the values updating, moved on to error handling.[[STEERING WHEEL DOCUMENTATION ROUGH#ERROR HANDLING]]

### ERROR HANDLING

NOTE: NOT FINAL, THIS IS A DRAFT, AN IDEA, NOT YET REVIEWED AND/OR FINALISED.

11) Defined what errors could happen, and how to identify (and potentially diagnose) them on the fly (by colour coding the errors). Below is a draft category of the errors on the car.
- Needed very urgently because of poor/time-consuming debugging process. We would waste almost entire nightslips just to find what error it was, and 8/10 times, it would turn out to be something so small, yet crucial. This was one of the main things that held back its further development, leaving us in a vicious, meaningless cycle of disappointment, anger, hopelessness, and relief.

- Error Handling CATEGORIZATION 1 (HW/SW)
	1) What category?
		1) Hardware
			1) Mechanical
				1) Engine sensors?
					1) Abnormal engine param values like overheating etc.
						1) Oil temp
						2) coolant temp
						3) radiator fan stopped working
				2) Transmission sensors?
					1) Pneumatic shifting solenoid blocked?
					2) low pressure in CO2 canister?
				3) Suspension sensors (if attached when testing) (no real need for this)
					1) Values exceeding threshold set for suspension component
				4) dik...
					1) ...
			2) Electronic
				1) idk, maybe too advanced????
		2) Software
			1) CAN bus
				1) No signal found
				2) signal found, no data available
				3) signal found, mismatch in baud rate (idk how the system would identify this event)
				4) signal found, data available, can't parse data
				5) signal found, data available, parse data, invalid values (incorrect data type of sum shit)
				6) signal found, data available, parse data, valid values, (another error to be thought about and included)
			2) ECU
				1) No data being received
			3) Datalogger
				1) No data being received

- Error Handling CATEGORIZATION 2 (component specific categorization)
	1) What component?
		1) Engine
			1) ...
		2) Transmission/Shifting
		3) Electronics

- Error Handling CATEGORY 3 (priority to driver safety, then vehicle, then) (doesn't make sense, cuz what driver will know is what engineer will also know, so no point)
	1) Which category?
		1) Racecar Driver Critical? (relevant to the )
		2) System Critical?
		3) ...
