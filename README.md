Stopping Distance Calculator
=============================

A Python script that estimates a vehicle's stopping distance under various conditions.

Features
------------

Car Database

• Choose from a preset list of cars (e.g., Toyota Corolla, Mazda CX-5)
• Each car has approximate mass, drag coefficient, and frontal area

Manual Entry

• Enter your car's mass for a simpler friction-based calculation

Road & Weather

• Multiple surfaces supported (dry asphalt, wet asphalt, snow, ice) with associated friction coefficients
• Customizable friction coefficients for advanced users

Tyre Condition & ABS

• Optional tyre condition modification (good, decent, poor)
• Adjustable ABS factor to modify effective friction

Driver Reaction Time

• Switch between "Not tired" (~1 s reaction) and "Tired" (~2 s reaction)

Road Slope

• Simulate slight, moderate, or steep inclines/declines
• Calculate how slope affects stopping distance

Speed Variation

• Demonstrate the impact of speeding on stopping distances
• Compare ±10 km/h and ±20 km/h offsets from a baseline speed

Plots
--------

• Speed vs. Distance charts (in km/h vs. meters)
• Multiple subplots for clarity, each showing:
	+ Dry/wet conditions
	+ Tired/alert driver reactions
	+ Speed offset variations

How to Run
--------------

Prerequisites

• Python 3 installed
• matplotlib available (pip install matplotlib)

Running the Script

1. Copy and run: python3 stopping_distance.py
2. Follow prompts for:
	* Car selection (or manual mass entry)
	* Road/weather condition
	* Tyre quality
	* ABS presence
	* Driver reaction time
	* Road slope
	* Speed
