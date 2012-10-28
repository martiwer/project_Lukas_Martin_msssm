# MATLAB HS12 – Research Plan 


> * Group Name: StuckInTraffic
> * Group participants names: Lukas Stadelmann, Martin Wermelinger
> * Project Title: The influence of a speed limit reduction on a traffic jam

## General Introduction

Traffic jam is getting more and more a problem. The number of cars is increasing every day and the time wasted stuck in traffic is annoying everyone of us.
There's a lot of effort to prevent traffic jams. One notice on swiss highways a speed limit reduction when the traffic is overloaded.
We want to model the effect of the speed limit reduction and see if this method improves the traffic flow.


## The Model

We model a section of a fictitious two lane highway with a high traffic density. 
The driver model is an "intelligent driver model" (maybe extended with the "human driver model"?). So the state of a car depends ond the state of the car in the front and his driver has a finite reaction time. The driver wants to pass the highway as fast as possible, but also without any crash. So he keeps an safety distance, which
depends on the velocity. If you are faster you need more safety distance. Of course the drivers have to respect the speed limit.



## Fundamental Questions

We would like to analyze two cases.

Case 1:
There's no specific event, which causes a traffic jam. We study the influence of the speed limit on self-induced stop-and-go traffic or even traffic jams, that can suddenly appear on crowded highways.
What's the ideal speed limit that allows a high traffic flow and at the same time prevents these phenomena?

Case 2:
We cause a traffic jam by a specific event. For example a sudden temporary slow down of the cars due to the sun which blinds the drivers. We now reduce the speed limit at a certain point some kilometers ahead the traffic jam.
Is it possible to improve the traffic flow with this method or is the traffic jam just displaced?
Where is ideal point for this speed intervention and which is the appropriate speed limit?





## Expected Results

Because this method of reducing the speed limit is actually in use, we hope our models can prove a positive effect on the traffic flow. 
Case 1:
We expect that if there is an high traffic density, one can improve the traffic flow with a lower speed limit (but not too low!).
Case 2:
We estimate there will be a specific place ahead the traffic jam, where a speed reduction has a positive effect on prevent or clear a traffic jam.

## References 

[1]Martin Treiber, Arne Kesting, Dirk Helbing, Delays, inaccuracies and anticipation in
microscopic traffic models February 2005

[2]Martin Treiber and Dirk Helbing,Explanation of observed features of self-organization in traffic flow February 2008



## Research Methods

Cellular Automata



## Other

