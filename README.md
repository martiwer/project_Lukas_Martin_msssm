# MATLAB HS12 – Research Plan 


> * Group Name: StuckInTraffic
> * Group participants names: Lukas Stadelmann, Martin Wermelinger
> * Project Title: The influence of a speed limit reduction on a traffic jam

## General Introduction

Traffic jam is getting more and more a problem. The number of cars is increasing every day and the time wasted stuck in traffic is annoying everyone of us.
There's a lot of effort to prevent traffic jams. One notice on swiss highways a speed limit reduction when the traffic is overloaded.
We want to model the effect of the speed limit reduction and see if this method improves the traffic flow.


## The Model

We model a section of a fictitious highway with a high traffic density or even a traffic jam, just with a single lane. The driver model is an "intelligent driver model" (maybe extended with the "human driver model"?).
So his state of driving depends ond the state of the car in the front and he has a finite reaction time. A driver wants to pass the highway as fast as possible, but also without any crash. So he keeps an safety distance, which
depends on the velocity. If you are faster you need more safety distance. Of course the drivers have to respect the speed limit.



## Fundamental Questions

Analyze the influence of different speed limits to the traffic flow.
Does a speed limit reduction prevent an annoying stop-and-go traffich or can one even prevent a traffic jam with a speed limit reduction?
So is it at all possible to improve the traffic flow with a speed limit reduction?
What is the ideal speed limit at a specific traffic density and how stable is the system in this case? (Does it drop suddenly?)




## Expected Results

Because this method of reducing the speed limit is actually in use, we hope our model can prove a positive effect on the traffic flow. We expect that
if there is an higher traffic density, one can improve the traffic flow with a lower speed limit (but not to low!).


## References 

[1]Martin Treiber, Arne Kesting, Dirk Helbing, Delays, inaccuracies and anticipation in
microscopic traffic models February 2005

[2]Martin Treiber and Dirk Helbing,Explanation of observed features of self-organization in traffic flow February 2008



## Research Methods

Cellular Automata



## Other

