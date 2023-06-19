# Give A Ride ?

## General Background Of The Problem

In a city, there are some roads that have carpool lanes whcih can only be used whenever there are atleast 2 passengers in the vehicle. The speed limit on these carpool lanes are higher than regular lanes, hence shorter travel time.

The list of key locations in the city and also the travel time information between the locations for both carpool and regular lanes are provided. Additionally, the location of the passengers looking for a ride are also provided.

The problem to solve is to look for the shortest travel time from a source to a destination while deciding whether to give a ride to the passenger along the way.

## Approach Description

Model the problem as a layered graph such that

- each location represents a vertex
- each road represents the edge connecting the locations
- each edge has 2 weights where one represents the travel time using the regular lane, the other represent the carpool lane
- the locations that contains a passenger would have an edge connecting from the first layer graph to its corresponding vertex in the second layer graph
- Use Dijkstra's Algorithm to look for the shortest travel time on the graph modelled
- For better visualisation of the approach, download "Graph_Before_Pre-Processed.pdf" and "Layered_Graph.pdf", containing a visual representation of the approach

## Instructions

1. Download the "give_a_ride.py" python file
2. Run the file given the inputs
3. Feel free to modify the inputs as you wish

## Contributing

1. Chan Yanhan
2. Monash School of IT FIT1008 & FIT2004 Teaching Team
