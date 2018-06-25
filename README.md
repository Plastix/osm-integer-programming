# osm-integer-programming
A program for calculating optimal cycling routes modelled as an integer program (known as the *Cycle Trip Planning Problem*). Mapping data is provided by [OpenStreetMaps](https://www.openstreetmap.org/#) and parsed by [GraphHopper](https://github.com/graphhopper/graphhopper). The integer program is solved using [Gurobi](http://www.gurobi.com/).

Mathematical formulation of the Cycle Trip Planning Problem is from the following [paper](https://www.sciencedirect.com/science/article/pii/S1366554514000751):
```
Verbeeck, C., Vansteenwegen, P., & Aghezzaf, E. H. (2014). An Extension of the Arc Orienteering 
Problem and its Application to Cycle Trip Planning. Transportation Research Part E: Logistics 
and Transportation Teview, 68, 64-78.
```

This is part of my senior thesis code: [Plastix/osm-bike-routing](https://github.com/Plastix/osm-bike-routing)
