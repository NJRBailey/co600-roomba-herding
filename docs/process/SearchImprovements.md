# Search Algorithm Improvements

Although both the search patterns are successful in finding the Roomba in a reasonable time, there are a still a few ways the search algorithm could be improved.

1. One way to improve the lawnmower pattern is to be able to return its coordinates once it has found the Roomba. This means that the enhanced guide solution can be utilised with this pattern as it uses the current coordinates of the Roomba.

2. The lawnmower path can use a last known coordinate of the Roomba to further improve the speed of search. This can change the lawnmower pattern so that it doesn't have to reach the other side of the arena, but only a few units above and below the last known coordinate. It may decrease the time it takes to find the Roomba this way.

3. To improve the spiral-like pattern further, GPS could be used instead of estimating where the robot is located. A GPS  tracker will be able to return accurate coordinates, and will also help map out the area which can in turn make the search pattern more efficient.

4. It would be ideal for the search algorithm to recognise where the pen is located before the search. This means the pen can then be located anywhere in the arena.

5. An improvement for all search algorithms could be to add a functionality where, if it sees a part of a marker but not the whole marker, it can backtrack or move towards the direction of the object to confirm whether it is the Roomba or not.

6. A more complex search pattern may be able to find the Roomba faster.
