# Catch Me If You Can

# Overview

This project presents using two different type of path finding algorithm to intercept the target: **A-start** algorithm
and **RRT** algorithm.

# Mazes

In this project, there are a myriad of maps, ranging from small maps (6x8) to extremely large maps (5000*50000) [1].
Here are some of the mazes that we use:

<picture>
  <img src="https://github.com/neilchen1998/catch-me-if-you-can/blob/main/maps/map-0.png" width="300" height="250">
</picture>

- Map 0 (small map)

# Approaches

In this project, we use **A-start** algorithm
and **RRT** algorithm to intercept the target.

For the **A-start** algorithm, we use Manhattan distance as
our heuristic function.

For the **RRT** algorithm, we use two slightly different values for $\epsilon$ (the distance between the current point and the new random point). For **RRT1**, the value of $\epsilon$ is 1.4, whereas that of **RRT2** is porportional to the size of the map, i.e., the larger the size of the map, the larger the value of $\epsilon$ is.

# Results

<picture>
  <img src="https://github.com/neilchen1998/catch-me-if-you-can/blob/main/results-gif/A/map-0-A.gif" width="300" height="250">
</picture>

- Intercepting the target using A-star in Map 0

<picture>
  <img src="https://github.com/neilchen1998/catch-me-if-you-can/blob/main/results-gif/RRT2/map-6-RRT-2.gif" width="300" height="250">
</picture>

- Intercepting the target using RRT2 in Map 6

<table border="2" summary="">
<tr>
    <th>Algorithm</th>
    <th>Map</th>
    <th>Steps</th>
    <th>Avg. Time</th>
  </tr>
 <tr>
  <td rowspan="11">A-star</td>
  <td>Map 0</td>
  <td>8</td>
  <td>5.9*E-3</td>
 </tr>
 <tr>
   <td>Map 1</td>
   <td>1281</td>
   <td>10.5</td>
 </tr>
 <tr>
   <td>Map 2</td>
   <td>16</td>
   <td>4.4*E-2</td>
 </tr>
 <tr>
   <td>Map 3</td>
   <td>257</td>
   <td>0.86</td>
 </tr>
 <tr>
   <td>Map 4</td>
   <td>9</td>
   <td>5.6*E-3</td>
 </tr>
 <tr>
   <td>Map 5</td>
   <td>NaN</td>
   <td>NaN</td>
 </tr>
 <tr>
   <td>Map 6</td>
   <td>76</td>
   <td>0.57</td>
 </tr>
 <tr>
   <td>Map 7</td>
   <td>NaN</td>
   <td>NaN</td>
 </tr>
 <tr>
   <td>Map 1B</td>
   <td>NaN</td>
   <td>NaN</td>
 </tr>
 <tr>
   <td>Map 3B</td>
   <td>NaN</td>
   <td>NaN</td>
 </tr>
 <tr>
   <td>Map 3C</td>
   <td>NaN</td>
   <td>NaN</td>
 </tr>
</table>

<table border="2" summary="">
<tr>
    <th>Algorithm</th>
    <th>Map</th>
    <th>Avg. Steps</th>
    <th>Avg. Time</th>
    <th>Avg. Iterations</th>
  </tr>
  <tr>
    <td rowspan="11">RRT2</td>
    <td>Map 0</td>
    <td>7</td>
    <td>4.6*E-3</td>
    <td>34.3</td>
   </tr>
   <tr>
     <td>Map 1</td>
     <td>1419</td>
     <td>1.3</td>
     <td>834.3</td>
   </tr>
   <tr>
     <td>Map 2</td>
     <td>15.3</td>
     <td>2.9*E-2</td>
     <td>427.7</td>
   </tr>
   <tr>
     <td>Map 3</td>
     <td>403</td>
     <td>2.8</td>
     <td>667.7</td>
   </tr>
   <tr>
     <td>Map 4</td>
     <td>9</td>
     <td>8.3*E-2</td>
     <td>142</td>
   </tr>
   <tr>
     <td>Map 5</td>
     <td>1019.3</td>
     <td>7.7</td>
     <td>6412.7</td>
   </tr>
   <tr>
     <td>Map 6</td>
     <td>16</td>
     <td>4.4*E-2</td>
     <td>2994.3</td>
   </tr>
   <tr>
     <td>Map 7</td>
     <td>NaN</td>
     <td>NaN</td>
     <td>NaN</td>
   </tr>
   <tr>
     <td>Map 1B</td>
     <td>4461.7</td>
     <td>293.1</td>
     <td>51874.3</td>
   </tr>
   <tr>
     <td>Map 3B</td>
     <td>719.3</td>
     <td>49.6</td>
     <td>5194.3</td>
   </tr>
   <tr>
     <td>Map 3C</td>
     <td>1278.6</td>
     <td>29.1</td>
     <td>2941</td>
   </tr>
</table>

# Reference

1. [UCSD ECE276B Planning & Learning in Robotics](https://natanaso.github.io/ece276b/)
