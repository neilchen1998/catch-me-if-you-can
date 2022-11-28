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

# Results

<picture>
  <img src="https://github.com/neilchen1998/escape-the-maze/blob/main/result-example-gif.gif" width="300" height="250">
</picture>

- One of the results

| Map | Steps | Avg. Time |
| ------------- | ------------- | -------- |
| Map 0  | 8  | 5.9*10^-3 |
| Map 1  | 1281  | 10.5 |
| Map 2  | 16  | 4.4*10^-2 |
| Map 3  | 257  | 0.86 |
| Map 4  | 9  | 5.6*10^-3 |
| Map 5  | NaN  | NaN |
| Map 6  | 76  | 0.57 |
| Map 7  | NaN  | NaN |
| Map 1B  | NaN  | NaN |
| Map 3B  | NaN  | NaN |
| Map 3C  | NaN  | NaN |

# Reference

1. [UCSD ECE276B Planning & Learning in Robotics](https://natanaso.github.io/ece276b/)
