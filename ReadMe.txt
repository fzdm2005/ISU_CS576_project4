This file is coded and tested with python3.7.0
'matplotlib' package is needed to plot the result
'dubins-1.0.1' is used to calculate dubins curve 

1. Run rrt.py for task1.
2. Run prm.py for task2. 
3. In image for task2, the grey edge is visited edge during searching. The blue line is the path found from qI to qG.


Comments:

1. For prm, the destination(qG) is selected to check periodicly(every 50 iterations). This operation was not in the algorithm in text book because when we generate the map the destination maybe unknown. Here we do so just for accelerating the running speed. It works well even though without this operation the code can still find the configeration which is close ennough to the target. 