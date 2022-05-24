# Research dev log for large scale exploration project

## July 26, 2021
**FINISHED**:
1. Working on the mech design for small drone, many problems:
   1. two cnc board for N3/LB2, need to change height, some weird small parts, width for LB2
   2. change the CAD for down board, wrong dim
2. change some parameters to be set in launch file

**TODO**:
1. make mapping module a new thread
2. exploration add revisit to make more loop closure 
3. add a more real loop closure in simulation, i.e. near position

## Aug 2, 2021
**FINISHED**:
1. ROS Multi-thread spinner
   1. fix the multi thread problem in sdf map (inputPointCloud & LoopClosureCallback)

**TODO**:
1. exploration add revisit to make more loop closure 
2. add a more real loop closure in simulation, i.e. near position

## Aug 5, 2021
**FINISHED**:
1. Make the reintegration process more smooth

**TODO**:
1. exploration add revisit to make more loop closure 
   1. add viewpoint to TSP
2. add a more real loop closure in simulation, i.e. near position
   1. creat a new pose graph to check loop in simulation
3. fix the frame problem of loop compensation
4. changed fixed map range (looks hard, need to change the framework)

## Aug 9, 2021
**FINISHED**:
1. fix the frame problem of loop compensation, abandoned in simulation. Will use loop fusion in VINS
2. add a more real loop closure in simulation, i.e. near position

**TODO**:
1. exploration add revisit to make more loop closure 
   1. add viewpoint to TSP
2. changed fixed map range (looks hard, need to change the framework)
3. make set cover more robust (cover one viewpoint multiple times instead of just 1)
4. use a new caster for set cover solver in find covered grids

## Aug 10, 2021
**NOTES**:
1. Function stack for exploration planner:
   1. exec_timer_(0.01s) -> FSMCallback() -> case PLAN_TRAJ: callExplorationPlanner() -> planExploreMotion() -> searchFrontiers, computeFrontiersToVisit, getFrontiers, getFrontierBoxes, getDormantFrontiers, getTopViewpointsInfo, if (ed_->points_.size() > 1) findGlobalTour() -> updateFrontierCostMatrix(), getFullCostMatrix() -> cost matrix size not correct

**BUG**:
1. getFullCostMatrix: frontier size != cost matrix size, e.g. frontier size = 1, cost/path size = 2, (2,2), should be same
2. getPathForTour:
3. TSP solver DIM < 3

**FINISHED**:
1. 

**TODO**:
1. exploration add revisit to make more loop closure 
   1. add viewpoint to TSP
2. changed fixed map range (looks hard, need to change the framework)
3. make set cover more robust (cover one viewpoint multiple times instead of just 1)
4. use a new caster for set cover solver in find covered grids
5. debug the frontier cost problem

## Aug 11, 2021
**FINISHED**:
1. use a new caster for set cover solver in find covered grids
2. debug the frontier cost problem (boyu helped)
3. make set cover more robust (cover one viewpoint multiple times instead of just 1)

**TODO**:
1. Active loop closure: exploration add revisit viewpoint to make more loop closure 
   1. add viewpoint to TSP
2. changed fixed map range (looks hard, need to change the framework)
3. add a points filter? like luqi did on small drone
4. Real vins loop fusion code, may modify the code to make the pose graph part compatible with the mapping module (ddl aug 12)
   1. add a subscriber in sdf_map to subscribe pose graph from loop fusion
   2. compare the pose graph if any changes
      1. may need to change the loop fusion to signal pose change
   3. 

## Aug 14, 2021
**FINISHED**:
1. Real vins loop fusion code, may modify the code to make the pose graph part compatible with the mapping module (finished yesterday)
2. add a points filter? like luqi did on small drone (no need, too much computing)

**TODO**:
1. Active loop closure: exploration add revisit viewpoint to make more loop closure 
   1. add viewpoint to TSP
2. changed fixed map range (looks hard, need to change the framework)
   1. 30m * 30m * 2m map, 0.1 resolution, a tsdf vector with double type cost 115.2MB memory
3. add a points filter? like luqi did on small drone (n)
4. better scp solver? 
5. make near blockes reintegration first


## Aug 17, 2021
**FINISHED**:
1. Active loop closure: exploration add revisit viewpoint to make more loop closure 
   1. not that good, need to delete looped points

**TODO**:
1. changed fixed map range (looks hard, need to change the framework)
   1. 30m * 30m * 2m map, 0.1 resolution, a tsdf vector with double type cost 115.2MB memory
2. better scp solver? 
3. make near blockes reintegration first


## Bug list
1. kino t too long, Traj: too long, Total time too long!!! (burst to ~10s)
2. corrupted double-linked list, do not know where it is
3. search 1 fail, plan fail, stuck
4. deal loop, do not know where happen, only show "flight time: ", normally after "Set cover solved for blocks", sometimes died directly
5. corrupted size vs. prev_size
6. too slow under large scale simulation
7. set cover bug, must changed to greedy_003
8. planExploreMotion() fast_planner::Astar::reset(), free(): invalid pointer, last msg: Next view: xxx
9. computeCost -> searchPath -> Astar::reset, free(): invalid pointer
10. isFrontierCovered() -> Frontier::~Frontier(), munmap_chunk(): invalid pointer


## Improvement list
1. speed up integration (target: avg. 0.005s per frame)
   1. speed in FUEL: "Fusion t: cur: 0.006782, avg: 0.005823, max: 0.012428"
2. TSDF weight not only constant weight



1. full sdf raycast 
Integration t: cur: 0.016939, avg: 0.016931, max: 0.025439
Integration t: cur: 0.018085, avg: 0.016933, max: 0.024612 (without inline)
Integration t: cur: 0.013801, avg: 0.014315, max: 0.021502 (sdf = trunc dist if not around surface)
Integration t: cur: 0.011956, avg: 0.010488, max: 0.018078 (sdf = trunc dist if not around surface && eayly terminate)

2. raycast only surface
Integration t: cur: 0.004861, avg: 0.005599, max: 0.010264

3. sdf raycast on surface, occu for other
Integration t: cur: 0.011150, avg: 0.010599, max: 0.016844

4. only raycast
Integration t: cur: 0.004827, avg: 0.006531, max: 0.011139

5. FUEL
Fusion t: cur: 0.005996, avg: 0.007846, max: 0.013549

## Sep 1, 2021
**TODO**
1. change update bbox for frontiers and esdf
2. robustness test, sometime the planner will stuck
3. benchmark comparison

## Exp:
Ablation:
1. voxblox
2. voxgraph / clox
3. reintegration (brute-force)
4. reintegration with set cover

Drone test:
1. original FUEL
2. all fetures on
