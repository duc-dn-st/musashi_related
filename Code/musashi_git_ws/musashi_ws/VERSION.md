## Version tracking
# VER1.0
---
1. Change mapping execution file:
- Add Executetion for mapping : start_mapping.sh
- Add Executetion for save map : start_creating_map.sh
- Detail :
    + ( Update ) Only need to run ``` bash start_mapping.sh ``` for mapping
    + ( Update ) start_creating_map.sh : Change route
---
2. Add standby mode 
- Add Executetion : start_standby_mode.sh
- Detail : 
    + Standby mode : 
        + Turn on device (LiDAR)
        + Connect to PLC.
        + ( NEW ) Wait for trajectory to be published.
        + ( NEW ) Only use newest trajectory and only show after getting trajectory.
        + ( Update ) Start map and load map.
        
---
3. Add route selecting mode 
- Add Executetion : start_route_selecting_mode.sh
- Detail : 
    + Route selecting mode : 
        + ( NEW ) Add selecting route points.
        + ( Update ) Trajectory will be generated based new route points.
        + ( NEW ) Add publish trajectory to stand by mode after finish selecting route.
        + ( NEW ) Can re-select new route
        
4. Change controller mode 
- Add Executetion : start_controller_mode.sh
- Detail : 
    + Route selecting mode : 
        + ( NEW ) At start, robot will rotate if the orientation of robot doesnt match with trajectory.
        + ( NEW ) After finish running route, the node will shutdown.