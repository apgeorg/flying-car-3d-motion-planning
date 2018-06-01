## 3D Motion Planning
![Quad Image](./misc/enroute.png)

---

### Path Planning Algorithm

#### 1. Set your global home position
The code snipped below reads the first line of the colliders.csv, extracts lat0 and lon0 and use them to set the global home position.

```python
# Read lat0, lon0 from colliders into floating point values
lat0, lon0 = read_home('colliders.csv')
# Set home position to (lon0, lat0, 0)
self.set_home_position(lon0, lat0, 0)
```   

Below the definition of the read_home function:

```python
def read_home(file):
    with open(file) as f:
        lat, lon = f.readline().split(',')
        lat, lon = float(lat.split(' ')[-1]), float(lon.split(' ')[-1])
    return lat, lon
```  

And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Retrieve the current global position and convert it to local position.

```python
# Convert to current local position using global_to_local()
current_position= [self._longitude, self._latitude, self._altitude]
self._north, self._east, self._down = global_to_local(current_position, self.global_home)
```

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. 

```python
# Read in obstacle map
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
# Define a grid for a particular altitude and safety margin around obstacles
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
print("North offset: {0}, East offset: {1}".format(north_offset, east_offset))
# Define starting point on the grid
grid_start = (int(np.ceil(-north_offset + self.local_position[0])), int(np.ceil(-east_offset + self.local_position[1])))
```

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

```python
# Set goal as some arbitrary position on the grid
local_target_position = global_to_local(self.global_target_position, self.global_home)
grid_goal = (int(np.ceil(-north_offset + local_target_position[0])), int(np.ceil(-east_offset + local_target_position[1])))
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Add four new actions in for diagonal motion:

```python
SOUTH_WEST =(-1,-1, np.sqrt(2))
NORTH_WEST =(1,-1, np.sqrt(2))
SOUTH_EAST =(-1, 1, np.sqrt(2))
NORTH_EAST =(1, 1, np.sqrt(2)) 
```
Check if actions are valid:

```python
if y - 1 < 0 or x - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
if y - 1 < 0 or x + 1 > n or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
if y + 1 > m or x + 1 > n or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if y + 1 > m or x - 1 < 0 or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
```
Run the A* Algorithms to find a path from start to goal:

```python
#Compute the lowest cost path with a_star
path, _ = a_star(grid, heuristic, grid_start, grid_goal)
```

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. 

```python
# Prune path to minimize number of waypoints
pruned_path = prune_path(path)
```

### Execute the flight

#### Run
python motion_planning.py --target_lat=37.792572 --target_lon=-122.397336 --target_alt=0.

#### Test flight 1
Start location:   Lon: 37.792572, Lat: -122.397336, Alt: 0.0 
Target location:  Lon: 37.793658, Lat: -122.398028, Alt: 0.0
