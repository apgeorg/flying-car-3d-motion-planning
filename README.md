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
    """
    Reads home (lat, lon) from the first line of the `file`.
    """
    with open(file) as f:
        lat, lon = f.readline().split(',')
        lat, lon = float(lat.split(' ')[-1]), float(lon.split(' ')[-1])
    return lat, lon
```  

And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

### Execute the flight
#### 1. Does it work?
It works!

