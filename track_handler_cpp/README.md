# Track Handler
- [Track Handler](#track-handler)
  - [Provided Functionality](#provided-functionality)
    - [Classes](#classes)
    - [Functions](#functions)
  - [Usage](#usage)
    - [Add track\_handler\_cpp to your Package](#add-track_handler_cpp-to-your-package)
    - [In your C++ code](#in-your-c-code)
    - [PyBinding](#pybinding)
  - [Config](#config)
    - [Changing Configuration](#changing-configuration)
    - [Overwrites](#overwrites)
  - [Coordinate Convention](#coordinate-convention)
  - [Example](#example)


## Provided Functionality
### Classes
The package provides the following classes:
- [RaceTrackHandler](include/track_handler_cpp/race_track_handler.hpp): Loads the configuration of a Racetrack (e.g. Monza) from config-Files (and overwrites). Provides you with the desired track and raceline and additional Information of the track e.g.: 
    - geodetic_origin
    - sim_start_pos
    - initial_heading

- [Track](include/track_handler_cpp/track.hpp): Provides access to track data and various functions to calculate accelerations, rotations, etc. based on the track.
- [Raceline](include/track_handler_cpp/raceline.hpp): Provides access to raceline data

### Functions
If you have a track instance (see above) those functions should not be required. These are a implementation detail of the [Track](include/track_handler_cpp/track.hpp).
- [rotation.hpp](include/track_handler_cpp/rotation.hpp): Provides functions to calc/convert angles on the track
- [acceleration.hpp](include/track_handler_cpp/acceleration.hpp): Provides functions to calc/convert accelerations on the track

## Usage
### Add track_handler_cpp to your Package
CMakeLists.txt
```CMAKE
find_package(track_handler_cpp REQUIRED)
ament_target_dependencies(${YOUR_TARKGET} PUBLIC track_handler_cpp)
```

package.xml
```xml
<depend>track_handler_cpp</depend>
```

### In your C++ code

The recommended way to get a track is via a RaceTrackHandler. This will automatically load the right track from configuration/overwrites.
```C++
#include "track_handler_cpp/race_track_handler.hpp"

auto raceTrackHandler = tam::common::RaceTrackHandler::from_pkg_config();
```
This RaceTrackHandler provides you with additional information on the Track:

```C++
raceTrackHandler->get_geo_origin();
raceTrackHandler->get_sim_start_pos();
raceTrackHandler->get_initial_heading();
raceTrackHandler->get_track_name();
...
```
To get a track use the following method of the `RaceTrackHandler`, and store the returned `Track` in your code.
```C++
std::unique_ptr<tam::common::Track> track = raceTrackHandler->create_track();
std::unique_ptr<tam::common::Track> pit = raceTrackHandler->create_pitlane();
```
To get a raceline use the following method of the `RaceTrackHandler`, and store the returned `Raceline` in your code.
```C++
std::unique_ptr<tam::common::Raceline> track = raceTrackHandler->create_raceline();
```

### PyBinding

To use the pybind of the track_handler:

In the pathon script:
```python
from track_handler_py import RaceTrackHandler, Track, Raceline
```


## Config

Constructing the Track/Raceline via 
```C++
auto raceTrackHandler = tam::common::RaceTrackHandler::from_pkg_config();
```
handles the parameters and configuration for you.

### Changing Configuration
Default parameters for different Tracks are stored in [config/](config/) and should be provided in the shown format. All Parameters/Files provided in [config_overwrite/](config_overwrite/) will overwrite the default parameters.

You can specify the following Parameters:
- Racetrack ([config.yml](config/config.yml)): Allows to specify the racetrack (e.g. Monza)
- Track Config (e.g. [Monza/config.yml](config/Monza/config.yml)): Allows to specify individual parameters for racetracks
- Raceline Files (e.g. [Monza/raceline/](config/Monza/raceline/)): Dump Raceline-Files here (Raceline files include all required track information)

### Overwrites
To overwrite the track configuration inside the container (e.g. from tam_launch), mount a local folder to the [config_overwrite/](config_overwrite/) folder inside the container:
```yaml
YOUR_MODULE:
  image: gitlab.lrz.de:5005/tam/<YOUR_MODULE>:<YOUR_TAG>
  network_mode: "host"
volumes:
    - "./<LOCAL_FOLDER>:/dev_ws/install/track_handler_cpp/share/track_handler_cpp/config_overwrite
```

For **tam_launch**:
```yaml
YOUR_MODULE:
  image: gitlab.lrz.de:5005/tam/<YOUR_MODULE>:<YOUR_TAG>
  network_mode: "host"
volumes:
    - "./config/track_handler:/dev_ws/install/track_handler_cpp/share/track_handler_cpp/config_overwrite
```
## Coordinate Convention
The provided track data follows the ENU (East, North, Up) convention for cartesian coordinate systems. And is a right-handed coordinate system.
- x-axis: East
- y-axis: North
- z-axis: Up
- heading=0 means driving east. The heading increases counter clockwise.

## Example

For an Example on how to plot the whole track/raceline data see: [test_plot.cpp](test/test_plot.cpp)

To run [test_plot.cpp](test/test_plot.cpp):
```sh
colcon build --packages-up-to track_handler_cpp && . install/setup.bash && ros2 run track_handler_cpp test_plot
```