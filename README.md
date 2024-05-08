# Pipeline-Status

| Workflow | Branch | Status |
| -------  | ------ | ------ |
| Build and test | | [![rosci](https://github.com/AIT-Assistive-Autonomous-Systems/ros2bag_tools/actions/workflows/rosci.yaml/badge.svg?branch=master)](https://github.com/AIT-Assistive-Autonomous-Systems/ros2bag_tools/actions/workflows/rosci.yaml) |
| Lint | | [![roslint](https://github.com/AIT-Assistive-Autonomous-Systems/ros2bag_tools/actions/workflows/roslint.yaml/badge.svg?branch=master)](https://github.com/AIT-Assistive-Autonomous-Systems/ros2bag_tools/actions/workflows/roslint.yaml) |

# ROS2 bag tools

ros2bag_tools adds verb extensions to the ros2bag cli.

| Verb    | Usage |
| ------- |:------------------|
| **add** | add new topic, with messages aligned to existing topic |
| **cut** | cut time slice by wall time or duration offset |
| **drop** | drop X out of every Y messages of a topic |
| export  | export data to other formats, see [export](#export) |
| **extract** | extract topics by name |
| plot    | plot message data to a new window, see [plot](#plot) |
| process | chain multiple filters, see [chaining](#chaining) |
| **prune** | remove topics without messages |
| **reframe** | change frame_id on messages with headers |
| **rename**  | change name of a topic |
| **replace** | replace messages of a specific topic with message data specified in a yaml file |
| **restamp** | for all messages with headers, change the bag timestamp to their header stamp |
| summary | print summary on data to stdout |
| **sync** | output synchronized bundles of messages using the ApproximateTimeSynchronizer |
| video | show or write video of image data |

You can check detailed usage information with `ros2 bag $VERB --help`.
Bold verbs support chaining as described in [chaining](#chaining).

## chaining

Each command writes a new output bag on disk.
If you need to chain multiple commands together, you can use `ros2 bag process` to process all messages in-memory and write only one output bag.
To use it, create a config file that contains one line per command, with the same arguments as the cli interface.

For example create a file named `process.config` containing those lines:

```
cut --duration 10
extract -t /image_rect
restamp
```

and run `ros2 bag process -c process.config in.bag -o out.bag` to use only the first 10 seconds, keep only the /image_rect topic and restamp the remaining messages in one go.

## export

Exporting is currently possible for:

* `pcd`: `sensor_msgs/msg/PointCloud2` -> ASCII PCD files
* `image`: `sensor_msgs/msg/[Compressed]Image` -> jpg or png
* `stamp`: `std_msgs/msg/Header` (any message with header) -> stamp file
* `tum_trajectory`: `nav_msgs/msg/Odometry` -> [TUM trajectory file](https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#ground-truth_trajectories)

Run `ros2 bag export --in $BAG_PATH -t $TOPIC_NAME $EXPORTER` to see available options.

### synchronized export

`ros2 bag export` supports preprocessing of messages before exporting, using the filters described above.
In addition, a list of exporters can be passed, to export messages from multiple topics simultaneously while reading the data only once.

For instance, this can be used to export synchronized images in the presence of frame drops:

```
# filters.config
# same syntax as described in chaining
sync -t /left/image_rect /right/image_rect
```

```
# export.config
# syntax: TOPIC EXPORTER_NAME EXPORTER_ARGS..
/left/image_rect image --dir cam_left
/right/image_rect image --dir cam_right
```

```bash
# write only those images in /left/image_rect and /right/image_rect with header.stamp synchronized
# by the default tolerance to the cam_left and cam_right directories respectively
ros2 bag export -f filters.config -c export.config
```

## plot

Quickly plot timeseries from message data using matplotlib.
You can specify one topic + field per time series.

Example:

```bash
# plot header stamps on x axis, range value on y
ros2 bag plot rosbag2_tools/test/range.bag -t /range.range
```

## rosbag2_tools

This packages provides python utilities to implement the verbs, but can be used independently.

Iterate message data using BagView.

```python
from rosbag2_tools.bag_view import BagView
for _, range_msg, _ in BagView('rosbag2_tools/test/range.bag'):
    print(range_msg.range)
# =>
# 10.0
# 20.0
```

Read topics as pandas data frames.

```python
from rosbag2_tools.bag_view import BagView
from rosbag2_tools.data_frame import read_data_frames
bag_view = BagView('rosbag2_tools/test/range.bag')
dfs = read_data_frames(bag_view, {'/range': ['range']})
dfs['/range']
# =>
#    range                  header.stamp
# 0   10.0 1970-01-01 00:00:00.000000090
# 1   20.0 1970-01-01 00:00:00.000000190
```

## Current limitations

* Tools do not operate in-place, they all create new output bags, potentially doubling the required disk space
* The time filters used in the `cut` verb truncate timestamps to the microsecond, due to the precision loss of the pybind11-conversion of C++ chrono time objects to python3 datetime objects. Thus, filters are not sufficiently precise to handle timestamp deltas below 1000ns.
* Export of images: `cv.imencode` is being used to recompress the image. So the same input format implications apply. This also means implies that color images (RGB) when not input as BGR you will experience a color swap
in the output. Force an output encoding if this is not desired.
* Export of CompressedImage images: If the format is encoded like in image_transport_plugins or as cv_bridge encodes it and the desired output format is the same the image will not be recompressed when `passthrough` is selected as the desired output encoding or the stored encoding matches, in this case the data content is just written to the output file. But decoding is done using `cv_bridge` if necessary thus the same restrictions apply.
