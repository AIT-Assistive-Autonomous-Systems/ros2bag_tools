# ROS2 bag tools

This package is built and tested with **ROS2 galactic**.

This package adds verb extensions to the ros2bag cli.

| Verb    | Usage |
| ------- |:------------------|
| cut     | cut time slice by wall time or duration offset |
| drop    | drop X out of every Y messages of a topic |
| export  | export data to other formats |
| extract | extract topics by name |
| merge   | merge multiple bags into one |
| plot    | plot message data to a new window |
| reframe | change frame_id on messages with headers |
| rename  | change name of a topic |
| restamp | for all messages with headers, change the bag timestamp to their header stamp |
| replace | replace messages of a specific topic with message data specified in a yaml file |
| summary | print summary on data to stdout |
| video | show or write video of image data |

You can check detailed usage information with `ros2 bag $VERB --help`.

Each command writes a new output bag on disk.
If you need to chain multiple commands together, you can use `ros2 bag process` to process all messages in-memory and write only one output bag.
To use it, create a config file that contains one line per command, with the same arguments as the cli interface.

For example create a file named `process.config` containing those lines:

```
cut --duration 10
extract -t /image_rect
restamp
```

and run `ros2 bag process -c process.config in.bag -o out.bag` to crop to the first 10 seconds, keep only the /image_rect topic and restamp the remaining messages in one go.

## Python API

The verbs are built on top of utilities in `rosbag2_tools`.

You can iterate message data using BagView.

```python
from rosbag2_tools.bag_view import BagView
for _, range_msg, _ in BagView('rosbag2_tools/test/range.bag'):
    print(range_msg.range)
# =>
# 10.0
# 20.0
```

Or read topics as pandas data frames.

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
