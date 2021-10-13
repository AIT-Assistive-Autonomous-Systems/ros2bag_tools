# ROS2 bag tools

This package is built and tested with **ROS2 galactic**.

This package adds verb extensions to the ros2bag cli.

| Verb    | Usage |
| ------- |:------------------|
| cut     | cut time slice by wall time or duration offset |
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
reader = SequentialReader()
storage_options = StorageOptions(uri='test/range.bag', storage_id='sqlite3')
converter_options = ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr')
reader.open(storage_options, converter_options)
for _, range_msg, _ in BagView(reader):
    print('r', range_msg.range)
```

Or read topics as pandas data frames.

```python
from rosbag2_tools.data_frame import read_data_frames
reader = SequentialReader()
storage_options = StorageOptions(uri='test/range.bag', storage_id='sqlite3')
converter_options = ConverterOptions(
    input_serialization_format='cdr',
    output_serialization_format='cdr')
reader.open(storage_options, converter_options)
dfs = read_data_frames(BagView(reader))
dfs['/range'].summary()
```

## Current limitations

* Tools do not operate in-place, they all create new output bags, potentially doubling the required disk space
* The time filters used in the `cut` verb truncate timestamps to the microsecond, due to the precision loss of the pybind11-conversion of C++ chrono time objects to python3 datetime objects. Thus, filters are not sufficiently precise to handle timestamp deltas below 1000ns.
