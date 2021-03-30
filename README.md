# ros2bag_tools

This package is built and tested with **ROS2 rolling**, as it depends on `rosbag2_py`, which as of March 2021 has not been released to foxy.

Tool verb extensions to the ros2bag cli.

| Verb    | Usage |
| ------- |:------------------|
| extract | extract topics from a bag and store in new output bag |
| cut     | cut time slice from a bag and store in new output bag |
| reframe | change frame_id on messages with headers |
| rename  | change name of a topic |
| restamp | write header timestamps of all messages with headers as message timestamp in a new output bag |
| replace | replace messages of a specific topic with message data specified in a yaml file, write all messages to new bag |

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

and run `ros2 bag process -c process.config in.bag -o out.bag` to cut off the first 10 seconds, extract messages of the /image_rect topic and restamp them in one go.
