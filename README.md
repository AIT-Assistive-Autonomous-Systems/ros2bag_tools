# ros2bag_tools

Tool verb extensions to the ros2bag cli.

| Verb    | Usage |
| ------- |:------------------|
| extract | extract topics from a bag and store in new output bag |
| cut     | cut time slice from a bag and store in new output bag |
| restamp | write header timestamps of all messages with headers as message timestamp in a new output bag |
| replace_uniform | replace messages of a specific topic with message data specified in a yaml file, write all messages to new bag |


You can check detailed usage information with `ros2 bag $VERB --help`.