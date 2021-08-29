# Pose Sequence Transform

Given a sequence of 6D poses, this repo provides a class which can transform all poses in sequence into the coordinate of a specific pose in the sequence.

The pose sequence is expected to be stored in a csv file.

## Usage

```cpp
using pose_seq_trans::PoseSeqTransFormer;

// Construct instance with path to csv file
std::string csv_path(argv[1]);
PoseSeqTransFormer transformer(csv_path);

// Load all poses into memory
PoseSeqTransFormer::PosesVecPtr loaded_ptr = transformer.LoadPoseInMemory();

// Perform transform    
PoseSeqTransFormer::PosesVecPtr transed_ptr = PoseSeqTransFormer::TransformRelativeTo(loaded_ptr, 0);
```