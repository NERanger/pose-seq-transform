#pragma once

#include <string>
#include <memory>
#include <vector>
#include <fstream>

#include <Eigen/Geometry>

#include "csv.h"

namespace pose_seq_trans{

struct PoseData{
    size_t seq;
    double timestamp;  // Sec
    double position_x;
    double position_y;
    double position_z;
    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
};

class PoseSeqTransFormer{
public:
    using RawDataVecType = std::vector<PoseData>;
    using RawDataVecPtr = std::shared_ptr<RawDataVecType>;
    using PoseVecType = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;
    using PosesVecPtr = std::shared_ptr<PoseVecType>;

    PoseSeqTransFormer(const std::string &csv_path);

    PosesVecPtr GetIsometry3dPoseVec() const;
    RawDataVecPtr GetRawPoseVec() const;

    static PosesVecPtr TransformRelativeTo(const PosesVecPtr poses, size_t rel);

    static void OutPutKittiFormat(const PosesVecPtr poses_ptr, std::ofstream &ofstream);
    static void OutPutTumFormat(const RawDataVecPtr poses_ptr, std::ofstream &ofstream);

private:
    void LoadRawPoseData();

    std::shared_ptr<io::CSVReader<9>> reader_ptr_ = nullptr;

    RawDataVecPtr raw_data_;

    // Expected header for input csv file
    std::string header_seq_str_ = std::string("seq");

    std::string header_stamp_str_ = std::string("timestamp");
    
    std::string header_x_str_ = std::string("position_x");
    std::string header_y_str_ = std::string("position_y");
    std::string header_z_str_ = std::string("position_z");

    std::string header_quat_x_str_ = std::string("quat_x");
    std::string header_quat_y_str_ = std::string("quat_y");
    std::string header_quat_z_str_ = std::string("quat_z");
    std::string header_quat_w_str_ = std::string("quat_w");
};

}