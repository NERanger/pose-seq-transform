#pragma once

#include <string>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "csv.h"

namespace pose_seq_trans{

class PoseSeqTransFormer{
public:
    using PoseVecType = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;
    using PosesVecPtr = std::shared_ptr<PoseVecType>;

    PoseSeqTransFormer(const std::string &csv_path);

    PosesVecPtr LoadPoseInMemory() const;

    static PosesVecPtr TransformRelativeTo(const PosesVecPtr poses, size_t rel);

private:
    std::shared_ptr<io::CSVReader<8>> reader_ptr_ = nullptr;

    // Expected header for input csv file
    std::string header_seq_str_ = std::string("seq");
    
    std::string header_x_str_ = std::string("position_x");
    std::string header_y_str_ = std::string("position_y");
    std::string header_z_str_ = std::string("position_z");

    std::string header_quat_x_str_ = std::string("quat_x");
    std::string header_quat_y_str_ = std::string("quat_y");
    std::string header_quat_z_str_ = std::string("quat_z");
    std::string header_quat_w_str_ = std::string("quat_w");
};

void CsvWritePose(std::ofstream &ostream, const PoseData &data){
    static bool header_written = false;

    if(header_written == false){
        ostream << "seq, position_x, position_y, position_z, quat_x, quat_y, quat_z, quat_w" << std::endl;
        header_written = true;
    }

    ostream << data.seq << "," << data.position_x << "," << data.position_y << "," << data.position_z 
                        << "," << data.quat_x << "," << data.quat_y << "," << data.quat_z << "," << data.quat_w << std::endl;
}

}