#include <iostream>

#include "PoseSeqTransformer.hpp"

using pose_seq_trans::PoseSeqTransFormer;

PoseSeqTransFormer::PoseSeqTransFormer(const std::string &csv_path){
    reader_ptr_ = std::make_shared<io::CSVReader<8>>(csv_path);
}

PoseSeqTransFormer::PosesVecPtr PoseSeqTransFormer::LoadPoseInMemory() const{
    if(reader_ptr_ == nullptr){
        return nullptr;
    }

    reader_ptr_->read_header(io::ignore_extra_column, header_seq_str_, 
                             header_x_str_, header_y_str_, header_z_str_, 
                             header_quat_x_str_, header_quat_y_str_, header_quat_z_str_, header_quat_w_str_);

    int seq;
    double position_x, position_y, position_z;
    double quat_x, quat_y, quat_z, quat_w;

    PosesVecPtr poses_ptr = std::make_shared<PoseVecType>();
    while(reader_ptr_->read_row(seq, position_x, position_y, position_z, quat_x, quat_y, quat_z, quat_w)){
        Eigen::Vector3d trans(position_x, position_y, position_z);
        Eigen::Quaterniond quat(quat_w, quat_x, quat_y, quat_z);

        Eigen::Isometry3d iso(Eigen::Isometry3d::Identity());
        iso.rotate(quat);
        iso.pretranslate(trans);

        poses_ptr->emplace_back(iso);
    }

    return poses_ptr;

}

PoseSeqTransFormer::PosesVecPtr PoseSeqTransFormer::TransformRelativeTo(const PoseSeqTransFormer::PosesVecPtr poses_ptr, size_t rel){
    PosesVecPtr transed_poses = std::make_shared<PoseVecType>();

    for(size_t i = 0; i < poses_ptr->size(); ++i){
        Eigen::Isometry3d transed = poses_ptr->at(rel).inverse() * poses_ptr->at(i);
        transed_poses->emplace_back(transed);
    }

    return transed_poses;
}