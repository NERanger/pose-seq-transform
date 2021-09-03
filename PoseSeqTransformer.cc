#include <iostream>

#include "PoseSeqTransformer.hpp"

using pose_seq_trans::PoseSeqTransFormer;

PoseSeqTransFormer::PoseSeqTransFormer(const std::string &csv_path){
    reader_ptr_ = std::make_shared<io::CSVReader<9>>(csv_path);
    LoadRawPoseData();
}

void PoseSeqTransFormer::LoadRawPoseData(){
    reader_ptr_->read_header(io::ignore_extra_column, header_stamp_str_, header_seq_str_, 
                             header_x_str_, header_y_str_, header_z_str_, 
                             header_quat_x_str_, header_quat_y_str_, header_quat_z_str_, header_quat_w_str_);

    size_t seq;
    double timestamp;
    double position_x, position_y, position_z;
    double quat_x, quat_y, quat_z, quat_w;

    raw_data_ = std::make_shared<RawDataVecType>();
    while(reader_ptr_->read_row(seq, timestamp, position_x, position_y, position_z, quat_x, quat_y, quat_z, quat_w)){
        raw_data_->emplace_back(PoseData{seq, timestamp, position_x, position_y, position_z, quat_x, quat_y, quat_z, quat_w});
    }

}

PoseSeqTransFormer::PosesVecPtr PoseSeqTransFormer::GetIsometry3dPoseVec() const{
    PosesVecPtr poses_ptr = std::make_shared<PoseVecType>();
    for(auto data : *raw_data_){
        Eigen::Vector3d trans(data.position_x, data.position_y, data.position_z);
        Eigen::Quaterniond quat(data.quat_w, data.quat_x, data.quat_y, data.quat_z);

        Eigen::Isometry3d iso(Eigen::Isometry3d::Identity());
        iso.rotate(quat);
        iso.pretranslate(trans);

        poses_ptr->emplace_back(iso);
    }

    return poses_ptr;
}

PoseSeqTransFormer::RawDataVecPtr PoseSeqTransFormer::GetRawPoseVec() const{
    return raw_data_;
}

PoseSeqTransFormer::PosesVecPtr PoseSeqTransFormer::TransformRelativeTo(const PoseSeqTransFormer::PosesVecPtr poses_ptr, size_t rel){
    PosesVecPtr transed_poses = std::make_shared<PoseVecType>();

    for(size_t i = 0; i < poses_ptr->size(); ++i){
        Eigen::Isometry3d transed = poses_ptr->at(rel).inverse() * poses_ptr->at(i);
        transed_poses->emplace_back(transed);
    }

    return transed_poses;
}

void PoseSeqTransFormer::OutPutKittiFormat(const PosesVecPtr poses_ptr, std::ofstream &ofstream){
    for(auto p : *poses_ptr){
        ofstream << p(0, 0) << " " << p(0, 1) << " " << p(0, 2) << " " << p(0, 3) << " "
                 << p(1, 0) << " " << p(1, 1) << " " << p(1, 2) << " " << p(1, 3) << " "
                 << p(2, 0) << " " << p(2, 1) << " " << p(2, 2) << " " << p(2, 3) << std::endl;
    }   
}

void PoseSeqTransFormer::OutPutTumFormat(const RawDataVecPtr poses_ptr, std::ofstream &ofstream){
    for(auto p : *poses_ptr){
        ofstream << p.timestamp << " " << p.position_x << " " << p.position_y << " " << p.position_z << " "
                 << p.quat_x << " " << p.quat_y << " " << p.quat_z << " " << p.quat_w << std::endl;
    } 
}