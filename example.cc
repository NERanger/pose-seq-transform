#include <iostream>
#include <string>

#include "PoseSeqTransformer.hpp"

using pose_seq_trans::PoseSeqTransFormer;

int main(int argc, char const *argv[]){
    if(argc != 2){
        std::cerr << "Usage: ./example <path-to-csv>" << std::endl;
        return EXIT_FAILURE;
    }

    std::string csv_path(argv[1]);
    PoseSeqTransFormer transformer(csv_path);

    PoseSeqTransFormer::PosesVecPtr poses_ptr = transformer.GetIsometry3dPoseVec();
    PoseSeqTransFormer::PosesVecPtr transed_ptr = PoseSeqTransFormer::TransformRelativeTo(poses_ptr, 0);

    // for(size_t i = 0; i < transed_ptr->size(); ++i){
    //     Eigen::Isometry3d iso = transed_ptr->at(i);
    //     std::cout << iso.matrix() << std::endl;
    // }

    std::ofstream out("transed.txt");
    PoseSeqTransFormer::OutPutKittiFormat(transed_ptr, out);
    out.close();

    return EXIT_SUCCESS;
}
