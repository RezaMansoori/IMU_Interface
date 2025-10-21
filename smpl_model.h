#ifndef SMPL_MODEL_H
#define SMPL_MODEL_H

#undef slots
#undef signals
#undef emit
#include <torch/torch.h>
#include <torch/script.h>
#define slots Q_SLOTS
#define signals Q_SIGNALS
#define emit Q_EMIT

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <memory>

struct CausalAvgImpl : public torch::nn::Module {
    int kernel_size;
    int pad;
    torch::nn::AvgPool2d conv{nullptr};

    CausalAvgImpl(int k);

    std::pair<torch::Tensor, torch::Tensor> forward(const torch::Tensor& x,
                                                    const c10::optional<torch::Tensor>& state = c10::nullopt);
};
TORCH_MODULE(CausalAvg);

class SMPLModel
{
public:
    explicit SMPLModel(const std::string &npz_path, const std::string &fk_model_path);

    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
    view_motion(const std::vector<torch::Tensor>& p_pose_list, const std::vector<torch::Tensor>& p_trans_list);
private:
    void loadParameters(const std::string &npz_path);
    // std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
    // SMPLModel::view_mesh(
    //     const std::vector<torch::Tensor>& vertex_list,
    //     std::vector<torch::Tensor>& joints_list,
    //     float distance_between_subjects);
    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> view_mesh(
        const std::vector<torch::Tensor>& vertex_list,
        std::vector<torch::Tensor>& joints_list,
        double distance_between_subjects = 0.8
        );

    torch::jit::script::Module fk_model;

    torch::Tensor v_template;     // [6890, 3]
    torch::Tensor weights;        // [6890, 24]
    torch::Tensor faces;          // [13776, 3]
};

#endif // SMPL_MODEL_H
