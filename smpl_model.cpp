#include "smpl_model.h"
#include "cnpy.h"
#include <iostream>
#include <QMessageBox>

using namespace torch::indexing;

CausalAvgImpl::CausalAvgImpl(int k) : kernel_size(k), pad(k - 1) {
    conv = torch::nn::AvgPool2d(torch::nn::AvgPool2dOptions({kernel_size, 1}).stride(1).padding(0));
    register_module("conv", conv);
}

std::pair<torch::Tensor, torch::Tensor> CausalAvgImpl::forward(const torch::Tensor& x,
                                                               const c10::optional<torch::Tensor>& state) {
    auto B = x.size(0);
    auto T = x.size(1);
    auto N = x.size(2);
    auto D = x.size(3);

    // torch::Tensor s = (state.has_value() && state->defined())
    //                       ? *state
    //                       : torch::zeros({B, pad, N, D}, x.options());
    torch::Tensor s = (state && state->defined()) ? state.value()
                                                  : torch::zeros({B, pad, N, D}, x.options());

    auto y = torch::cat({s, x}, 1);

    auto pooled = conv->forward(y.permute({0, 3, 1, 2})).permute({0, 2, 3, 1});

    auto new_state = y.slice(1, -pad);
    return {pooled, new_state};
}

SMPLModel::SMPLModel(const std::string &npz_path, const std::string &fk_model_path)
{
    // Load parameters
    loadParameters(npz_path);

    // Load forward kinematics model
    try {
        fk_model = torch::jit::load(fk_model_path);
        fk_model.eval();
        std::cout << "[SMPLModel] Loaded FK model.\n";
    } catch (const c10::Error &e) {
        std::cerr << "[SMPLModel] Failed to load FK model: " << e.what() << "\n";
    }
}


void SMPLModel::loadParameters(const std::string &npz_path)
{
    auto npz = cnpy::npz_load(npz_path);

    auto vt = npz["v_template"];
    v_template = torch::from_blob(vt.data<float>(), {6890, 3}).clone();

    auto w = npz["weights"];
    weights = torch::from_blob(w.data<float>(), {6890, 24}).clone();

    auto f = npz["f"]; // faces
    faces = torch::from_blob(f.data<int>(), {13776, 3}, torch::kInt32).clone();
}

std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
SMPLModel::view_motion(const std::vector<torch::Tensor>& p_pose_list, const std::vector<torch::Tensor>& p_trans_list)
{
    std::vector<torch::Tensor> verts_list;
    std::vector<torch::Tensor> joints_list;
    for (size_t i = 0; i < p_pose_list.size(); ++i) {
        torch::Tensor pose = p_pose_list[i].view({-1, 24, 3, 3});
        torch::Tensor trans = p_trans_list[i].view({-1, 3});

        if (i > 0) {
            torch::Tensor t = trans.clone();
            t -= trans[0];
            trans = t;
        }

        auto shape = torch::zeros({pose.size(0), 10}, torch::TensorOptions().device(pose.device()));
        auto R_global = torch::zeros({pose.size(0), 24, 3, 3}, torch::TensorOptions().device(pose.device()));

        auto output = fk_model.forward({pose, shape, trans, R_global}).toTuple();

        verts_list.push_back(output->elements()[2].toTensor().clone());
        joints_list.push_back(output->elements()[3].toTensor().clone());
    }
    return view_mesh(verts_list, joints_list, 0.2);
}

std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
SMPLModel::view_mesh(
    const std::vector<torch::Tensor>& vertex_list,
    std::vector<torch::Tensor>& joints_list,
    double distance_between_subjects)
{
    // Ensure the model has the required base tensors.
    if (this->faces.numel() == 0 || this->v_template.numel() == 0) {
        throw std::runtime_error("'face' and '_v_template' tensors are not initialized in SMPLModel.");
    }

    std::vector<torch::Tensor> v_list;
    std::vector<torch::Tensor> f_list;

    torch::Tensor f = this->faces.clone();

    // Loop through each subject provided in the input lists.
    for (size_t i = 0; i < vertex_list.size(); ++i) {
        torch::Tensor v = vertex_list[i].clone().view({-1, this->v_template.size(0), 3});
        joints_list[i] = joints_list[i].clone(); // اضافه کردن clone اینجا

        // If there are multiple subjects (i > 0), displace them along the x-axis to prevent overlap.
        if (i > 0) {
            double offset = distance_between_subjects * static_cast<double>(i);
            // Select the x-coordinates (index 0 of the last dimension) and add the offset.
            // .select(2, 0) returns a view of the tensor, and .add_() modifies it in-place.
            v.select(2, 0).add_(offset);
            joints_list[i].select(2, 0).add_(offset);
        }
        // Add the (potentially displaced) vertices to our list of vertex tensors.
        v_list.push_back(v.clone());
        // Add a copy of the current face index tensor to our list.
        // 'f' currently holds the correct indices for this subject's vertices.
        f_list.push_back(f.clone());
        // Increment the face indices by the number of vertices in the current subject.
        // This ensures that the faces for the *next* subject will correctly point to its
        // own vertices after all vertex tensors are concatenated.
        f.add_(v.size(1)); // v.size(1) is num_vertices
    }

    // Final concatenation (like torch.cat)
    torch::Tensor verts = torch::cat(v_list, /*dim=*/1);
    torch::Tensor joints = torch::cat(joints_list, /*dim=*/1);
    torch::Tensor faces = torch::cat(f_list, /*dim=*/0);

    // Optionally convert to CPU NumPy-equivalent if needed:
    torch::Tensor verts_cpu = verts.cpu().detach();
    torch::Tensor joints_cpu = joints.cpu().detach();
    torch::Tensor faces_cpu = faces.cpu().detach();

    return std::make_tuple(verts, faces, joints);
}
