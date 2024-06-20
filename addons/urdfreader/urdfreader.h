#ifndef RBDL_URDFREADER_H
#define RBDL_URDFREADER_H

#include <rbdl/rbdl_config.h>
#include <rbdl/rbdl_math.h>

namespace RigidBodyDynamics {

struct Model;

namespace Addons {
  /**
   * This function will load a URDF model from a file.
   *
   * @param filename: path to the URDF file
   * @param model: reference to the (loaded) multibody model
   * @param floating_pase: does the model use a floating base
   * @param verbose: information will be printed to the command window if this
   *                 is set to true
   */
    int initialize_root(Model &rbdl_model, ModelDatad &model_data, ConstLinkPtr urdf_link,
                        const FloatingBaseType floating_base_type)


    int initialize_link(Model &rbdl_model, ModelDatad &model_data, ConstLinkPtr urdf_link,
                        const int parent_id)


    bool construct_model(Model &rbdl_model, ModelDatad &model_data, ConstLinkPtr urdf_link, int parent_id)

    bool construct_model_cuttips(Model &rbdl_model, ModelDatad &model_data, ConstLinkPtr urdf_link,
                                const int parent_id, const std::vector<std::string> &tipLinks)


    // ============================================================
    // from URDF
    // ============================================================

    // basic version with explicit root
    bool URDFReadFromURDF(Model *model, ModelDatad &model_data,
                        urdf::LinkConstSharedPtr root,
                        const FloatingBaseType floating_base_type, const bool verbose)
        // basic version (default root)
    bool URDFReadFromURDF(urdf::Model &urdf_model, Model *model, ModelDatad &model_data,
                        const FloatingBaseType floating_base_type, const bool verbose)


    // cut tips
    bool URDFReadFromURDF(urdf::Model &urdf_model, Model *model, ModelDatad &model_data,
                        const std::vector<std::string> &tip_links,
                        const FloatingBaseType floating_base_type, const bool verbose)

    // basic version with extra info and explicit root
    bool URDFReadFromURDF(urdf::Model &urdf_model, Model *model, ModelDatad &model_data,
                        urdf::LinkConstSharedPtr root,
                        const FloatingBaseType floating_base_type,
                        std::vector<std::string> &joint_names,
                        std::vector<double> &position_min, std::vector<double> &position_max,
                        std::vector<double> &vel_min, std::vector<double> &vel_max,
                        std::vector<double> &damping, std::vector<double> &friction,
                        std::vector<double> &max_effort, const bool verbose)
    
    // basic version with extra info and default root
    bool URDFReadFromURDF(urdf::Model &urdf_model, Model *model, ModelDatad &model_data,
                        const FloatingBaseType floating_base_type,
                        std::vector<std::string> &joint_names,
                        std::vector<double> &position_min, std::vector<double> &position_max,
                        std::vector<double> &vel_min, std::vector<double> &vel_max,
                        std::vector<double> &damping, std::vector<double> &friction,
                        std::vector<double> &max_effort, const bool verbose)

    // cut tips + extra info
    bool URDFReadFromURDF(urdf::Model &urdf_model, Model *model, ModelDatad &model_data,
                        const FloatingBaseType floating_base_type,
                        const std::vector<std::string> &cut_tips,
                        std::vector<std::string> &joint_names,
                        std::vector<double> &position_min, std::vector<double> &position_max,
                        std::vector<double> &vel_min, std::vector<double> &vel_max,
                        std::vector<double> &damping, std::vector<double> &friction,
                        std::vector<double> &max_effort, const bool verbose)
    
    // compatibility
    bool URDFReadFromURDF(urdf::Model &urdf_model, Model *model,
                        const FloatingBaseType floating_base_type,
                        std::vector<std::string> &joint_names,
                        std::vector<double> &position_min, std::vector<double> &position_max,
                        std::vector<double> &vel_min, std::vector<double> &vel_max,
                        std::vector<double> &damping, std::vector<double> &friction,
                        std::vector<double> &max_effort, const bool verbose)
    
    // ============================================================
    // from a string
    // ============================================================

    bool initializeURDFModelFromString(urdf::Model &urdf_model, const char *model_xml_string)
    
    bool URDFReadFromString(const char *model_xml_string, Model *model, ModelDatad &model_data,
                            const FloatingBaseType floating_base_type, const bool verbose)
    
    bool URDFReadFromString(const char *model_xml_string, Model *model,
                            ModelDatad &model_data, const FloatingBaseType floating_base_type,
                            std::vector<std::string> &joint_names,
                            std::vector<double> &position_min, std::vector<double> &position_max,
                            std::vector<double> &vel_min, std::vector<double> &vel_max,
                            std::vector<double> &damping, std::vector<double> &friction,
                            std::vector<double> &max_effort, const bool verbose)
    
    // compatibility
    bool URDFReadFromString(const char *model_xml_string, Model *model,
                            const FloatingBaseType floating_base_type,
                            std::vector<std::string> &joint_names,
                            std::vector<double> &position_min, std::vector<double> &position_max,
                            std::vector<double> &vel_min, std::vector<double> &vel_max,
                            std::vector<double> &damping, std::vector<double> &friction,
                            std::vector<double> &max_effort, const bool verbose)
    
    // compatibility
    bool URDFReadFromString(const char *model_xml_string, Model *model,
                            const FloatingBaseType floating_base_type, const bool verbose)
    
    // ============================================================
    // from the parameter server
    // ============================================================

    bool initializeURDFModelFromParamServer(urdf::Model &urdf_model)
    
    bool URDFReadFromParamServer(Model *model, ModelDatad &model_data,
                                const FloatingBaseType floating_base_type, const bool verbose)
    
    bool URDFReadFromParamServer(Model *model, ModelDatad &model_data,
                                const FloatingBaseType floating_base_type,
                                std::vector<std::string> &joint_names,
                                std::vector<double> &position_min, std::vector<double> &position_max,
                                std::vector<double> &vel_min, std::vector<double> &vel_max,
                                std::vector<double> &damping, std::vector<double> &friction,
                                std::vector<double> &max_effort, const bool verbose)
    
    // cut tips
    bool URDFReadFromParamServer(Model *model, ModelDatad &model_data,
                                const std::vector<std::string> &tipLinks,
                                const FloatingBaseType floating_base_type, const bool verbose)
    
    bool URDFReadFromParamServer(Model *model, ModelDatad &model_data,
                                const FloatingBaseType floating_base_type,
                                const std::vector<std::string> &tipLinks,
                                std::vector<std::string> &joint_names,
                                std::vector<double> &position_min, std::vector<double> &position_max,
                                std::vector<double> &vel_min, std::vector<double> &vel_max,
                                std::vector<double> &damping, std::vector<double> &friction,
                                std::vector<double> &max_effort, const bool verbose)

    // compatibility
    bool URDFReadFromParamServer(Model *model, const FloatingBaseType floating_base_type,
                                const bool verbose)
    
    // compatibility
    bool URDFReadFromParamServer(Model *model, const std::vector<std::string> &tipLinks,
                                const FloatingBaseType floating_base_type, const bool verbose)
    
    // compatibility
    bool URDFReadFromParamServer(Model *model, const FloatingBaseType floating_base_type,
                                std::vector<std::string> &joint_names,
                                std::vector<double> &position_min, std::vector<double> &position_max,
                                std::vector<double> &vel_min, std::vector<double> &vel_max,
                                std::vector<double> &damping, std::vector<double> &friction,
                                std::vector<double> &max_effort, const bool verbose)
    
    // compatibility
    bool URDFReadFromParamServer(Model *model, const FloatingBaseType floating_base_type,
                                const std::vector<std::string> &tipLinks,
                                std::vector<std::string> &joint_names,
                                std::vector<double> &position_min, std::vector<double> &position_max,
                                std::vector<double> &vel_min, std::vector<double> &vel_max,
                                std::vector<double> &damping, std::vector<double> &friction,
                                std::vector<double> &max_effort, const bool verbose)
    

    // ============================================================
    // from a file
    // ============================================================

    bool initializeURDFModelFromFile(urdf::Model &urdf_model, const char *filename)
    
    bool URDFReadFromFile(const char *filename, Model *model, ModelDatad &model_data,
                        const FloatingBaseType floating_base_type, const bool verbose)
    
    bool URDFReadFromFile(const char *filename, Model *model, ModelDatad &model_data,
                        const FloatingBaseType floating_base_type,
                        std::vector<std::string> &joint_names,
                        std::vector<double> &position_min, std::vector<double> &position_max,
                        std::vector<double> &vel_min, std::vector<double> &vel_max,
                        std::vector<double> &damping, std::vector<double> &friction,
                        std::vector<double> &max_effort, const bool verbose)
    
    // compatibility
    bool URDFReadFromFile(const char *filename, Model *model,
                        const FloatingBaseType floating_base_type, const bool verbose)
    
    // compatibility
    bool URDFReadFromFile(const char *filename, Model *model, const FloatingBaseType floating_base_type,
                        std::vector<std::string> &joint_names,
                        std::vector<double> &position_min, std::vector<double> &position_max,
                        std::vector<double> &vel_min, std::vector<double> &vel_max,
                        std::vector<double> &damping, std::vector<double> &friction,
                        std::vector<double> &max_effort, const bool verbose)
    
    ///////////////////////////

    bool parseExtraInformation(urdf::Model &urdf_model, Model *rbdl_model,
                            std::vector<std::string> &joint_names, std::vector<double> &position_min,
                            std::vector<double> &position_max, std::vector<double> &vel_min,
                            std::vector<double> &vel_max, std::vector<double> &damping,
                            std::vector<double> &friction, std::vector<double> &max_effort,
                            const FloatingBaseType floating_base_type)
}

/* _RBDL_URDFREADER_H */
#endif