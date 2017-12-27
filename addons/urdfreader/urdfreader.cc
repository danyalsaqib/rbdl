#include <rbdl/rbdl.h>

#include <rbdl/addons/urdfreader/urdfreader.h>

#include <assert.h>
#include <iostream>
#include <fstream>
#include <map>
#include <stack>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include "ros/ros.h"

typedef boost::shared_ptr<urdf::Link> LinkPtr;
typedef const boost::shared_ptr<const urdf::Link> ConstLinkPtr;
typedef boost::shared_ptr<urdf::Joint> JointPtr;

using namespace std;


/* Check if the specified link is inside this urdf submodel */
bool isLinkInUrdfModel(LinkPtr link, std::string tip){
  if(tip == link->name){
    return true;
  }
  else{
    if(link->child_links.empty()){
      return false;
    }
    else{
      bool found = false;
      for(unsigned int i=0; i<link->child_links.size() && !found; ++i){
        found = found || isLinkInUrdfModel(link->child_links[i], tip);
      }
      return found;
    }
  }
}

namespace RigidBodyDynamics {

  namespace Addons {

    using namespace Math;

    typedef vector<LinkPtr> URDFLinkVector;
    typedef vector<JointPtr> URDFJointVector;
    typedef map<string, LinkPtr > URDFLinkMap;
    typedef map<string, JointPtr > URDFJointMap;

    bool construct_model(Model &rbdl_model, LinkPtr urdf_link, int parent_id, FloatingBaseType floatingBaseType,
                         const std::string &root_name,
                         bool verbose){

      int new_id = 0;
      if(urdf_link->parent_joint.get()){
        JointPtr urdf_joint = urdf_link->parent_joint;
        // create the joint
        Joint rbdl_joint;
        if ((urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS) && !urdf_joint->mimic) {
          rbdl_joint = Joint (SpatialVector (urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.));

        }
        else if (urdf_joint->type == urdf::Joint::PRISMATIC && !urdf_joint->mimic) {
          rbdl_joint = Joint (SpatialVector (0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        }
        else if (urdf_joint->type == urdf::Joint::FIXED || urdf_joint->mimic) {
          rbdl_joint = Joint (JointTypeFixed);
        }

        // compute the joint transformation
        Vector3d joint_rpy;
        Vector3d joint_translation;
        urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
        joint_translation.set (
              urdf_joint->parent_to_joint_origin_transform.position.x,
              urdf_joint->parent_to_joint_origin_transform.position.y,
              urdf_joint->parent_to_joint_origin_transform.position.z
              );
        SpatialTransform rbdl_joint_frame =
            Xrot (joint_rpy[0], Vector3d (1., 0., 0.))
            * Xrot (joint_rpy[1], Vector3d (0., 1., 0.))
            * Xrot (joint_rpy[2], Vector3d (0., 0., 1.))
            * Xtrans (Vector3d (
                        joint_translation
                        ));

        // assemble the body
        Vector3d link_inertial_position;
        Vector3d link_inertial_rpy;
        Matrix3d link_inertial_inertia = Matrix3d::Zero();
        double link_inertial_mass = 0;

        // but only if we actually have inertial data
        if (urdf_link->inertial) {
          link_inertial_mass = urdf_link->inertial->mass;

          link_inertial_position.set (
                urdf_link->inertial->origin.position.x,
                urdf_link->inertial->origin.position.y,
                urdf_link->inertial->origin.position.z
                );
          urdf_link->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

          link_inertial_inertia(0,0) = urdf_link->inertial->ixx;
          link_inertial_inertia(0,1) = urdf_link->inertial->ixy;
          link_inertial_inertia(0,2) = urdf_link->inertial->ixz;

          link_inertial_inertia(1,0) = urdf_link->inertial->ixy;
          link_inertial_inertia(1,1) = urdf_link->inertial->iyy;
          link_inertial_inertia(1,2) = urdf_link->inertial->iyz;

          link_inertial_inertia(2,0) = urdf_link->inertial->ixz;
          link_inertial_inertia(2,1) = urdf_link->inertial->iyz;
          link_inertial_inertia(2,2) = urdf_link->inertial->izz;


          if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
            cerr << "Error while processing body '" << urdf_link->name <<
                    "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
          }
        }
        Body rbdl_body = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);

        new_id = rbdl_model.AddBody (parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, urdf_link->name);
      }
      else{
        //If a floating base is desired we need to compute the inertia data of the floating base link,
        //if the floating base has no inertial details, then it makes no sense to make it floating
        //base (Like kdl). (The root is the only link allowed to be floating base

        if(floatingBaseType != +FloatingBaseType::FixedBase){
          double link_inertial_mass = 0;
          Vector3d link_inertial_position;
          Vector3d link_inertial_rpy;
          Matrix3d link_inertial_inertia = Matrix3d::Zero();

          if (urdf_link->inertial) {
            link_inertial_mass = urdf_link->inertial->mass;

            link_inertial_position.set (
                  urdf_link->inertial->origin.position.x,
                  urdf_link->inertial->origin.position.y,
                  urdf_link->inertial->origin.position.z
                  );
            urdf_link->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

            link_inertial_inertia(0,0) = urdf_link->inertial->ixx;
            link_inertial_inertia(0,1) = urdf_link->inertial->ixy;
            link_inertial_inertia(0,2) = urdf_link->inertial->ixz;

            link_inertial_inertia(1,0) = urdf_link->inertial->ixy;
            link_inertial_inertia(1,1) = urdf_link->inertial->iyy;
            link_inertial_inertia(1,2) = urdf_link->inertial->iyz;

            link_inertial_inertia(2,0) = urdf_link->inertial->ixz;
            link_inertial_inertia(2,1) = urdf_link->inertial->iyz;
            link_inertial_inertia(2,2) = urdf_link->inertial->izz;

            if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
              cerr << "Error while processing body '" << urdf_link->name <<
                      "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
            }
          }


          Body base = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);
          //make floating base
          if(floatingBaseType == +FloatingBaseType::XY_Yaw){

            ROS_DEBUG_STREAM("CREATING PLANAR FLOATING BASE");
            Joint floating_base_joint (
                  SpatialVector (0., 0., 0., 1., 0., 0.),
                  SpatialVector (0., 0., 0., 0., 1., 0.),
                  SpatialVector (0., 0., 1., 0., 0., 0.)
                  );

            new_id = rbdl_model.AddBody (0, Xtrans (Vector3d (0., 0., 0.)), floating_base_joint, base, urdf_link->name);
          }
          else if(floatingBaseType == +FloatingBaseType::XYZ_RollPitchYaw){

            ROS_DEBUG_STREAM("Creating 6D FLOATING BASE");
            Joint floating_base_joint (
                  SpatialVector (0., 0., 0., 1., 0., 0.),
                  SpatialVector (0., 0., 0., 0., 1., 0.),
                  SpatialVector (0., 0., 0., 0., 0., 1.),
                  SpatialVector (0., 0., 1., 0., 0., 0.),
                  SpatialVector (0., 1., 0., 0., 0., 0.),
                  SpatialVector (1., 0., 0., 0., 0., 0.)
                  );

            new_id = rbdl_model.AddBody(0, Xtrans (Vector3d (0., 0., 0.)), floating_base_joint, base, urdf_link->name);
          }
          else if(floatingBaseType == +FloatingBaseType::XYZ_Quaternion){

            ROS_DEBUG_STREAM("Creating 6D QUATERNION FLOATING BASE");
            Joint root_joint = JointTypeFloatingBase;

            SpatialTransform root_joint_frame = SpatialTransform ();

            new_id = rbdl_model.AddBody(0, root_joint_frame,
                                        root_joint,
                                        base,
                                        urdf_link->name);

          }
          else{

            throw std::runtime_error("Floating base type not supported");
          }
        }
        else
        {
          // Change the root name
          rbdl_model.mBodyNameMap.erase("ROOT");
          rbdl_model.mBodyNameMap[root_name] = 0;
        }
      }

      for(unsigned int i=0; i< urdf_link->child_links.size(); ++i){
        construct_model(rbdl_model, urdf_link->child_links[i], new_id, floatingBaseType, root_name, verbose);
      }

      return true;
    }



    bool construct_model(Model &rbdl_model, LinkPtr urdf_link, int parent_id, FloatingBaseType floatingBaseType,
                         const std::vector<std::string> &tipLinks, const std::string &root_name, bool verbose){

      int new_id = 0;
      if(urdf_link->parent_joint.get()){
        JointPtr urdf_joint = urdf_link->parent_joint;
        // create the joint
        Joint rbdl_joint;
        if ((urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS)  && !urdf_joint->mimic) {
          rbdl_joint = Joint (SpatialVector (urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.));
        }
        else if (urdf_joint->type == urdf::Joint::PRISMATIC && !urdf_joint->mimic) {
          rbdl_joint = Joint (SpatialVector (0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
        }
        else if (urdf_joint->type == urdf::Joint::FIXED || urdf_joint->mimic) {
          rbdl_joint = Joint (JointTypeFixed);
        }

        // compute the joint transformation
        Vector3d joint_rpy;
        Vector3d joint_translation;
        urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
        joint_translation.set (
              urdf_joint->parent_to_joint_origin_transform.position.x,
              urdf_joint->parent_to_joint_origin_transform.position.y,
              urdf_joint->parent_to_joint_origin_transform.position.z
              );
        SpatialTransform rbdl_joint_frame =
            Xrot (joint_rpy[0], Vector3d (1., 0., 0.))
            * Xrot (joint_rpy[1], Vector3d (0., 1., 0.))
            * Xrot (joint_rpy[2], Vector3d (0., 0., 1.))
            * Xtrans (Vector3d (
                        joint_translation
                        ));

        // assemble the body
        Vector3d link_inertial_position;
        Vector3d link_inertial_rpy;
        Matrix3d link_inertial_inertia = Matrix3d::Zero();
        double link_inertial_mass = 0;

        // but only if we actually have inertial data
        if (urdf_link->inertial) {
          link_inertial_mass = urdf_link->inertial->mass;

          link_inertial_position.set (
                urdf_link->inertial->origin.position.x,
                urdf_link->inertial->origin.position.y,
                urdf_link->inertial->origin.position.z
                );
          urdf_link->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

          link_inertial_inertia(0,0) = urdf_link->inertial->ixx;
          link_inertial_inertia(0,1) = urdf_link->inertial->ixy;
          link_inertial_inertia(0,2) = urdf_link->inertial->ixz;

          link_inertial_inertia(1,0) = urdf_link->inertial->ixy;
          link_inertial_inertia(1,1) = urdf_link->inertial->iyy;
          link_inertial_inertia(1,2) = urdf_link->inertial->iyz;

          link_inertial_inertia(2,0) = urdf_link->inertial->ixz;
          link_inertial_inertia(2,1) = urdf_link->inertial->iyz;
          link_inertial_inertia(2,2) = urdf_link->inertial->izz;

          if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
            cerr << "Error while processing body '" << urdf_link->name <<
                    "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
          }
        }
        Body rbdl_body = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);

        new_id = rbdl_model.AddBody (parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, urdf_link->name);
      }
      else{
        //If a floating base is desired we need to compute the inertia data of the floating base link,
        //if the floating base has no inertial details, then it makes no sense to make it floating
        //base (Like kdl). (The root is the only link allowed to be floating base

        if(floatingBaseType != +FloatingBaseType::FixedBase){
          double link_inertial_mass = 0;
          Vector3d link_inertial_position;
          Vector3d link_inertial_rpy;
          Matrix3d link_inertial_inertia = Matrix3d::Zero();

          if (urdf_link->inertial) {
            link_inertial_mass = urdf_link->inertial->mass;

            link_inertial_position.set (
                  urdf_link->inertial->origin.position.x,
                  urdf_link->inertial->origin.position.y,
                  urdf_link->inertial->origin.position.z
                  );
            urdf_link->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

            link_inertial_inertia(0,0) = urdf_link->inertial->ixx;
            link_inertial_inertia(0,1) = urdf_link->inertial->ixy;
            link_inertial_inertia(0,2) = urdf_link->inertial->ixz;

            link_inertial_inertia(1,0) = urdf_link->inertial->ixy;
            link_inertial_inertia(1,1) = urdf_link->inertial->iyy;
            link_inertial_inertia(1,2) = urdf_link->inertial->iyz;

            link_inertial_inertia(2,0) = urdf_link->inertial->ixz;
            link_inertial_inertia(2,1) = urdf_link->inertial->iyz;
            link_inertial_inertia(2,2) = urdf_link->inertial->izz;

            // Change the root name
            rbdl_model.mBodyNameMap.erase("ROOT");
            rbdl_model.mBodyNameMap[root_name] = 0;

            if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
              cerr << "Error while processing body '" << urdf_link->name <<
                      "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
            }
          }

          Body base = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);
          //make floating base
          if(floatingBaseType == +FloatingBaseType::XY_Yaw){

            ROS_DEBUG_STREAM("CREATING PLANAR FLOATING BASE");
            Joint floating_base_joint (
                  SpatialVector (0., 0., 0., 1., 0., 0.),
                  SpatialVector (0., 0., 0., 0., 1., 0.),
                  SpatialVector (0., 0., 1., 0., 0., 0.)
                  );

            new_id = rbdl_model.AddBody (0, Xtrans (Vector3d (0., 0., 0.)), floating_base_joint, base, urdf_link->name);
          }
          else if(floatingBaseType == +FloatingBaseType::XYZ_RollPitchYaw){

            ROS_DEBUG_STREAM("Creating 6D FLOATING BASE");
            Joint floating_base_joint (
                  SpatialVector (0., 0., 0., 1., 0., 0.),
                  SpatialVector (0., 0., 0., 0., 1., 0.),
                  SpatialVector (0., 0., 0., 0., 0., 1.),
                  SpatialVector (0., 0., 1., 0., 0., 0.),
                  SpatialVector (0., 1., 0., 0., 0., 0.),
                  SpatialVector (1., 0., 0., 0., 0., 0.)
                  );

            new_id = rbdl_model.AddBody(0, Xtrans (Vector3d (0., 0., 0.)), floating_base_joint, base, urdf_link->name);
          }
          else if(floatingBaseType == +FloatingBaseType::XYZ_Quaternion){

            ROS_DEBUG_STREAM("Creating 6D QUATERNION FLOATING BASE");
            Joint root_joint = JointTypeFloatingBase;

            SpatialTransform root_joint_frame = SpatialTransform ();

            new_id = rbdl_model.AddBody(0, root_joint_frame,
                                        root_joint,
                                        base,
                                        urdf_link->name);

          }
          else{
            std::cerr<<static_cast<int>(floatingBaseType)<<std::endl;
            throw std::runtime_error("Floating base type not supported in rbdl parser");
          }
        }
        else
        {
          // Change the root name
          rbdl_model.mBodyNameMap.erase("ROOT");
          rbdl_model.mBodyNameMap[root_name] = 0;
        }
      }

      /* The current link is a desired tip */
      bool tip_found = false;
      for(unsigned int i=0; i<tipLinks.size() && !tip_found; ++i){
        if(tipLinks[i] == urdf_link->name){
          tip_found = true;
        }
      }

      if(!tip_found){
        for(unsigned int i=0; i< urdf_link->child_links.size(); ++i){
          /* Check that the branch contains atleast on of the specified links */
          bool found = false;
          for(unsigned int j=0; j<tipLinks.size() && !found; ++j){
            found = isLinkInUrdfModel(urdf_link->child_links[i], tipLinks[j]);
          }
          if(found){
            construct_model(rbdl_model, urdf_link->child_links[i], new_id, floatingBaseType, tipLinks, root_name, verbose);
          }
        }

      }

      return true;
    }


    bool construct_model_depht_first(Model &rbdl_model, urdf::Model &urdf_model,
                                     FloatingBaseType floatingBaseType, bool verbose){

      boost::shared_ptr<urdf::Link> root(boost::const_pointer_cast<urdf::Link>(urdf_model.getRoot()));

      return construct_model(rbdl_model, root, 0, floatingBaseType, urdf_model.getRoot()->name, verbose);
    }

    bool construct_model_depht_first(Model &rbdl_model, urdf::Model &urdf_model, FloatingBaseType floatingBaseType,
                                     const std::vector<std::string> &tipLinks, bool verbose){

      boost::shared_ptr<urdf::Link> root(boost::const_pointer_cast<urdf::Link>(urdf_model.getRoot()));

      return construct_model(rbdl_model, root, 0, floatingBaseType, tipLinks, urdf_model.getRoot()->name, verbose);
    }

    bool URDFReadFromFile (const char* filename, Model* model, FloatingBaseType floatingBaseType, bool verbose) {

      ifstream model_file (filename);
      if (!model_file) {
        cerr << "Error opening file '" << filename << "'." << endl;
        abort();
      }

      // reserve memory for the contents of the file
      string model_xml_string;
      model_file.seekg(0, std::ios::end);
      model_xml_string.reserve(model_file.tellg());
      model_file.seekg(0, std::ios::beg);
      model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

      model_file.close();

      return URDFReadFromString (model_xml_string.c_str(), model, floatingBaseType, verbose);
    }

    bool URDFReadFromString (const char* model_xml_string, Model* model, FloatingBaseType floatingBaseType, bool verbose) {

      urdf::Model urdf_model;
      if (!urdf_model.initString(model_xml_string)){
        ROS_ERROR("Failed to parse urdf file");
        return false;
      }

      if (!construct_model_depht_first (*model, urdf_model, floatingBaseType, verbose)) {
        cerr << "Error constructing model from urdf file." << endl;
        return false;
      }

      model->gravity.set (0., 0., -9.81);

      return true;
    }

    bool URDFReadFromParamServer (Model* model, FloatingBaseType floatingBaseType, bool verbose) {

      urdf::Model urdf_model;
      if (!urdf_model.initParam("robot_description")){
        ROS_ERROR("Failed to parse urdf file");
        return false;
      }

      if (!construct_model_depht_first (*model, urdf_model, floatingBaseType, verbose)) {
        cerr << "Error constructing model from urdf file." << endl;
        return false;
      }

      model->gravity.set (0., 0., -9.81);

      return true;
    }

    bool URDFReadFromURDF(urdf::Model &urdf_model, Model* model, FloatingBaseType floatingBaseType, bool verbose){

      if (!construct_model_depht_first (*model, urdf_model, floatingBaseType, verbose)) {
        cerr << "Error constructing model from urdf file." << endl;
        return false;
      }

      model->gravity.set (0., 0., -9.81);
      return true;
    }

    ///////////////////////////

    bool URDFReadFromFile (const char* filename, Model* model, FloatingBaseType floatingBaseType,
                           std::vector<std::string> &joint_names,
                           std::vector<double> &position_min,  std::vector<double> &position_max,
                           std::vector<double> &vel_min,  std::vector<double> &vel_max,
                           std::vector<double> &damping, std::vector<double> &friction,
                           std::vector<double> &max_effort,
                           bool verbose){

      ifstream model_file (filename);
      if (!model_file) {
        cerr << "Error opening file '" << filename << "'." << endl;
        abort();
      }

      // reserve memory for the contents of the file
      string model_xml_string;
      model_file.seekg(0, std::ios::end);
      model_xml_string.reserve(model_file.tellg());
      model_file.seekg(0, std::ios::beg);
      model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

      model_file.close();

      return URDFReadFromString(model_xml_string.c_str(), model, floatingBaseType,
                                joint_names,
                                position_min, position_max,
                                vel_min, vel_max,
                                damping, friction, max_effort, verbose);
    }

    bool URDFReadFromString (const char* model_xml_string, Model* model, FloatingBaseType floatingBaseType,
                             std::vector<std::string> &joint_names,
                             std::vector<double> &position_min,  std::vector<double> &position_max,
                             std::vector<double> &vel_min,  std::vector<double> &vel_max,
                             std::vector<double> &damping, std::vector<double> &friction,
                             std::vector<double> &max_effort,
                             bool verbose){

      urdf::Model urdf_model;
      if (!urdf_model.initString(model_xml_string)){
        ROS_ERROR("Failed to parse urdf file");
        return false;
      }

      return URDFReadFromURDF(urdf_model, model, floatingBaseType,
                              joint_names,
                              position_min, position_max,
                              vel_min, vel_max,
                              damping, friction,
                              max_effort, verbose);

    }

    bool URDFReadFromURDF(urdf::Model &urdf_model, Model* model, FloatingBaseType floatingBaseType,
                          std::vector<std::string> &joint_names,
                          std::vector<double> &position_min,  std::vector<double> &position_max,
                          std::vector<double> &vel_min,  std::vector<double> &vel_max,
                          std::vector<double> &damping, std::vector<double> &friction,
                          std::vector<double> &max_effort,
                          bool verbose){


      bool urdfOK = URDFReadFromURDF(urdf_model, model, floatingBaseType, verbose);
      bool extraOK = parseExtraInformation(urdf_model, model, joint_names,
                                           position_min, position_max,
                                           vel_min, vel_max,
                                           damping, friction, max_effort,
                                           floatingBaseType);

      return urdfOK && extraOK;

    }

    bool URDFReadFromParamServer(Model* model, FloatingBaseType floatingBaseType,
                                 std::vector<std::string> &joint_names,
                                 std::vector<double> &position_min,  std::vector<double> &position_max,
                                 std::vector<double> &vel_min,  std::vector<double> &vel_max,
                                 std::vector<double> &damping, std::vector<double> &friction,
                                 std::vector<double> &max_effort,
                                 bool verbose){

      urdf::Model urdf_model;
      if (!urdf_model.initParam("robot_description")){
        ROS_ERROR("Failed to parse urdf file");
        return false;
      }

      if (!construct_model_depht_first (*model, urdf_model, floatingBaseType, verbose)) {
        cerr << "Error constructing model from urdf file." << endl;
        return false;
      }

      model->gravity.set (0., 0., -9.81);

      bool extraOK = parseExtraInformation(urdf_model, model, joint_names,
                                           position_min, position_max,
                                           vel_min, vel_max,
                                           damping, friction, max_effort,
                                           floatingBaseType);

      return extraOK;
    }

    bool URDFReadFromParamServer(Model* model, const std::vector<std::string> &tipLinks, FloatingBaseType floatingBaseType, bool verbose){

      urdf::Model urdf_model;
      if (!urdf_model.initParam("robot_description")){
        ROS_ERROR("Failed to parse urdf file");
        return false;
      }

      if (!construct_model_depht_first (*model, urdf_model, floatingBaseType, tipLinks, verbose)) {
        cerr << "Error constructing model from urdf file." << endl;
        return false;
      }

      model->gravity.set (0., 0., -9.81);

      return true;
    }

    bool URDFReadFromParamServer(Model* model, FloatingBaseType floatingBaseType, const std::vector<std::string> &tipLinks,
                                 std::vector<std::string> &joint_names,
                                 std::vector<double> &position_min,  std::vector<double> &position_max,
                                 std::vector<double> &vel_min,  std::vector<double> &vel_max,
                                 std::vector<double> &damping, std::vector<double> &friction,
                                 std::vector<double> &max_effort,
                                 bool verbose){

      urdf::Model urdf_model;
      if (!urdf_model.initParam("robot_description")){
        ROS_ERROR("Failed to parse urdf file");
        return false;
      }

      if (!construct_model_depht_first (*model, urdf_model, floatingBaseType, tipLinks, verbose)) {
        cerr << "Error constructing model from urdf file." << endl;
        return false;
      }

      model->gravity.set (0., 0., -9.81);

      bool extraOK = parseExtraInformation(urdf_model, model, joint_names,
                                           position_min, position_max,
                                           vel_min, vel_max,
                                           damping, friction, max_effort,
                                           floatingBaseType);

      return extraOK;
    }

    ///////////////////////////

    bool parseExtraInformation(urdf::Model &urdf_model, Model *rbdl_model, std::vector<std::string> &joint_names,
                               std::vector<double> &position_min,  std::vector<double> &position_max,
                               std::vector<double> &vel_min,  std::vector<double> &vel_max,
                               std::vector<double> &damping, std::vector<double> &friction,
                               std::vector<double> &max_effort,
                               FloatingBaseType floatingBaseType){

      int nDof;
      if(floatingBaseType == +FloatingBaseType::XYZ_RollPitchYaw){
        nDof = rbdl_model->qdot_size - 6;
      }
      else if(floatingBaseType == +FloatingBaseType::XYZ_Quaternion){
        nDof = rbdl_model->qdot_size - 6;
      }
      else if(floatingBaseType == +FloatingBaseType::XY_Yaw){
        nDof = rbdl_model->qdot_size - 3;
      }
      else if(floatingBaseType == +FloatingBaseType::FixedBase){
        nDof = rbdl_model->qdot_size;
      }
      else{

        throw std::runtime_error("Floating base type not supported in rbdl parser extra information");
        return false;
      }
      joint_names.resize(nDof);
      position_min.resize(nDof);
      position_max.resize(nDof);
      vel_min.resize(nDof);
      vel_max.resize(nDof);
      damping.resize(nDof);
      friction.resize(nDof);
      max_effort.resize(nDof);

      std::string rootName = urdf_model.getRoot()->name;

      for(auto it = rbdl_model->mBodyNameMap.begin(); it != rbdl_model->mBodyNameMap.end(); ++it){

        if(it->first != "ROOT" && it->first != rootName){

          boost::shared_ptr<urdf::Link> urdf_link;
          //As the joints, the bodyes with movable joints start ad +1
          //Since the movable bodies have a small numbering and the fixed bodyes have
          //a high numbering will get n joint names in the lower numbers

          std::string body_name = it->first;
          urdf_model.getLink(body_name, urdf_link);

          if((urdf_link->parent_joint->type != urdf::Joint::FIXED) && !urdf_link->parent_joint->mimic) {

            unsigned id;
            if(floatingBaseType == +FloatingBaseType::XYZ_RollPitchYaw){
              id = rbdl_model->GetBodyId(body_name.c_str()) - 7;
            }
            else if(floatingBaseType == +FloatingBaseType::XYZ_Quaternion){
              id = rbdl_model->GetBodyId(body_name.c_str()) - 3;
            }
            else if(floatingBaseType == +FloatingBaseType::XY_Yaw){
              id = rbdl_model->GetBodyId(body_name.c_str()) - 4;
            }
            else if(floatingBaseType == +FloatingBaseType::FixedBase){
              id = rbdl_model->GetBodyId(body_name.c_str())- 1;
            }
            else{

              throw std::runtime_error("Floating base type not supported");
              return false;
            }

            ROS_DEBUG_STREAM("rbdl body name: "<<body_name);

            joint_names[id] = (urdf_link->parent_joint->name);
            ROS_DEBUG_STREAM(" names "<<urdf_link->parent_joint->name);
            //Store the joint limits position
            if ( urdf_link->parent_joint->type != urdf::Joint::CONTINUOUS ) {
              position_min[id] = (urdf_link->parent_joint->limits->lower);
              position_max[id] = (urdf_link->parent_joint->limits->upper);
            }
            else{
              position_min[id] = (-3.14);
              position_max[id] = (3.14);
            }
            //Store the joint limits velocity
            if ( urdf_link->parent_joint->type != urdf::Joint::CONTINUOUS ) {
              vel_min[id] = (-urdf_link->parent_joint->limits->velocity);
              vel_max[id] = (urdf_link->parent_joint->limits->velocity);
            }
            else{
              /// Random high value
              vel_min[id] = (-100.0);
              vel_max[id] = (100.0);
            }
            //Store joint damping
            if (urdf_link->parent_joint->dynamics) {
              damping[id] = (urdf_link->parent_joint->dynamics->damping);
            }
            else{
              damping[id] = (0.0);
            }
            //Store joint friction
            if (urdf_link->parent_joint->dynamics) {
              friction[id] = (urdf_link->parent_joint->dynamics->friction);
            }
            else{
              friction[id] = (0.0);
            }
            //Store joint max_effor
            if (urdf_link->parent_joint->limits) {
              max_effort[id] = (urdf_link->parent_joint->limits->effort);
            }
            else{
              max_effort[id] = (0.0);
            }
          }
        }
      }
      ROS_DEBUG_STREAM("parsed the rbdl model succesfully");
      return true;
    }

  }


}
