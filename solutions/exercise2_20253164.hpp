#pragma once

#include <Eigen/Core>
#include <raisim/math.hpp>
#include "tinyxml_rai/tinyxml_rai.h"
using namespace raisim;

// Specify the name of the end-effector joint
const std::string ENDEFFECTOR = "panda_finger_joint3";

// Define some functions for concise code writing
inline Eigen::Vector3d getJointPosition (const Eigen::VectorXd& gc, int joint_idx);
inline Eigen::Matrix3d getJointRotation (const Eigen::VectorXd& gc, int joint_idx);
inline Eigen::Vector3d str2vec(const std::string& str);
inline int get_joint_idx(const std::string& jointName);
inline Eigen::VectorXi get_joint_tree(int joint_idx);
inline int get_gc_index(int joint_idx);
inline Eigen::Matrix3d get_origin_rot(int joint_idx);
inline Eigen::Vector3d get_origin_pos(int joint_idx);
inline Eigen::Vector3d get_axis(int joint_idx);
inline std::string get_joint_type(int joint_idx);

/// do not change the name of the method
inline Eigen::Vector3d getLinearVelocity (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv) {

    // get joint tree from root to end-effector
    int ee_idx = get_joint_idx(ENDEFFECTOR);
    Eigen::VectorXi tree = get_joint_tree(ee_idx);

    // get positional Jacobian
    Eigen::Matrix3Xd J_p;
    J_p.resize(3, gv.size());
    J_p.setZero();
    for (int i = 0; i < tree.size(); i++) {
      if (get_joint_type(tree(i)) == "revolute")
        J_p.col(get_gc_index(tree(i))) << (getJointRotation(gc, tree(i))*get_axis(tree(i))).cross(getJointPosition(gc, ee_idx) - getJointPosition(gc, tree(i)));
      else if (get_joint_type(tree(i)) == "prismatic")
        J_p.col(get_gc_index(tree(i))) << getJointRotation(gc, tree(i))*get_axis(tree(i));
    }

    // calculate linear velocity
    Eigen::Vector3d lin_vel = J_p * gv;
    return lin_vel;
}

/// do not change the name of the method
inline Eigen::Vector3d getAngularVelocity (const Eigen::VectorXd& gc, const Eigen::VectorXd& gv) {

  // get joint tree from root to end-effector
  int ee_idx = get_joint_idx(ENDEFFECTOR);
  Eigen::VectorXi tree = get_joint_tree(ee_idx);

  //get angular Jacobian
  Eigen::Matrix3Xd J_a;
  J_a.resize(3, gv.size());
  J_a.setZero();
  for (int i = 0; i < tree.size(); i++) {
    if (get_joint_type(tree(i)) == "revolute")
      J_a.col(get_gc_index(tree(i))) << getJointRotation(gc, tree(i))*get_axis(tree(i));
  }

  // calculate angular velocity
  Eigen::Vector3d ang_vel = J_a * gv;
  return ang_vel;
}

// function that calculates the global position of chosen joint
inline Eigen::Vector3d getJointPosition (const Eigen::VectorXd& gc, int joint_idx) {

  // Variables for saving solution
  Vec<3> sol, temp;
  sol.setZero();

  // get joint tree from root to given joint
  Eigen::VectorXi tree = get_joint_tree(joint_idx);

  // Run until reaching to the root
  for (int i = tree.size() - 1; i >= 0; i--) {

    // save the information of the joint
    Mat<3,3> R1 = get_origin_rot(tree(i));
    Vec<3> xyz = get_origin_pos(tree(i));
    Vec<3> axis = get_axis(tree(i));

    // Calculate according to the joint types
    const std::string joint_type = get_joint_type(tree(i));
    if (joint_type == "revolute") {
      Mat<3,3> R2;
      angleAxisToRotMat(axis, gc(get_gc_index(tree(i))), R2);
      temp = xyz + R1 * R2 * sol; }
    else if (joint_type == "prismatic")
      temp = xyz + R1 * (axis * gc(get_gc_index(tree(i))) + sol);
    else if (joint_type == "fixed")
      temp = xyz + R1 * sol;
    sol = temp;
  }
  return sol.e();
}

// function that calculates the rotation matrix between world and chosen joint
inline Eigen::Matrix3d getJointRotation (const Eigen::VectorXd& gc, int joint_idx) {

  // Variables for saving solution
  Mat<3,3> sol, temp;
  sol.setIdentity();

  // get joint tree from root to given joint
  Eigen::VectorXi tree = get_joint_tree(joint_idx);

  // Run until reaching to the root
  for (int i = tree.size() - 1; i >= 0; i--) {

    // save the information of the joint
    Mat<3,3> R1 = get_origin_rot(tree(i));
    Vec<3> axis = get_axis(tree(i));

    // Calculate according to the joint types
    const std::string joint_type = get_joint_type(tree(i));
    if (joint_type == "revolute") {
      Mat<3,3> R2;
      angleAxisToRotMat(axis, gc(get_gc_index(tree(i))), R2);
      temp = R1 * R2 * sol; }
    else if (joint_type == "prismatic")
      temp = R1 * sol;
    else if (joint_type == "fixed")
      temp = R1 * sol;
    sol = temp;
  }
  return sol.e();
}

// Function that converts a string separated by spaces to the form of vector
inline Eigen::Vector3d str2vec(const std::string& str) {
  Eigen::Vector3d vec;
  const size_t del1 = str.find(' ');
  const size_t del2 = str.find(' ', del1 + 1);
  vec(0) = stod(str.substr(0, del1));
  vec(1) = stod(str.substr(del1 + 1, del2-del1));
  vec(2) = stod(str.substr(del2 + 1, str.length()-del2));
  return vec;
}

// Function that finds the index of given joint name
inline int get_joint_idx(const std::string& jointName) {
  TiXmlDocument doc;
  doc.LoadFile("../resource/Panda/panda.urdf");
  TiXmlElement *root = doc.FirstChildElement();
  TiXmlElement *item = root->FirstChildElement("joint");
  int idx = 0;
  while (item != nullptr) {
    if (item->Attribute("name") == jointName) return idx;
    item = item->NextSiblingElement("joint");
    idx++;
  }
  return -1;
}

// Function that finds the tree branch from root to given joint
inline Eigen::VectorXi get_joint_tree(const int joint_idx) {

  TiXmlDocument doc;
  doc.LoadFile("../resource/Panda/panda.urdf");
  TiXmlElement *root = doc.FirstChildElement();
  TiXmlElement *item = root->FirstChildElement("joint");
  for (int i = 0; i < joint_idx; i++) item = item->NextSiblingElement("joint");

  Eigen::VectorXi tree;
  while (item != nullptr) {
    Eigen::VectorXi temp(tree.size() + 1);
    for (int i = 0; i < tree.size(); i++) temp(i+1) = tree(i);
    tree = temp;
    tree(0) = get_joint_idx(item->Attribute("name"));

    const std::string parent_link = item->FirstChildElement("parent")->Attribute("link");
    item = root->FirstChildElement("joint");
    while (item != nullptr) {
      if (item->FirstChildElement("child")->Attribute("link") == parent_link) break;
      item = item->NextSiblingElement("joint");
    }
  }
  return tree;
}

// Function that returns the GC index of given joint
inline int get_gc_index(const int joint_idx) {
  TiXmlDocument doc;
  doc.LoadFile("../resource/Panda/panda.urdf");
  TiXmlElement *root = doc.FirstChildElement();
  TiXmlElement *item = root->FirstChildElement("joint");
  int gc_idx = 0;
  for (int i = 0; i < joint_idx; i++) {
    const std::string joint_type = item->Attribute("type");
    if (joint_type == "revolute" || joint_type == "prismatic") gc_idx++;
    item = item->NextSiblingElement("joint");
  }
  const std::string joint_type = item->Attribute("type");
  if (joint_type == "revolute" || joint_type == "prismatic") return gc_idx;
  return -1;
}

// Function that returns the rotation matrix of the origin of given joint
inline Eigen::Matrix3d get_origin_rot(int joint_idx) {
  TiXmlDocument doc;
  doc.LoadFile("../resource/Panda/panda.urdf");
  TiXmlElement *root = doc.FirstChildElement();
  TiXmlElement *item = root->FirstChildElement("joint");
  for (int i = 0; i < joint_idx; i++) item = item->NextSiblingElement("joint");

  Vec<3> rpy;
  if (item->FirstChildElement("origin")->Attribute("rpy") != nullptr)
    rpy = str2vec(item->FirstChildElement("origin")->Attribute("rpy"));
  else
    rpy.setZero();

  Mat<3,3> R1;
  rpyToRotMat_intrinsic(rpy, R1);

  return R1.e();
}

// Function that returns the positional offset of the origin of given joint
inline Eigen::Vector3d get_origin_pos(int joint_idx) {
  TiXmlDocument doc;
  doc.LoadFile("../resource/Panda/panda.urdf");
  TiXmlElement *root = doc.FirstChildElement();
  TiXmlElement *item = root->FirstChildElement("joint");
  for (int i = 0; i < joint_idx; i++) item = item->NextSiblingElement("joint");

  Eigen::Vector3d xyz;
  if (item->FirstChildElement("origin")->Attribute("xyz") != nullptr)
    xyz = str2vec(item->FirstChildElement("origin")->Attribute("xyz"));
  else
    xyz.setZero();

  return xyz;
}

// Function that returns the axis vector of given joint
inline Eigen::Vector3d get_axis(int joint_idx) {
  TiXmlDocument doc;
  doc.LoadFile("../resource/Panda/panda.urdf");
  TiXmlElement *root = doc.FirstChildElement();
  TiXmlElement *item = root->FirstChildElement("joint");
  for (int i = 0; i < joint_idx; i++) item = item->NextSiblingElement("joint");

  Eigen::Vector3d axis;
  if (item->FirstChildElement("axis") != nullptr)
    axis = str2vec(item->FirstChildElement("axis")->Attribute("xyz"));
  else
    axis.setZero();

  return axis;
}

// Function that returns the joint type of given joint
inline std::string get_joint_type(int joint_idx) {
  TiXmlDocument doc;
  doc.LoadFile("../resource/Panda/panda.urdf");
  TiXmlElement *root = doc.FirstChildElement();
  TiXmlElement *item = root->FirstChildElement("joint");
  for (int i = 0; i < joint_idx; i++) item = item->NextSiblingElement("joint");

  std::string type = item->Attribute("type");
  return type;
}