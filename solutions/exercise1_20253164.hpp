//
// Created by Woojin Kim on 2025/03/31.
//

#ifndef ME553_2025_SOLUTIONS_EXERCISE1_20253164_HPP_
#define ME553_2025_SOLUTIONS_EXERCISE1_20253164_HPP_

#include <Eigen/Core>
#include <raisim/math.hpp>
#include "tinyxml_rai/tinyxml_rai.h"
using namespace raisim;

// Specify the name of the end-effector joint
const std::string endEffectorName = "panda_finger_joint3";

// Define some functions for concise code writing
inline Vec<3> str2vec(const std::string& str);
inline TiXmlElement* find_joint_name(TiXmlElement *root, const std::string& jointName);
inline TiXmlElement* find_parent_joint(TiXmlElement *root, TiXmlElement *joint);
inline int find_gc_index(TiXmlElement *root, const std::string& jointName);

/// do not change the name of the method
inline Eigen::Vector3d getEndEffectorPosition (const Eigen::VectorXd& gc) {

  // Variables for saving solution
  Vec<3> sol, temp;
  sol.setZero();

  // Read the urdf file and save the end-effector element to the item
  TiXmlDocument doc;
  doc.LoadFile("../resource/Panda/panda.urdf");
  TiXmlElement *root = doc.FirstChildElement();
  TiXmlElement *item = find_joint_name(root, endEffectorName);

  // Run until reaching to the root
  while (item != nullptr) {

    // Save information of the current joint element
    const std::string joint_name = item->Attribute("name");
    const std::string joint_type = item->Attribute("type");

    // Save information about the origin pose and axis
    Vec<3> rpy, xyz, axis;

    if (item->FirstChildElement("origin")->Attribute("rpy") != nullptr)
      rpy = str2vec(item->FirstChildElement("origin")->Attribute("rpy"));
    else rpy.setZero();

    if (item->FirstChildElement("origin")->Attribute("xyz") != nullptr)
      xyz = str2vec(item->FirstChildElement("origin")->Attribute("xyz"));
    else xyz.setZero();

    if (item->FirstChildElement("axis") != nullptr)
      axis = str2vec(item->FirstChildElement("axis")->Attribute("xyz"));

    // Convert rotation information to rotation matrix
    Mat<3,3> R1;
    rpyToRotMat_intrinsic(rpy, R1);

    // Calculate according to the joint types
    if (joint_type == "revolute") {
      Mat<3,3> R2;
      angleAxisToRotMat(axis, gc(find_gc_index(root, joint_name)), R2);
      temp = xyz + R1 * R2 * sol; }
    else if (joint_type == "prismatic")
      temp = xyz + R1 * (axis * gc(find_gc_index(root, joint_name)) + sol);
    else if (joint_type == "fixed")
      temp = xyz + R1 * sol;
    sol = temp;

    // Go up the tree to the parent joint's location
    item = find_parent_joint(root, item);
  }
  return sol.e();
}

// Function that converts a string separated by spaces to the form of vector
inline Vec<3> str2vec(const std::string& str) {
  Vec<3> vec;
  const size_t del1 = str.find(' ');
  const size_t del2 = str.find(' ', del1 + 1);
  vec(0) = stod(str.substr(0, del1));
  vec(1) = stod(str.substr(del1 + 1, del2-del1));
  vec(2) = stod(str.substr(del2 + 1, str.length()-del2));
  return vec;
}

// Function that finds the joint element with the name received as a parameter
inline TiXmlElement* find_joint_name(TiXmlElement *root, const std::string& jointName) {
  TiXmlElement *item = root->FirstChildElement("joint");
  while (item != nullptr) {
    if (item->Attribute("name") == jointName) return item;
    item = item->NextSiblingElement("joint");
  }
  return nullptr;
}

// Function that finds parent joint of the joint received as a parameter
inline TiXmlElement* find_parent_joint(TiXmlElement *root, TiXmlElement* joint) {
  const std::string link_between = joint->FirstChildElement("parent")->Attribute("link");

  TiXmlElement *item = root->FirstChildElement("joint");
  while (item != nullptr) {
    if (item->FirstChildElement("child")->Attribute("link") == link_between) return item;
    item = item->NextSiblingElement("joint");
  }
  return nullptr;
}

// Function that returns the index of the joint received as parameters in the Generalized Coordinate
inline int find_gc_index(TiXmlElement *root, const std::string& jointName) {
  TiXmlElement *item = root->FirstChildElement("joint");
  int idx = 0;
  while (item != nullptr) {
    if (item->Attribute("name") == jointName) return idx;
    const std::string joint_type = item->Attribute("type");
    if (joint_type == "revolute" || joint_type == "prismatic") idx++;
    item = item->NextSiblingElement("joint");
  }
  return -1;
}

#endif // ME553_2025_SOLUTIONS_EXERCISE1_20253164_HPP_
