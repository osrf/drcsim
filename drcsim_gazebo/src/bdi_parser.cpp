/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* this programs takes BDI's cfg file, and generates an URDF */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <utility>
#include <map>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <tinyxml.h>

#if USE_ROS
#include <ros/package.h>
#endif

enum StructType { LINK, JOINT };

std::string findNextKeyword(std::ifstream &ifs,
  boost::shared_ptr<urdf::ModelInterface> model, std::string struct_tok)
{
  std::string line;
  std::getline(ifs, line);
  // find first keyword "struct "
  while (ifs.good() && line.find(struct_tok) == std::string::npos)
    std::getline(ifs, line);
  if (!line.empty())
  {
    boost::trim(line);
    size_t pos1 = line.find(struct_tok) + struct_tok.size();
    return line.substr(pos1, line.size()-pos1);
  }
  return std::string();
}


std::pair<std::string, std::vector<std::string> > findNextKeyValuesPair(
  std::ifstream &ifs, boost::shared_ptr<urdf::ModelInterface> model,
  std::string delim_tok)
{
  std::string line;
  std::getline(ifs, line);
  // find first keyword "struct "
  while (ifs.good() && line.find(delim_tok) == std::string::npos)
    std::getline(ifs, line);
  if (!line.empty())
  {
    boost::trim(line);
    size_t pos1 = line.find(delim_tok) + delim_tok.size();
    std::string vals = line.substr(pos1, line.size()-pos1);
    boost::trim(vals);
    // split vals into pieces
    std::vector<std::string> pieces;
    boost::split(pieces, vals, boost::is_any_of(" "));

    // how to figure out what the key is
    std::string key = line.substr(0, pos1-1);
    boost::trim(key);
    return std::pair<std::string, std::vector<std::string> >(key, pieces);
  }
  return std::pair<std::string, std::vector<std::string> >(std::string(),
    std::vector<std::string>());
}

urdf::Vector3 stringToVector3(std::string str, double scale = 1)
{
  if (!str.empty())
  {
    std::vector<std::string> pieces;
    std::vector<double> vals;

    boost::split(pieces, str, boost::is_any_of(" "));
    for (unsigned int i = 0; i < pieces.size(); ++i)
    {
      if (pieces[i] != "")
      {
        try
        {
          vals.push_back(scale *
                         boost::lexical_cast<double>(pieces[i].c_str()));
        }
        catch(boost::bad_lexical_cast &e)
        {
          printf("xml key [%s][%d] value [%s] is not a valid double"
                 " from a 3-tuple\n", str.c_str(), i, pieces[i].c_str());
          return urdf::Vector3(0, 0, 0);
        }
      }
    }
    return urdf::Vector3(vals[0], vals[1], vals[2]);
  }
  else
  {
    std::cout << "WARNING:   stringToVector3, input string empty\n";
    return urdf::Vector3(0, 0, 0);
  }
}


std::pair<std::string, std::string> findNextKeyValuePair(
  std::ifstream &ifs, boost::shared_ptr<urdf::ModelInterface> model,
  std::string delim_tok)
{
  std::string line;
  std::getline(ifs, line);
  // find first keyword "struct "
  while (ifs.good() && line.find(delim_tok) == std::string::npos)
    std::getline(ifs, line);
  if (!line.empty())
  {
    boost::trim(line);
    size_t pos1 = line.find(delim_tok) + delim_tok.size();
    std::string val = line.substr(pos1, line.size()-pos1);
    boost::trim(val);
    std::string key = line.substr(0, pos1-1);
    boost::trim(key);
    return std::pair<std::string, std::string>(key, val);
  }
  return std::pair<std::string, std::string>(std::string(), std::string());
}

std::pair<std::string, std::string> findNextStructOrKeyValuePair(
  std::ifstream &ifs, boost::shared_ptr<urdf::ModelInterface> model,
  std::string struct_tok, std::string delim_tok)
{
  std::string line;
  std::getline(ifs, line);
  // find first keyword "struct "
  while (ifs.good() && line.find(delim_tok) == std::string::npos)
    std::getline(ifs, line);
  if (!line.empty())
  {
    boost::trim(line);
    size_t pos1 = line.find(delim_tok) + delim_tok.size();
    std::string val = line.substr(pos1, line.size()-pos1);
    boost::trim(val);
    std::string key = line.substr(0, pos1-1);
    boost::trim(key);
    return std::pair<std::string, std::string>(key, val);
  }
  return std::pair<std::string, std::string>(std::string(), std::string());
}

void removeComments(std::string &val)
{
  size_t pos = val.find("#", 0);
  if (pos != std::string::npos)
    val = val.substr(0, pos - 1);
}

void printTree(boost::shared_ptr<const urdf::Link> link, int level = 0)
{
  level += 2;
  int count = 0;
  for (std::vector<boost::shared_ptr<urdf::Link> >::const_iterator
       child = link->child_links.begin();
       child != link->child_links.end(); ++child)
  {
    if (*child)
    {
      for (int j = 0; j < level; ++j)
        std::cout << "  ";
      std::cout << "child(" << (count++)+1 << "):  "
                << (*child)->name  << std::endl;
      // first grandchild
      printTree(*child, level);
    }
    else
    {
      for (int j = 0; j < level; ++j)
        std::cout << " ";
      std::cout << "root link: " << link->name
                << " has a null child!" << *child << std::endl;
    }
  }
}

int main(int argc, char** argv)
{
  if (argc < 2)
    printf("run:\n./bdi_parser drc_skeleton.cfg\n");
  else
  {
    std::string filename(argv[1]);
    std::ifstream ifs(filename.c_str(), std::ios::in);

    boost::shared_ptr<urdf::ModelInterface> model(new urdf::ModelInterface);
    model->clear();

    if (ifs.good())
    {
      // read very first struct, use the value as model name
      model->name_ = findNextKeyword(ifs, model, "struct ");
      std::cout << "model name [" << model->name_ << "]\n";


      // read the first key value pair, apparently, this is a list of link names
      std::pair<std::string, std::vector<std::string> > key_values;
      key_values = findNextKeyValuesPair(ifs, model, "=");
      std::cout << "key [" << key_values.first << "] ";
      for (std::vector<std::string>::iterator it = key_values.second.begin();
           it != key_values.second.end(); ++it)
      {
        std::cout << " val [" << (*it) << "] ";
        // add each link to model
      }
      std::cout << " \n";


      // Next, read groups of (struct followed by key value pairs)
      std::string struct_tok("struct ");
      std::string delim_tok("=");
      int struct_level = 0;
      std::map<int, std::string> struct_name;
      std::string entity_name;
      while (ifs.good())
      {
        // read a line
        std::string line;
        std::getline(ifs, line);

        std::cout << "debug: " << line.find("#") << "\n";

        size_t comment_pos = line.find("#");
        if (comment_pos == 0)
        {
          // this line is a comment, skip
          continue;
        }
        else if (comment_pos != std::string::npos)
        {
          // truncate line from beginning to # sign
          line = line.substr(0, comment_pos);
        }

        std::string joint_namespace;
        if (line.find(struct_tok) != std::string::npos)
        {
          // this is a struct
          // struct_level = number of characters before keyword "struct"
          struct_level = line.find(struct_tok);
          if (!line.empty())
          {
            boost::trim(line);
            size_t pos1 = line.find(struct_tok) + struct_tok.size();
            struct_name[struct_level] = line.substr(pos1, line.size()-pos1);
          }

          if (struct_level == 1)
          {
            // potentially a link name, but it could be followed
            // by another struct, then it is not a link
            std::cout << "------------------------------------\n"
                      << "struct level [" << struct_level << "] "
                      << "name [" << struct_name[struct_level] << "]\n";

            entity_name = struct_name[struct_level];
          }
          else
          {
            // init a joint namespace
            // infor for link
            entity_name = struct_name[1];
            for (int i = 2; i <= struct_level; ++i)
              entity_name = entity_name + "." + struct_name[i];
            std::cout << "struct level [" << struct_level << "] ";
            std::cout << "current struct name [" << entity_name << "]\n";


            // just checking, joint should have been created when
            // reading link struct
            boost::shared_ptr<urdf::Joint> joint =
              model->joints_.find(entity_name)->second;
            if (!joint)
              std::cout << "     intermediate struct, not a joint name ["
                        << entity_name << "].\n";
          }
        }
        else if (line.find("=") != std::string::npos)
        {
          // this is a key value pair
          if (struct_level == 0)
          {
            // should not be here, should have found a struct first
          }
          else if (struct_level == 1)
          {
            // infor for link
            // insert link to model
            // add key value pair to link
            boost::shared_ptr<urdf::Link> link =
              model->links_.find(entity_name)->second;
            if (!link)
            {
              std::cout << "  LINK: Creating [" << entity_name << "]\n";
              link.reset(new urdf::Link);
              link->name = entity_name;
              model->links_.insert(std::make_pair(link->name, link));

              link->inertial.reset(new urdf::Inertial());
              link->visual.reset(new urdf::Visual());
              link->collision.reset(new urdf::Collision());
            }

            // parse key value pair
            std::string key, val;
            if (!line.empty())
            {
              boost::trim(line);
              size_t pos1 = line.find(delim_tok) + delim_tok.size();
              val = line.substr(pos1, line.size()-pos1);
              removeComments(val);
              boost::trim(val);
              key = line.substr(0, pos1-1);
              boost::trim(key);
            }
            std::cout << "    LINK: key [" << key << "] "
                      << "val [" << val << "]\n";


            if (key == "parent_link")
            {
              std::cout << "          parent link [" << val << "]\n";
              // add parent to child link, add child to parent link
              boost::shared_ptr<urdf::Link> parent =
                model->links_.find(val)->second;
              parent->child_links.push_back(link);
              link->setParent(parent);
            }
            else if (key == "parent_kin_dof")
            {
              std::cout << "          parent joint [" << val << "]\n";

              // create parent joint with name
              boost::shared_ptr<urdf::Joint> joint;
              std::cout << "  JOINT: Creating [" << val << "]\n";
              joint.reset(new urdf::Joint);

              // as ROS Graph Resource Names do not allow "." characters,
              // replace with _
              std::replace(val.begin(), val.end(), '.', '_');
              joint->name = val;
              std::cout << "\n\n" << joint->name << "\n\n";

              joint->limits.reset(new urdf::JointLimits());
              joint->safety.reset(new urdf::JointSafety());
              joint->dynamics.reset(new urdf::JointDynamics());
              model->joints_.insert(std::make_pair(joint->name, joint));

              // this is the parent joint for link
              link->parent_joint = joint;
              joint->child_link_name = link->name;
              joint->parent_link_name = link->getParent()->name;
            }
            else if (key == "mass")
            {
              link->inertial->mass = boost::lexical_cast<double>(val);
            }
            else if (key == "moi_xx")
            {
              link->inertial->ixx = boost::lexical_cast<double>(val);
            }
            else if (key == "moi_xy")
            {
              link->inertial->ixy = boost::lexical_cast<double>(val);
            }
            else if (key == "moi_xz")
            {
              link->inertial->ixz = boost::lexical_cast<double>(val);
            }
            else if (key == "moi_yy")
            {
              link->inertial->iyy = boost::lexical_cast<double>(val);
            }
            else if (key == "moi_yz")
            {
              link->inertial->iyz = boost::lexical_cast<double>(val);
            }
            else if (key == "moi_zz")
            {
              link->inertial->izz = boost::lexical_cast<double>(val);
            }
            else if (key == "com_x")
            {
              link->inertial->origin.position.x =
                boost::lexical_cast<double>(val);
            }
            else if (key == "com_y")
            {
              link->inertial->origin.position.y =
                boost::lexical_cast<double>(val);
            }
            else if (key == "com_z")
            {
              link->inertial->origin.position.z =
                boost::lexical_cast<double>(val);
	    }
	    else if (key == "visual")
	      {
		link->visual.reset(); // This may cause a memory leak?

		boost::shared_ptr<urdf::Mesh> mesh_dae;
		mesh_dae.reset(new urdf::Mesh);
		mesh_dae->filename = std::string("package://atlas_description/meshes_v3/") +
		  val + std::string(".dae");
		boost::shared_ptr<urdf::Visual> visual(new urdf::Visual);
		// visual = new urdf::Visual();
		visual->geometry = mesh_dae;
		link->visual_array.push_back(visual);
	      }
	    else if (key == "collision")
	      {
		link->collision.reset(); // This may cause a memory leak?

		boost::shared_ptr<urdf::Mesh> mesh_stl;
		mesh_stl.reset(new urdf::Mesh);
		mesh_stl->filename = std::string("package://atlas_description/meshes_v3/") +
		  val + std::string(".stl");
		boost::shared_ptr<urdf::Collision> collision(new urdf::Collision);
		collision->geometry = mesh_stl;
		link->collision_array.push_back(collision);
	      }

            // insert collision and visual block for the robot manually,
            // currently the files I get have names that corresponds to
            // link name, so I can hack up a filename reference for each link
	    if (link->visual) {
            boost::shared_ptr<urdf::Mesh> mesh_dae;
            mesh_dae.reset(new urdf::Mesh);
            mesh_dae->filename = std::string("package://atlas_description/meshes_v3/") +
                                 entity_name + std::string(".dae");
            link->visual->geometry = mesh_dae;
	    }
	    if (link->collision) {
            boost::shared_ptr<urdf::Mesh> mesh_stl;
            mesh_stl.reset(new urdf::Mesh);
            mesh_stl->filename = std::string("package://atlas_description/meshes_v3/") +
                                 entity_name + std::string(".stl");
            link->collision->geometry = mesh_stl;
	    }
          }
          else
          {
            // this is a joint name
            // as ROS Graph Resource Names do not allow "." characters,
            // replace with _
            std::replace(entity_name.begin(), entity_name.end(), '.', '_');
              std::cout << "\n\n" << entity_name << "\n\n";

            // parse key value pair
            std::string key, val;
            if (!line.empty())
            {
              boost::trim(line);
              size_t pos1 = line.find(delim_tok) + delim_tok.size();
              val = line.substr(pos1, line.size()-pos1);
              removeComments(val);
              boost::trim(val);
              key = line.substr(0, pos1-1);
              boost::trim(key);
            }
            std::cout << "    JOINT: key [" << key << "] "
                      << "val [" << val << "]\n";

            // add key value pair to join
            boost::shared_ptr<urdf::Joint> joint =
              model->joints_.find(entity_name)->second;
            // std::cout << "    debug [" << joint->name << "] has "
            //           << " parent [" << joint->parent_link_name
            //           << "] child [" << joint->child_link_name << "]\n";

            if (!joint)
            {
              // this joint is not referred by any link, therefore,
              // not created yet here.
              /* we can create this joint, but it has no parent / child
              std::cout << "  JOINT: Creating [" << entity_name << "]\n";
              joint.reset(new urdf::Joint);
              joint->name = entity_name;
              model->joints_.insert(std::make_pair(joint->name, joint));
              */
              std::cout << "  JOINT: [" << entity_name
                        << "] is not referred to by any link\n";
            }
            else if (!val.empty())
            {
              if (key == "offset")
              {
                // add parent to child transform
                joint->parent_to_joint_origin_transform.position =
                  stringToVector3(val);
                std::cout << "  JOINT: [" << entity_name << "] origin ["
                          << joint->parent_to_joint_origin_transform.position.x
                          << ", "
                          << joint->parent_to_joint_origin_transform.position.y
                          << ", "
                          << joint->parent_to_joint_origin_transform.position.z
                          << "]\n";
              }
              else if (key == "axis")
              {
                // add axis
                joint->axis = stringToVector3(val);
                std::cout << "  JOINT: [" << entity_name << "] axis ["
                          << joint->axis.x << ", "
                          << joint->axis.y << ", "
                          << joint->axis.z
                          << "]\n";
              }
              else if (key == "type")
              {
                // assign type
                if (val == "planar")
                  joint->type = urdf::Joint::PLANAR;
                else if (val == "floating")
                  joint->type = urdf::Joint::FLOATING;
                else if (val == "revolute")
                  joint->type = urdf::Joint::REVOLUTE;
                else if (val == "continuous")
                  joint->type = urdf::Joint::CONTINUOUS;
                else if (val == "prismatic")
                  joint->type = urdf::Joint::PRISMATIC;
                else if (val == "fixed")
                  joint->type = urdf::Joint::FIXED;
                else
                {
                  printf("Joint [%s] has no known type [%s]",
                    joint->name.c_str(), val.c_str());
                  return false;
                }
                std::cout << "    JOINT: is of type [" << val << "] "
                          << "enum [" << joint->type << "]\n";
              }
              else if (key == "kin_min")
              {
                joint->limits->lower = boost::lexical_cast<double>(val);
                std::cout << "    JOINT limit lower [" << val << "]\n";
              }
              else if (key == "kin_max")
              {
                joint->limits->upper = boost::lexical_cast<double>(val);
                std::cout << "    JOINT limit upper [" << val << "]\n";
              }
              else if (key == "vel_min")
              {
                std::cout << "    URDF assumes symmetric velocity limits,"
                          << " vel_min ignored\n";
              }
              else if (key == "vel_max")
              {
                joint->limits->velocity = boost::lexical_cast<double>(val);
              }
              else if (key == "f_min")
              {
                std::cout << "    URDF assumes symmetric effort limits,"
                          << " f_min ignored\n";
              }
              else if (key == "f_max")
              {
                joint->limits->effort = boost::lexical_cast<double>(val);
              }
            }
            else
              std::cout << "    JOINT: key [" << key << "] has empty value\n";

            // add safety_controllers
            joint->safety->soft_upper_limit = joint->limits->upper+10.0;
            joint->safety->soft_lower_limit = joint->limits->lower-10.0;
            joint->safety->k_position = 100.0;
            joint->safety->k_velocity = 100.0;

            // add dynamic damping
            joint->dynamics->damping = 0.1;
          }
        }
      }
      std::map<std::string, std::string> parent_link_tree;
      parent_link_tree.clear();
      model->initTree(parent_link_tree);
      model->initRoot(parent_link_tree);
      printTree(model->getRoot());

#if USE_ROS
      // install the urdf in my own package at the right place
      // for the robot/*.xacro
      std::string package_name("drcsim_gazebo");
      std::string package_path = ros::package::getPath(package_name);
#else
      std::string package_path(".");
#endif
      TiXmlDocument *model_xml = urdf::exportURDF(model);
      model_xml->SaveFile(package_path + "/" + std::string("atlas.urdf"));
    }
  }
  return 0;
}
