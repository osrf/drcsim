#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <urdf_model/model.h>


enum StructType { LINK, JOINT };

std::string findNextKeyword(std::ifstream &ifs, boost::shared_ptr<urdf::ModelInterface> model, std::string struct_tok)
{
  std::string line;
  std::getline(ifs,line);
  // find first keyword "struct "
  while (ifs.good() && line.find(struct_tok) == std::string::npos)
    std::getline(ifs,line);
  if (!line.empty())
  {
    boost::trim(line);
    size_t pos1 = line.find(struct_tok) + struct_tok.size();
    return line.substr(pos1, line.size()-pos1);
  }
  return std::string();
}


std::pair<std::string, std::vector<std::string> > findNextKeyValuesPair(std::ifstream &ifs, boost::shared_ptr<urdf::ModelInterface> model, std::string delim_tok)
{
  std::string line;
  std::getline(ifs,line);
  // find first keyword "struct "
  while (ifs.good() && line.find(delim_tok) == std::string::npos)
    std::getline(ifs,line);
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
  return std::pair<std::string, std::vector<std::string> >(std::string(), std::vector<std::string>());
}

std::pair<std::string, std::string> findNextKeyValuePair(std::ifstream &ifs, boost::shared_ptr<urdf::ModelInterface> model, std::string delim_tok)
{
  std::string line;
  std::getline(ifs,line);
  // find first keyword "struct "
  while (ifs.good() && line.find(delim_tok) == std::string::npos)
    std::getline(ifs,line);
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

std::pair<std::string, std::string> findNextStructOrKeyValuePair(std::ifstream &ifs, boost::shared_ptr<urdf::ModelInterface> model, std::string struct_tok, std::string delim_tok)
{
  std::string line;
  std::getline(ifs,line);
  // find first keyword "struct "
  while (ifs.good() && line.find(delim_tok) == std::string::npos)
    std::getline(ifs,line);
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
  size_t pos = val.find("#",0);
  if (pos != std::string::npos)
    val = val.substr(0, pos - 1);
}

void printTree(boost::shared_ptr<const urdf::Link> link,int level = 0)
{
  level+=2;
  int count = 0;
  for (std::vector<boost::shared_ptr<urdf::Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
  {
    if (*child)
    {
      for(int j=0;j<level;j++) std::cout << "  "; //indent
      std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << std::endl;
      // first grandchild
      printTree(*child,level);
    }
    else
    {
      for(int j=0;j<level;j++) std::cout << " "; //indent
      std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
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
        std::getline(ifs,line);
        
        std::string joint_namespace;
        if (line.find(struct_tok) != std::string::npos)
        {
          // this is a struct
          // struct_level = number of spaces before keyword "struct"
          struct_level = line.find(struct_tok);
          if (!line.empty())
          {
            boost::trim(line);
            size_t pos1 = line.find(struct_tok) + struct_tok.size();
            struct_name[struct_level] = line.substr(pos1, line.size()-pos1);
          }

          if (struct_level == 1)
          {
            // potentially a link name, but it could be followed by another struct, then it is not a link
            std::cout << "struct level [" << struct_level << "] "
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


            // just checking, joint should have been created when reading link struct
            boost::shared_ptr<urdf::Joint> joint = model->joints_.find(entity_name)->second;
            if (!joint)
              std::cout << "     intermediate struct, not a joint name [" << entity_name << "].\n";
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
            boost::shared_ptr<urdf::Link> link = model->links_.find(entity_name)->second;
            if (!link)
            {
              std::cout << "  LINK: Creating [" << entity_name << "]\n";
              link.reset(new urdf::Link);
              link->name = entity_name;
              model->links_.insert(std::make_pair(link->name, link));
            }

            // parse key value pair
            std::string key, val;
            if (!line.empty())
            {
              boost::trim(line);
              size_t pos1 = line.find(delim_tok) + delim_tok.size();
              val = line.substr(pos1, line.size()-pos1);
              boost::trim(val);
              removeComments(val);
              key = line.substr(0, pos1-1);
              boost::trim(key);
            }
            std::cout << "    LINK: key [" << key << "] "
                      << "val [" << val << "]\n";


            if (key == "parent_link")
            {
              std::cout << "          parent link [" << val << "]\n";
              // add parent to child link, add child to parent link
              boost::shared_ptr<urdf::Link> parent = model->links_.find(val)->second;
              parent->child_links.push_back(link);
              link->setParent(parent);
            }
            else if (key == "parent_kin_dof")
            {
              std::cout << "          parent joint [" << val << "]\n";

              // create parent joint with name
              boost::shared_ptr<urdf::Joint> joint;
              std::cout << "  JOINT: Creating [" << entity_name << "]\n";
              joint.reset(new urdf::Joint);
              joint->name = val;
              model->joints_.insert(std::make_pair(joint->name, joint));

              // this is the parent joint for link
              link->parent_joint = joint;
              joint->child_link_name = link->name;
              joint->parent_link_name = link->getParent()->name;
            }
          }
          else
          {
            // parse key value pair
            std::string key, val;
            if (!line.empty())
            {
              boost::trim(line);
              size_t pos1 = line.find(delim_tok) + delim_tok.size();
              val = line.substr(pos1, line.size()-pos1);
              boost::trim(val);
              removeComments(val);
              key = line.substr(0, pos1-1);
              boost::trim(key);
            }
            std::cout << "    JOINT: key [" << key << "] "
                      << "val [" << val << "]\n";

            // add key value pair to join
            boost::shared_ptr<urdf::Joint> joint = model->joints_.find(entity_name)->second;
            // std::cout << joint->name << " has "
            //           << " parent " << joint->parent_link_name
            //           << " child " << joint->child_link_name << "\n";
          }
        }

      }
      std::map<std::string, std::string> parent_link_tree;
      parent_link_tree.clear();
      model->initTree(parent_link_tree);
      model->initRoot(parent_link_tree);
      printTree(model->getRoot());

    }
  }
  return 0;
}
