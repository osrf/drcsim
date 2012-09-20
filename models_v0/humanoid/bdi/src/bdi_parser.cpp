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
      model->name_ = findNextKeyword(ifs, model, "struct ");
      std::cout << "model name [" << model->name_ << "]\n";


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


      // Next, read groups of struct, followed by key value pairs
      //std::pair<std::string, std::string> result;
      std::string struct_tok("struct ");
      std::string delim_tok("=");
      int struct_level = 0;
      std::map<int, std::string> struct_name;
      std::string entity_name;
      while (ifs.good())
      {
        //result = findNextStructOrKeyValuePair(ifs, model, "struct ", "=");

        // read a line
        std::string line;
        std::getline(ifs,line);
        
        std::string joint_namespace;
        // find first keyword "struct "
        if (line.find(struct_tok) != std::string::npos)
        {
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
            // init a link
            std::cout << "struct level [" << struct_level << "] "
                      << "struct name [" << struct_name[struct_level] << "]\n";
            boost::shared_ptr<urdf::Link> link;
            link.reset(new urdf::Link);
            link->name = struct_name[struct_level];
            model->links_.insert(std::make_pair(link->name, link));
            entity_name = struct_name[struct_level];
          }
          else
          {
            // init a joint namespace
            // infor for link
            std::string full_struct_name = struct_name[1];
            for (int i = 2; i <= struct_level; ++i)
              full_struct_name = full_struct_name + "." + struct_name[i];
            std::cout << "struct level [" << struct_level << "] ";
            std::cout << "current struct name [" << full_struct_name << "]\n";
            boost::shared_ptr<urdf::Joint> joint;
            joint.reset(new urdf::Joint);
            joint->name = full_struct_name;
            model->joints_.insert(std::make_pair(joint->name, joint));
            entity_name = full_struct_name;
          }
        }
        else if (line.find("=") != std::string::npos)
        {
          if (struct_level == 0)
          {
            // should not be here, should have found a struct first
          }
          else if (struct_level == 1)
          {
            // infor for link

            // parse key value pair
            std::string key, val;
            if (!line.empty())
            {
              boost::trim(line);
              size_t pos1 = line.find(delim_tok) + delim_tok.size();
              val = line.substr(pos1, line.size()-pos1);
              boost::trim(val);
              key = line.substr(0, pos1-1);
              boost::trim(key);
            }
            std::cout << "    LINK: key [" << key << "] "
                      << "val [" << val << "]\n";

            // add key value pair to link
            boost::shared_ptr<urdf::Link> link = model->links_.find(entity_name)->second;
            
          }
          else if (struct_level > 1)
          {
            // parse key value pair
            std::string key, val;
            if (!line.empty())
            {
              boost::trim(line);
              size_t pos1 = line.find(delim_tok) + delim_tok.size();
              val = line.substr(pos1, line.size()-pos1);
              boost::trim(val);
              key = line.substr(0, pos1-1);
              boost::trim(key);
            }
            std::cout << "    JOINT: key [" << key << "] "
                      << "val [" << val << "]\n";

            // add key value pair to join
            boost::shared_ptr<urdf::Joint> joint = model->joints_.find(entity_name)->second;
          }
        }

      }


    }
  }
  return 0;
}
