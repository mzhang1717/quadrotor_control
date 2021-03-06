//#line 2 "/opt/ros/hydro/share/dynamic_reconfigure/templates/ConfigType.h.template"
// *********************************************************
// 
// File autogenerated for the quadrotor_input package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

/***********************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************/

// Author: Blaise Gassend


#ifndef __quadrotor_input__ALTITUDECONTROLLERCONFIG_H__
#define __quadrotor_input__ALTITUDECONTROLLERCONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace quadrotor_input
{
  class AltitudeControllerConfigStatics;
  
  class AltitudeControllerConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(AltitudeControllerConfig &config, const AltitudeControllerConfig &max, const AltitudeControllerConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const AltitudeControllerConfig &config1, const AltitudeControllerConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, AltitudeControllerConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const AltitudeControllerConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, AltitudeControllerConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const AltitudeControllerConfig &config) const = 0;
      virtual void getValue(const AltitudeControllerConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T AltitudeControllerConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (AltitudeControllerConfig::* field);

      virtual void clamp(AltitudeControllerConfig &config, const AltitudeControllerConfig &max, const AltitudeControllerConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const AltitudeControllerConfig &config1, const AltitudeControllerConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, AltitudeControllerConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const AltitudeControllerConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, AltitudeControllerConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const AltitudeControllerConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const AltitudeControllerConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, AltitudeControllerConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    template<class T, class PT>
    class GroupDescription : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string name, std::string type, int parent, int id, bool s, T PT::* f) : AbstractGroupDescription(name, type, parent, id, s), field(f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, AltitudeControllerConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<AltitudeControllerConfig::AbstractGroupDescriptionConstPtr> groups;
    };
    
class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(AltitudeControllerConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("z_p"==(*_i)->name){z_p = boost::any_cast<double>(val);}
        if("z_i"==(*_i)->name){z_i = boost::any_cast<double>(val);}
        if("z_d"==(*_i)->name){z_d = boost::any_cast<double>(val);}
        if("z_i_min"==(*_i)->name){z_i_min = boost::any_cast<double>(val);}
        if("z_i_max"==(*_i)->name){z_i_max = boost::any_cast<double>(val);}
        if("z_des"==(*_i)->name){z_des = boost::any_cast<double>(val);}
        if("base_throttle_start"==(*_i)->name){base_throttle_start = boost::any_cast<double>(val);}
        if("voltage_start"==(*_i)->name){voltage_start = boost::any_cast<double>(val);}
        if("base_throttle_end"==(*_i)->name){base_throttle_end = boost::any_cast<double>(val);}
        if("voltage_end"==(*_i)->name){voltage_end = boost::any_cast<double>(val);}
        if("mass"==(*_i)->name){mass = boost::any_cast<double>(val);}
        if("pid_limit"==(*_i)->name){pid_limit = boost::any_cast<double>(val);}
        if("cmd_max"==(*_i)->name){cmd_max = boost::any_cast<double>(val);}
        if("cmd_min"==(*_i)->name){cmd_min = boost::any_cast<double>(val);}
      }
    }

    double z_p;
double z_i;
double z_d;
double z_i_min;
double z_i_max;
double z_des;
double base_throttle_start;
double voltage_start;
double base_throttle_end;
double voltage_end;
double mass;
double pid_limit;
double cmd_max;
double cmd_min;

    bool state;
    std::string name;

    
}groups;



//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double z_p;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double z_i;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double z_d;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double z_i_min;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double z_i_max;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double z_des;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double base_throttle_start;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double voltage_start;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double base_throttle_end;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double voltage_end;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double mass;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double pid_limit;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double cmd_max;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double cmd_min;
//#line 255 "/opt/ros/hydro/share/dynamic_reconfigure/templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("AltitudeControllerConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const AltitudeControllerConfig &__max__ = __getMax__();
      const AltitudeControllerConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const AltitudeControllerConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const AltitudeControllerConfig &__getDefault__();
    static const AltitudeControllerConfig &__getMax__();
    static const AltitudeControllerConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();
    
  private:
    static const AltitudeControllerConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void AltitudeControllerConfig::ParamDescription<std::string>::clamp(AltitudeControllerConfig &config, const AltitudeControllerConfig &max, const AltitudeControllerConfig &min) const
  {
    return;
  }

  class AltitudeControllerConfigStatics
  {
    friend class AltitudeControllerConfig;
    
    AltitudeControllerConfigStatics()
    {
AltitudeControllerConfig::GroupDescription<AltitudeControllerConfig::DEFAULT, AltitudeControllerConfig> Default("Default", "", 0, 0, true, &AltitudeControllerConfig::groups);
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.z_p = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.z_p = 1000.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.z_p = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_p", "double", 0, "Z Pos Proportional Gain", "", &AltitudeControllerConfig::z_p)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_p", "double", 0, "Z Pos Proportional Gain", "", &AltitudeControllerConfig::z_p)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.z_i = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.z_i = 100.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.z_i = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_i", "double", 0, "Z Pos Integral Gain", "", &AltitudeControllerConfig::z_i)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_i", "double", 0, "Z Pos Integral Gain", "", &AltitudeControllerConfig::z_i)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.z_d = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.z_d = 100.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.z_d = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_d", "double", 0, "Z Pos Derivative Gain", "", &AltitudeControllerConfig::z_d)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_d", "double", 0, "Z Pos Derivative Gain", "", &AltitudeControllerConfig::z_d)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.z_i_min = -1000.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.z_i_min = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.z_i_min = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_i_min", "double", 0, "Z Pos Integral Min Limit", "", &AltitudeControllerConfig::z_i_min)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_i_min", "double", 0, "Z Pos Integral Min Limit", "", &AltitudeControllerConfig::z_i_min)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.z_i_max = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.z_i_max = 1000.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.z_i_max = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_i_max", "double", 0, "Z Pos Integral Max Limit", "", &AltitudeControllerConfig::z_i_max)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_i_max", "double", 0, "Z Pos Integral Max Limit", "", &AltitudeControllerConfig::z_i_max)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.z_des = -2.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.z_des = 10.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.z_des = 0.2;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_des", "double", 0, "Z position", "", &AltitudeControllerConfig::z_des)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("z_des", "double", 0, "Z position", "", &AltitudeControllerConfig::z_des)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.base_throttle_start = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.base_throttle_start = 1.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.base_throttle_start = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("base_throttle_start", "double", 0, "Base Throttle Start", "", &AltitudeControllerConfig::base_throttle_start)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("base_throttle_start", "double", 0, "Base Throttle Start", "", &AltitudeControllerConfig::base_throttle_start)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.voltage_start = 10.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.voltage_start = 20.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.voltage_start = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("voltage_start", "double", 0, "Voltage Start", "", &AltitudeControllerConfig::voltage_start)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("voltage_start", "double", 0, "Voltage Start", "", &AltitudeControllerConfig::voltage_start)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.base_throttle_end = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.base_throttle_end = 1.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.base_throttle_end = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("base_throttle_end", "double", 0, "Base Throttle End", "", &AltitudeControllerConfig::base_throttle_end)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("base_throttle_end", "double", 0, "Base Throttle End", "", &AltitudeControllerConfig::base_throttle_end)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.voltage_end = 10.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.voltage_end = 20.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.voltage_end = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("voltage_end", "double", 0, "Voltage End", "", &AltitudeControllerConfig::voltage_end)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("voltage_end", "double", 0, "Voltage End", "", &AltitudeControllerConfig::voltage_end)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.mass = 0.1;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.mass = 10.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.mass = 2.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("mass", "double", 0, "Aircraft Mass", "", &AltitudeControllerConfig::mass)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("mass", "double", 0, "Aircraft Mass", "", &AltitudeControllerConfig::mass)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.pid_limit = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.pid_limit = 1.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.pid_limit = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("pid_limit", "double", 0, "Limit of PID Command (abs)", "", &AltitudeControllerConfig::pid_limit)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("pid_limit", "double", 0, "Limit of PID Command (abs)", "", &AltitudeControllerConfig::pid_limit)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.cmd_max = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.cmd_max = 1.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.cmd_max = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("cmd_max", "double", 0, "Maximum overall throttle", "", &AltitudeControllerConfig::cmd_max)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("cmd_max", "double", 0, "Maximum overall throttle", "", &AltitudeControllerConfig::cmd_max)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.cmd_min = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.cmd_min = 1.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.cmd_min = 0.0;
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("cmd_min", "double", 0, "Minimum overall throttle", "", &AltitudeControllerConfig::cmd_min)));
//#line 259 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(AltitudeControllerConfig::AbstractParamDescriptionConstPtr(new AltitudeControllerConfig::ParamDescription<double>("cmd_min", "double", 0, "Minimum overall throttle", "", &AltitudeControllerConfig::cmd_min)));
//#line 233 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.convertParams();
//#line 233 "/opt/ros/hydro/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __group_descriptions__.push_back(AltitudeControllerConfig::AbstractGroupDescriptionConstPtr(new AltitudeControllerConfig::GroupDescription<AltitudeControllerConfig::DEFAULT, AltitudeControllerConfig>(Default)));
//#line 390 "/opt/ros/hydro/share/dynamic_reconfigure/templates/ConfigType.h.template"

      for (std::vector<AltitudeControllerConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__); 
    }
    std::vector<AltitudeControllerConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<AltitudeControllerConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    AltitudeControllerConfig __max__;
    AltitudeControllerConfig __min__;
    AltitudeControllerConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const AltitudeControllerConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static AltitudeControllerConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &AltitudeControllerConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const AltitudeControllerConfig &AltitudeControllerConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const AltitudeControllerConfig &AltitudeControllerConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const AltitudeControllerConfig &AltitudeControllerConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<AltitudeControllerConfig::AbstractParamDescriptionConstPtr> &AltitudeControllerConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<AltitudeControllerConfig::AbstractGroupDescriptionConstPtr> &AltitudeControllerConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const AltitudeControllerConfigStatics *AltitudeControllerConfig::__get_statics__()
  {
    const static AltitudeControllerConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = AltitudeControllerConfigStatics::get_instance();
    
    return statics;
  }


}

#endif // __ALTITUDECONTROLLERRECONFIGURATOR_H__
