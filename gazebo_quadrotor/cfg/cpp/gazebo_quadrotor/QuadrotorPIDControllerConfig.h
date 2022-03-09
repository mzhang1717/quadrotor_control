//#line 2 "/opt/ros/groovy/share/dynamic_reconfigure/templates/ConfigType.h.template"
// *********************************************************
// 
// File autogenerated for the gazebo_quadrotor package 
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


#ifndef __gazebo_quadrotor__QUADROTORPIDCONTROLLERCONFIG_H__
#define __gazebo_quadrotor__QUADROTORPIDCONTROLLERCONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace gazebo_quadrotor
{
  class QuadrotorPIDControllerConfigStatics;
  
  class QuadrotorPIDControllerConfig
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
      
      virtual void clamp(QuadrotorPIDControllerConfig &config, const QuadrotorPIDControllerConfig &max, const QuadrotorPIDControllerConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const QuadrotorPIDControllerConfig &config1, const QuadrotorPIDControllerConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, QuadrotorPIDControllerConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const QuadrotorPIDControllerConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, QuadrotorPIDControllerConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const QuadrotorPIDControllerConfig &config) const = 0;
      virtual void getValue(const QuadrotorPIDControllerConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T QuadrotorPIDControllerConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (QuadrotorPIDControllerConfig::* field);

      virtual void clamp(QuadrotorPIDControllerConfig &config, const QuadrotorPIDControllerConfig &max, const QuadrotorPIDControllerConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const QuadrotorPIDControllerConfig &config1, const QuadrotorPIDControllerConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, QuadrotorPIDControllerConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const QuadrotorPIDControllerConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, QuadrotorPIDControllerConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const QuadrotorPIDControllerConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const QuadrotorPIDControllerConfig &config, boost::any &val) const
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
      virtual void updateParams(boost::any &cfg, QuadrotorPIDControllerConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); i++)
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
        
        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); i++) 
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

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); i++)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }
      
      virtual void updateParams(boost::any &cfg, QuadrotorPIDControllerConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); i++) 
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); i++)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<QuadrotorPIDControllerConfig::AbstractGroupDescriptionConstPtr> groups;
    };
    
class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(QuadrotorPIDControllerConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = params.begin(); i != params.end(); i++)
      {
        boost::any val;
        (*i)->getValue(config, val);

        if("roll_p"==(*i)->name){roll_p = boost::any_cast<double>(val);}
        if("roll_i"==(*i)->name){roll_i = boost::any_cast<double>(val);}
        if("roll_d"==(*i)->name){roll_d = boost::any_cast<double>(val);}
        if("pitch_p"==(*i)->name){pitch_p = boost::any_cast<double>(val);}
        if("pitch_i"==(*i)->name){pitch_i = boost::any_cast<double>(val);}
        if("pitch_d"==(*i)->name){pitch_d = boost::any_cast<double>(val);}
        if("yaw_p"==(*i)->name){yaw_p = boost::any_cast<double>(val);}
        if("yaw_i"==(*i)->name){yaw_i = boost::any_cast<double>(val);}
        if("yaw_d"==(*i)->name){yaw_d = boost::any_cast<double>(val);}
      }
    }

    double roll_p;
double roll_i;
double roll_d;
double pitch_p;
double pitch_i;
double pitch_d;
double yaw_p;
double yaw_i;
double yaw_d;

    bool state;
    std::string name;

    
}groups;



//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double roll_p;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double roll_i;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double roll_d;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double pitch_p;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double pitch_i;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double pitch_d;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double yaw_p;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double yaw_i;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double yaw_d;
//#line 255 "/opt/ros/groovy/share/dynamic_reconfigure/templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
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
        ROS_ERROR("QuadrotorPIDControllerConfig::__fromMessage__ called with an unexpected parameter.");
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
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++)
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
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
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
      const QuadrotorPIDControllerConfig &__max__ = __getMax__();
      const QuadrotorPIDControllerConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const QuadrotorPIDControllerConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); i++)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const QuadrotorPIDControllerConfig &__getDefault__();
    static const QuadrotorPIDControllerConfig &__getMax__();
    static const QuadrotorPIDControllerConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();
    
  private:
    static const QuadrotorPIDControllerConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void QuadrotorPIDControllerConfig::ParamDescription<std::string>::clamp(QuadrotorPIDControllerConfig &config, const QuadrotorPIDControllerConfig &max, const QuadrotorPIDControllerConfig &min) const
  {
    return;
  }

  class QuadrotorPIDControllerConfigStatics
  {
    friend class QuadrotorPIDControllerConfig;
    
    QuadrotorPIDControllerConfigStatics()
    {
QuadrotorPIDControllerConfig::GroupDescription<QuadrotorPIDControllerConfig::DEFAULT, QuadrotorPIDControllerConfig> Default("Default", "", 0, 0, true, &QuadrotorPIDControllerConfig::groups);
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.roll_p = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.roll_p = 100.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.roll_p = 0.1;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("roll_p", "double", 0, "Roll Proportional Gain", "", &QuadrotorPIDControllerConfig::roll_p)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("roll_p", "double", 0, "Roll Proportional Gain", "", &QuadrotorPIDControllerConfig::roll_p)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.roll_i = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.roll_i = 100.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.roll_i = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("roll_i", "double", 0, "Roll Integral Gain", "", &QuadrotorPIDControllerConfig::roll_i)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("roll_i", "double", 0, "Roll Integral Gain", "", &QuadrotorPIDControllerConfig::roll_i)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.roll_d = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.roll_d = 100.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.roll_d = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("roll_d", "double", 0, "Roll Derivative Gain", "", &QuadrotorPIDControllerConfig::roll_d)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("roll_d", "double", 0, "Roll Derivative Gain", "", &QuadrotorPIDControllerConfig::roll_d)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.pitch_p = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.pitch_p = 100.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.pitch_p = 0.1;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("pitch_p", "double", 0, "Pitch Proportional Gain", "", &QuadrotorPIDControllerConfig::pitch_p)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("pitch_p", "double", 0, "Pitch Proportional Gain", "", &QuadrotorPIDControllerConfig::pitch_p)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.pitch_i = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.pitch_i = 100.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.pitch_i = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("pitch_i", "double", 0, "Pitch Integral Gain", "", &QuadrotorPIDControllerConfig::pitch_i)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("pitch_i", "double", 0, "Pitch Integral Gain", "", &QuadrotorPIDControllerConfig::pitch_i)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.pitch_d = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.pitch_d = 100.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.pitch_d = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("pitch_d", "double", 0, "Pitch Derivative Gain", "", &QuadrotorPIDControllerConfig::pitch_d)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("pitch_d", "double", 0, "Pitch Derivative Gain", "", &QuadrotorPIDControllerConfig::pitch_d)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.yaw_p = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.yaw_p = 100.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.yaw_p = 0.1;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("yaw_p", "double", 0, "Yaw Proportional Gain", "", &QuadrotorPIDControllerConfig::yaw_p)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("yaw_p", "double", 0, "Yaw Proportional Gain", "", &QuadrotorPIDControllerConfig::yaw_p)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.yaw_i = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.yaw_i = 100.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.yaw_i = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("yaw_i", "double", 0, "Yaw Integral Gain", "", &QuadrotorPIDControllerConfig::yaw_i)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("yaw_i", "double", 0, "Yaw Integral Gain", "", &QuadrotorPIDControllerConfig::yaw_i)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.yaw_d = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.yaw_d = 100.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.yaw_d = 0.0;
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("yaw_d", "double", 0, "Yaw Derivative Gain", "", &QuadrotorPIDControllerConfig::yaw_d)));
//#line 259 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr(new QuadrotorPIDControllerConfig::ParamDescription<double>("yaw_d", "double", 0, "Yaw Derivative Gain", "", &QuadrotorPIDControllerConfig::yaw_d)));
//#line 233 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.convertParams();
//#line 233 "/opt/ros/groovy/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __group_descriptions__.push_back(QuadrotorPIDControllerConfig::AbstractGroupDescriptionConstPtr(new QuadrotorPIDControllerConfig::GroupDescription<QuadrotorPIDControllerConfig::DEFAULT, QuadrotorPIDControllerConfig>(Default)));
//#line 390 "/opt/ros/groovy/share/dynamic_reconfigure/templates/ConfigType.h.template"
    
      for (std::vector<QuadrotorPIDControllerConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__); 
    }
    std::vector<QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<QuadrotorPIDControllerConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    QuadrotorPIDControllerConfig __max__;
    QuadrotorPIDControllerConfig __min__;
    QuadrotorPIDControllerConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const QuadrotorPIDControllerConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static QuadrotorPIDControllerConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &QuadrotorPIDControllerConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const QuadrotorPIDControllerConfig &QuadrotorPIDControllerConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const QuadrotorPIDControllerConfig &QuadrotorPIDControllerConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const QuadrotorPIDControllerConfig &QuadrotorPIDControllerConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<QuadrotorPIDControllerConfig::AbstractParamDescriptionConstPtr> &QuadrotorPIDControllerConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<QuadrotorPIDControllerConfig::AbstractGroupDescriptionConstPtr> &QuadrotorPIDControllerConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const QuadrotorPIDControllerConfigStatics *QuadrotorPIDControllerConfig::__get_statics__()
  {
    const static QuadrotorPIDControllerConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = QuadrotorPIDControllerConfigStatics::get_instance();
    
    return statics;
  }


}

#endif // __QUADROTORPIDCONTROLLERRECONFIGURATOR_H__
