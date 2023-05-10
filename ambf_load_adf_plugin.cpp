//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <henry.phalen@jhu.edu>
    \author    Henry Phalen
*/
//==============================================================================

#include <afFramework.h>
#include "adf_loader_interface.h"
#include <ros/ros.h>
#include <ambf_server/RosComBase.h>
#include <ambf_load_adf_plugin/SendString.h>

using namespace ambf;

// TODO: allow passing in a list of bodies, for multiple traces
// TODO: allow specification of color, line_width in args

class afTracePlugin : public afSimulatorPlugin
{
    int init(int argc, char **argv, const afWorldPtr a_afWorld)
    {
        m_worldPtr = a_afWorld;
        m_rosNode = afROSNode::getNode();
        std::string a_namespace = "ambf";
        std::string a_plugin = "load_adf_plugin";
        m_add_adf_file_service = m_rosNode->advertiseService(a_namespace + "/" + a_plugin + "/add_adf_file", &afTracePlugin::add_adf_file, this);
        g_adfLoaderPtr = new ADFLoaderInterface();
        return 1;
    }

    virtual void keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods) {}
    virtual void graphicsUpdate() {}
    virtual void mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes) {}
    virtual void mousePosUpdate(GLFWwindow *a_window, double x_pos, double y_pos) {}
    virtual void mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos) {}
    virtual void physicsUpdate(const afWorldPtr a_afWorld) {}
    virtual void reset() {}
    virtual bool close() { return 0; }

private:
    ADFLoaderInterface* g_adfLoaderPtr;

    // for now, to keep simple, single request is single file
    bool add_adf_file(ambf_load_adf_plugin::SendString::Request &req, ambf_load_adf_plugin::SendString::Response &res)
    {
        std::string adf_file = req.data;
        std::cout << "RECIEVED REQUEST TO LOAD: " << adf_file << std::endl;
        // find the last "." in the string (if any)
        size_t findIdx = adf_file.find_last_of(".");
        if (findIdx == std::string::npos) // no extension, must be a directory
        {
            res.resp = "INVALID; APPEARS TO BE A DIRECTORY \"" + adf_file + "\". PLEASE ONLY SEND FILES \n";
            std::cerr << res.resp << std::endl;
            res.success = false;
            return false;
        }

        // if extension is not .yaml or .ambf, return error
        auto extension = adf_file.substr(findIdx);
        if(extension.compare(".yaml") != 0 && extension.compare(".YAML") != 0 && extension.compare(".ambf") != 0 && extension.compare(".AMBF") != 0)
        {
            res.resp = "INVALID EXTENSION: \"" + adf_file + "\". ONLY \".AMBF\" OR \".YAML\" SUPPORTED \n";
            std::cerr << res.resp << std::endl;
            res.success = false;
            return false;
        }

        // now that we've done all the tests, we can load the file
        afModelAttributes modelAttribs;
        if (g_adfLoaderPtr->loadModelAttribs(adf_file, &modelAttribs))
        {
            afModelPtr model = new afModel(m_worldPtr);
            if (model->createFromAttribs(&modelAttribs))
            {
                m_worldPtr->pausePhysics(true);
                m_worldPtr->addModel(model);
                m_worldPtr->pausePhysics(false);
            }
            else
            {
                delete model;
            }
        }
        else
        {
            res.resp = "ERROR LOADING FILE: \"" + adf_file + "\". PLEASE CHECK FILEPATH AND TRY AGAIN \n";
            std::cerr << res.resp << std::endl;
            res.success = false;
            return false;
        }

        res.resp = "SUCCESSFULLY LOADED FILE: \"" + adf_file + "\"";
        std::cout << res.resp << std::endl;
        res.success = true;
        return true;
    }

    ros::NodeHandle *m_rosNode;
    ros::ServiceServer m_add_adf_file_service;
};

AF_REGISTER_SIMULATOR_PLUGIN(afTracePlugin)
