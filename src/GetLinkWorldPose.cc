/*
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "GetLinkWorldPose.hh"

namespace gazebo
{

    GZ_REGISTER_MODEL_PLUGIN ( GetLinkWorldPose )

    GetLinkWorldPose::GetLinkWorldPose() {}

    GetLinkWorldPose::~GetLinkWorldPose()
    {
        m_StreamThread.stop();
        this->m_updateConnection.reset();
    }

    void GetLinkWorldPose::UpdateChild()
    {
        // Reading apply wrench command
        yarp::os::Bottle tmpBottle;

        // Copying command
        this->m_lock.lock();
        tmpBottle = this->m_StreamThread.getCmd();   // get the name of the link from here

        this->m_lock.unlock();

        // initialCmdBottle will contain the initial bottle in StreamServerThread
        // Parsing command
        this->m_linkName = tmpBottle.get ( 0 ).asString();

        std::string fullScopeLinkName = "";

        if(this->m_subscope!="") 
        {
          fullScopeLinkName = std::string ( this->m_modelScope + "::" + this->m_subscope + "::" + this->m_linkName );
          this->m_onLink  = m_myModel->GetLink ( fullScopeLinkName );
        } 
        else 
        {
          this->m_onLink  = m_myModel->GetLink ( this->m_linkName );
        }

        if ( !this->m_onLink ) {
            //yError() << "GetLinkPOse plugin: link named " << this->m_linkName<< " not found";
            return;
        }

        if (true) // 
        {
            
            gazebo::math::Pose link_pose;
            

            #if GAZEBO_MAJOR_VERSION >= 8

                    link_pose = this->m_onLink->WorldPose();
                    // get the position
                    m_StreamThread.pose_value[0] = link_pose.Pos()[0];
                    m_StreamThread.pose_value[1] = link_pose.Pos()[1];
                    m_StreamThread.pose_value[2] = link_pose.Pos()[2];
                    // get the orientation
                    m_StreamThread.pose_value[3] = link_pose.Rot().Roll();
                    m_StreamThread.pose_value[4] = link_pose.Rot().Pitch();
                    m_StreamThread.pose_value[5] = link_pose.Rot().Yaw();

            #else
                    
                    link_pose = this->m_onLink->GetWorldPose();
                    // get the position
                    m_StreamThread.pose_value[0] = link_pose.pos[0];
                    m_StreamThread.pose_value[1] = link_pose.pos[1];
                    m_StreamThread.pose_value[2] = link_pose.pos[2];
                    // get the orientation
                    m_StreamThread.pose_value[3] = link_pose.rot.GetRoll();
                    m_StreamThread.pose_value[4] = link_pose.rot.GetPitch();
                    m_StreamThread.pose_value[5] = link_pose.rot.GetYaw();

            #endif
        }

    }

    void GetLinkWorldPose::Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
    {
        // Check if yarp network is active;
        if ( !this->m_yarpNet.checkNetwork() ) {
            yError ( "ERROR Yarp Network was not found active in GetLinkWorldPose plugin" );
            return;
        }

        // What is the parent name??
        this->m_modelScope = _model->GetScopedName();

        // Copy the pointer to the model to access later from UpdateChild
        this->m_myModel = _model;

        bool configuration_loaded = false;

        // Read robot name
        if ( _sdf->HasElement ( "robotNamefromConfigFile" ) ) {
            std::string iniRobotName      = _sdf->Get<std::string> ( "robotNamefromConfigFile" );
            std::string iniRobotNamePath =  gazebo::common::SystemPaths::Instance()->FindFileURI ( iniRobotName );

            if ( iniRobotNamePath != "" && this->m_iniParams.fromConfigFile ( iniRobotNamePath.c_str() ) ) {
                yarp::os::Value robotNameParam = m_iniParams.find ( "gazeboYarpPluginsRobotName" );
                this->robotName = robotNameParam.asString();
                m_StreamThread.setRobotName  ( robotName );
                m_StreamThread.setScopedName ( this->m_modelScope );

                gazebo::physics::Link_V links = _model->GetLinks();

                std::string defaultLink = links[0]->GetName();
                m_StreamThread.setDefaultLink(defaultLink);

    	        this->m_subscope = retrieveSubscope(links, m_modelScope);

                configuration_loaded = true;

            } else {
                yError ( "ERROR trying to get robot configuration file" );
                return;
            }
        } else {
                this->robotName = _model->GetName();
                m_StreamThread.setRobotName ( robotName );
                m_StreamThread.setScopedName ( this->m_modelScope );
                gazebo::physics::Link_V links = _model->GetLinks();
                m_StreamThread.setDefaultLink(links.at(0)->GetName());
                configuration_loaded = true;
        }

        // Starting thread to stream the link World pose
        if ( !m_StreamThread.start() ) {
            yError ( "ERROR: m_StreamThread did not start correctly" );
        }

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &GetLinkWorldPose::UpdateChild, this ) );
    }

    std::string GetLinkWorldPose::retrieveSubscope ( gazebo::physics::Link_V& v , std::string scope)
    {
        std::string tmpName = v[0]->GetName();
        std::size_t found = tmpName.find_first_of(":");
        if(found!=std::string::npos)
          tmpName = tmpName.substr (0, found);
        else
          tmpName = "";
        return tmpName;
    }

}

// ############ StreamServerThread class ###############

void StreamServerThread::setRobotName ( std::string robotName )
{
    this->m_robotName = robotName;
}

void StreamServerThread::setScopedName ( std::string scopedName )
{
    this->m_scopedName = scopedName;
}

void StreamServerThread::setDefaultLink(const std::string &defaultLink)
{
    this->m_defaultLink = defaultLink;
}


bool StreamServerThread::threadInit()
{

    // --------------------------------------------------------

    int argc;
    char *argv[3];
    // creation of a ressource finder object
    yarp::os::ResourceFinder rf;
    //
    rf.setVerbose(true);                                            //logs searched directories
    rf.setDefaultConfigFile("../config/GetLinkWorldPose.ini");      //default config file name.
    rf.configure(argc, argv);

    // get the link name
    std::string linkName = "";

    if(this->m_defaultLink =="")
    {
        linkName = rf.find("link").asString();
        if(linkName == ""){
            linkName = "root_link";
        }
    }
    else{
        linkName = rf.find("link").asString();
        if(linkName == ""){
            linkName = this->m_defaultLink;
        }
    }

    // opening the port 
    std::string linkPosePortName = "/";
    linkPosePortName += m_robotName;
    linkPosePortName += "/get_";
    linkPosePortName += linkName;
    linkPosePortName += "_WorldPose:o";
    
    // openning the port
    if ( !m_StreamPort.open(linkPosePortName.c_str()) ) {
        yError ( "ERROR opening Stream port /getLinkWorldPose" );
        return false;
    }
    // 
    m_cmd.addString ( linkName );

    pose_value.resize(6, 0);
    

    return true;
}


void StreamServerThread::run()
{
    
    while(!isStopping())
    {
        // preparing the port for new values
        yarp::sig::Vector &output_linkPose = m_StreamPort.prepare();

        // clear previous values
        output_linkPose.clear();

        // passing values to be streamed
        m_lock.lock();
        output_linkPose = this->pose_value;
        m_lock.unlock();
        // write the values to the port
        m_StreamPort.write();

    }

}


void StreamServerThread::threadRelease()
{
    yarp::os::Thread::threadRelease();
    // closing the port
    m_StreamPort.close();

}

yarp::os::Bottle StreamServerThread::getCmd()
{
    return m_cmd;
}

void StreamServerThread::onStop()
{
    m_StreamPort.interrupt();
}

