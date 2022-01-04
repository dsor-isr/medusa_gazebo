#include <string>
#include <uuv_sensor_ros_plugins/modemPlugin.hh>

using namespace gazebo;


modemPlugin::modemPlugin(): m_temperature(10.0), m_noiseMu(0),
                                        m_noiseSigma(1), m_soundSpeed(1500) {}
modemPlugin::~modemPlugin() {}
void modemPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    /*****************************  SDF PARAMETERS ****************************/

    // Ensure ROS is initialized for publishers and subscribers
    if (!ros::isInitialized())
    {
        gzerr << "ROS has not been inintialized\n";
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, "modem",
                  ros::init_options::NoSigintHandler);
        return;
    }

    /*------------------------------------------------------------------------*/
    // Grab namespace from SDF
    if (!_sdf->HasElement("namespace"))
    {
        gzerr << "Missing required parameter <namespace>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_namespace = _sdf->Get<std::string>("namespace");

    /*------------------------------------------------------------------------*/
    // Obtain modem device name from SDF
    if (!_sdf->HasElement("modem_device"))
    {
        gzerr << "Missing required parameter <modem_device>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_modemDevice = _sdf->Get<std::string>("modem_device");
    gzmsg << "modem device: " << this->m_modemDevice << std::endl;

    /*------------------------------------------------------------------------*/
    // get modem ID
    if (!_sdf->HasElement("modem_ID"))
    {
        gzerr << "Missing required parameter <modem_ID>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_modemID = _sdf->Get<std::string>("modem_ID");

    /*------------------------------------------------------------------------*/
    // get usbl ID
    if (!_sdf->HasElement("usbl_device"))
    {
        gzerr << "Missing required parameter <usbl_device>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_usblDevice = _sdf->Get<std::string>("usbl_device");

    /*------------------------------------------------------------------------*/
    // get usbl ID
    if (!_sdf->HasElement("usbl_ID"))
    {
        gzerr << "Missing required parameter <usbl_ID>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_usblID = _sdf->Get<std::string>("usbl_ID");

    /*------------------------------------------------------------------------*/
    // get the mean of normal distribution for the noise model
    if (_sdf->HasElement("mu"))
    {
        this->m_noiseMu = _sdf->Get<double>("mu");
    }

    /*------------------------------------------------------------------------*/
    // get the standard deviation of normal distribution for the noise model
    if (_sdf->HasElement("sigma"))
    {
        this->m_noiseSigma = _sdf->Get<double>("sigma");
    }

    // store this entity model
    this->m_model = _model;

    /*------------------------------------------------------------------------*/
    // Get object that modem attached to
    if (!_sdf->HasElement("modem_attached_object"))
    {
        gzerr << "Missing required parameter <modem_attached_object>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_modemAttachedObject = _sdf->Get<std::string>(
        "modem_attached_object");

    /*------------------------------------------------------------------------*/
    // Get object that usbl attached to
    if (!_sdf->HasElement("usbl_attached_object"))
    {
        gzerr << "Missing required parameter <usbl_attached_object>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_usblAttachedObject = _sdf->Get<std::string>(
        "usbl_attached_object");

    /*------------------------------------------------------------------------*/
    // Get time to wait
    if (!_sdf->HasElement("delay_until_answer"))
    {
        gzerr << "Missing required parameter <delay_until_answer>, "
              << "plugin will not be initialized." << std::endl;
        return;
    }
    this->m_delayUntilAnswer = _sdf->Get<std::string>(
        "delay_until_answer");

    /********************************************************************/

    /******************  ROS PUBLISHERS ************************/
    this->m_rosNode.reset(new ros::NodeHandle(this->m_modemDevice));

    std::string commandResponseTopic("/" + this->m_namespace + "/"
        + this->m_usblDevice + "_" + this->m_usblID
        + "/command_response");
    this->m_commandResponsePub = this->m_rosNode->advertise<std_msgs::String>(commandResponseTopic, 1);

    this->m_trigger_serialization = this->m_rosNode->advertise<std_msgs::Empty>("/"+this->m_modemAttachedObject+"/acomms/scheme/trigger_serialization", 1);
    
    this->m_payload_to_deserialize = this->m_rosNode->advertise<std_msgs::String>("/"+this->m_modemAttachedObject+"/acomms/scheme/payload_to_deserialize", 1);
    
    this->m_globalPosPub2 = this->m_rosNode->advertise<uuv_sensor_ros_plugins_msgs::modemLocation>("/" + this->m_namespace + "/" + this->m_usblDevice + "_" + this->m_modemID + "/global_position", 1);

    /*********************************************************************/

    /******************  ROS SUBSCRIBERS ********************/

    this->m_rosNode.reset(new ros::NodeHandle(this->m_modemDevice));

    ros::SubscribeOptions iis_ping =
        ros::SubscribeOptions::create<std_msgs::String>(
            "/" + this->m_namespace + "/" + this->m_modemDevice
            + "_" + this->m_modemID + "/individual_interrogation_ping",
            1,
            boost::bind(&modemPlugin::iisRosCallback, this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_iisSub = this->m_rosNode->subscribe(iis_ping);

    ros::SubscribeOptions cis_ping =
        ros::SubscribeOptions::create<std_msgs::String>(
            "/" + this->m_namespace + "/common_interrogation_ping",
            1,
            boost::bind(&modemPlugin::cisRosCallback, this, _1),
            ros::VoidPtr(), &this->m_rosQueue);

    this->m_cisSub = this->m_rosNode->subscribe(cis_ping);

    ros::SubscribeOptions payload_to_transmit = ros::SubscribeOptions::create<std_msgs::String>(
            "/"+this->m_modemAttachedObject+"/acomms/serializer/payload_to_transmit",
            1,
            boost::bind(&modemPlugin::payloadToTransmitCallback,
                        this, _1),
            ros::VoidPtr(), &this->m_rosQueue);
    
    this->m_payload_to_transmit = this->m_rosNode->subscribe(payload_to_transmit);
 
    /********************************************************************/

    /******************  ROS MISC ******************************/

    this->m_rosQueueThread = std::thread(std::bind(
        &modemPlugin::queueThread, this));
    gzmsg << "modem plugin loaded\n";
}

// receives ping from modem and call Send()
void modemPlugin::iisRosCallback(const std_msgs::StringConstPtr &msg)
{
    gzmsg << this->m_modemDevice+ "_" + this->m_modemID
        + ": Received iis_ping, responding\n";

    std_msgs::String aux_msg;
    aux_msg.data = msg->data + ":vehicle" + this->m_usblID;
    //deserialization
    this->m_payload_to_deserialize.publish(aux_msg); 

    //pub triggerSerialization
    std_msgs::Empty aux;
    this->m_trigger_serialization.publish(aux);
}

// receives ping from modem and call Send()
void modemPlugin::cisRosCallback(const std_msgs::StringConstPtr &msg)
{
    std_msgs::String aux_msg;
    aux_msg.data = msg->data + ":vehicle" + this->m_usblID;
    //deserialization
    this->m_payload_to_deserialize.publish(aux_msg);

    //pub triggerSerialization
    std_msgs::Empty aux;
    this->m_trigger_serialization.publish(aux);
}

void modemPlugin::payloadToTransmitCallback(const std_msgs::StringConstPtr &payload)
{
    auto box = this->m_model->GetWorld()->ModelByName(this->m_usblAttachedObject);
    double dist = (this->m_model->WorldPose().Pos()
                   - box->WorldPose().Pos()).Length();
    // delay the answer acording a parameter to garantee that the acoustic channel is not with messages from 2
    // different modems
    ros::Duration(std::stoi(this->m_delayUntilAnswer)).sleep();
    // simulate the delay of the acoustic channel
    ros::Duration(dist / this->m_soundSpeed).sleep();
    
    // randomly generate from normal distribution for noise
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d(this->m_noiseMu, this->m_noiseSigma);

    // Gazebo publishing modem's position with noise and delay
    auto curr_pose = this->m_model->WorldPose();
    ignition::math::Vector3<double> position = curr_pose.Pos();
    //auto pub_mPub->Publish(pub_msg);

    uuv_sensor_ros_plugins_msgs::modemLocation aux_modem_pos;

    aux_modem_pos.x = position.X() + d(gen);
    aux_modem_pos.y = position.Y() + d(gen);
    aux_modem_pos.z = position.Z() + d(gen);
    aux_modem_pos.modem_ID = this->m_modemID;
    aux_modem_pos.data = payload->data;

    this->m_globalPosPub2.publish(aux_modem_pos);
}

void modemPlugin::queueThread()
{
    static const double timeout = 0.01;
    while (this->m_rosNode->ok())
    {
        this->m_rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}
