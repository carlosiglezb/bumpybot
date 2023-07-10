 			//
// Created by Carlos on 11/1/21.
//

#include <bumpybot_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <math.h>

#define STUCK_IN()
#define STUCK_OUT()
#define STUCK_WHILE(x) STUCK_IN() while(x)
#define EC_TIMEOUTMON 500

ec_ODlistt ODlist;
ec_OElistt OElist;

char* dtype2string(uint16 dtype, uint16 bitlen)
{
    static char str[32] = { 0 };

    switch(dtype)
    {
        case ECT_BOOLEAN:
            sprintf(str, "BOOLEAN");
            break;
        case ECT_INTEGER8:
            sprintf(str, "INTEGER8");
            break;
        case ECT_INTEGER16:
            sprintf(str, "INTEGER16");
            break;
        case ECT_INTEGER32:
            sprintf(str, "INTEGER32");
            break;
        case ECT_INTEGER24:
            sprintf(str, "INTEGER24");
            break;
        case ECT_INTEGER64:
            sprintf(str, "INTEGER64");
            break;
        case ECT_UNSIGNED8:
            sprintf(str, "UNSIGNED8");
            break;
        case ECT_UNSIGNED16:
            sprintf(str, "UNSIGNED16");
            break;
        case ECT_UNSIGNED32:
            sprintf(str, "UNSIGNED32");
            break;
        case ECT_UNSIGNED24:
            sprintf(str, "UNSIGNED24");
            break;
        case ECT_UNSIGNED64:
            sprintf(str, "UNSIGNED64");
            break;
        case ECT_REAL32:
            sprintf(str, "REAL32");
            break;
        case ECT_REAL64:
            sprintf(str, "REAL64");
            break;
        case ECT_BIT1:
            sprintf(str, "BIT1");
            break;
        case ECT_BIT2:
            sprintf(str, "BIT2");
            break;
        case ECT_BIT3:
            sprintf(str, "BIT3");
            break;
        case ECT_BIT4:
            sprintf(str, "BIT4");
            break;
        case ECT_BIT5:
            sprintf(str, "BIT5");
            break;
        case ECT_BIT6:
            sprintf(str, "BIT6");
            break;
        case ECT_BIT7:
            sprintf(str, "BIT7");
            break;
        case ECT_BIT8:
            sprintf(str, "BIT8");
            break;
        case ECT_VISIBLE_STRING:
            sprintf(str, "VISIBLE_STR(%d)", bitlen);
            break;
        case ECT_OCTET_STRING:
            sprintf(str, "OCTET_STR(%d)", bitlen);
            break;
        default:
            sprintf(str, "dt:0x%4.4X (%d)", dtype, bitlen);
    }
    return str;
}


namespace bumpybot_hw
{
  BumpybotHWInterface::BumpybotHWInterface(ros::NodeHandle &nh, urdf::Model* urdf_model)
    : name_("hardware_interface"), nh_(nh)
  {
    ROS_INFO("Initializing BumpybotHWInterface");

    // Check if the URDF model needs to be loaded
    if (urdf_model == NULL)
      loadURDF(nh, "robot_description");
    else
      urdf_model_ = urdf_model;

    // Load rosparams
    ros::NodeHandle rpnh(nh_, "trikey/"+name_);
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
    error += !rosparam_shortcuts::get(name_, rpnh, "interface_name", ifname_);
    error += !rosparam_shortcuts::get(name_, rpnh, "wheel_encoder_counts", wheel_encoder_counts_);
    error += !rosparam_shortcuts::get(name_, rpnh, "wheel_encoder_sign", wheel_encoder_sign_);
    rosparam_shortcuts::shutdownIfError(name_, error);

    // setup publisher/subscriber for motor states/commands
    js_pub = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    temp_pub = nh_.advertise<sensor_msgs::JointState>("servo_temp", 10);
//    temp_pub = nh_.advertise<bumpybot_hw_interface::ServoTemp>("servo_temp", 10);
    js1_sub = nh_.subscribe("/wheel1_commands", 100, &BumpybotHWInterface::cmd1Callback, this);
    js2_sub = nh_.subscribe("/wheel2_commands", 100, &BumpybotHWInterface::cmd2Callback, this);
    js3_sub = nh_.subscribe("/wheel3_commands", 100, &BumpybotHWInterface::cmd3Callback, this);

    // ROS Services
    clear_faults_srv = nh_.advertiseService("/clear_faults", &BumpybotHWInterface::clearFaults, this);

    ROS_INFO("Finished constructing Bumpybot Hardware Interface");
}

  BumpybotHWInterface::~BumpybotHWInterface()
  {
    ros::Duration delay(0.1);
    double start = ros::Time::now().toSec();
    while (!stopOperation()) {
      delay.sleep();
      double wait_time = ros::Time::now().toSec() - start;
      if(wait_time > 5.0)
      {
        ROS_INFO("Everest did not reach shut down state after waiting 5 seconds.");
        break;
      }
    }

    /* request INIT state for all slaves */
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_close();

//    ecx_writestate(&ecx_context, 0);
//    ecx_close(&ecx_context);
    ROS_WARN("EtherCAT HardWare Interface Shutdown");
  }

    bool BumpybotHWInterface::stopOperation()
    {
      // Note: Make sure to change this if adding non-Everest EtherCAT devices!
      // set current back to zero
      *(ec_slave[0].outputs + 3) = 0;
      *(ec_slave[0].outputs + 4) = 0;
      *(ec_slave[0].outputs + 5) = 0;
      *(ec_slave[0].outputs + 6) = 0;

      // disable servo operation
      *(ec_slave[0].outputs + 0) = 0x7;
      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);

      // shutdown servo
      *(ec_slave[0].outputs + 0) = 0x6;
      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);

      // check that the servo shut down
      if(*(ec_slave[0].inputs + 0) == 0x06){
        inOP = false;
        ROS_INFO("Everest shut down successfully.");
        return true;
      } else {
        return false;
      }
    }


    bool BumpybotHWInterface::init()
  {
    num_joints_ = joint_names_.size();

    // Status
    joint_position_.resize(num_joints_, 0.0);
    joint_velocity_.resize(num_joints_, 0.0);
    joint_effort_.resize(num_joints_, 0.0);
    js_msg.name.resize(num_joints_);
    js_msg.position.resize(num_joints_);
    js_msg.velocity.resize(num_joints_);
    js_msg.effort.resize(num_joints_);
    temp_msg.name.resize(num_joints_);
    temp_msg.position.resize(num_joints_);
//    temp_msg.value_in_C.resize(num_joints_);

    // Command
    joint_effort_cmd_.resize(num_joints_, 0.0);

    // Initialize joint names
    for(unsigned int j_id = 0; j_id < num_joints_; j_id++)
    {
      js_msg.name[j_id] = joint_names_[j_id].c_str();
      temp_msg.name[j_id] = joint_names_[j_id].c_str();
    }

    // Initialize interfaces for each joint
    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
      // connect and register the joint state interface (reads)
      hardware_interface::JointStateHandle state_handle(joint_names_[joint_id].c_str(),
                                                    &joint_position_[joint_id],
                                                    &joint_velocity_[joint_id],
                                                    &joint_effort_[joint_id]);
      joint_state_interface_.registerHandle(state_handle);

      // connect and register the effort joint interface (writes)
      hardware_interface::JointHandle effort_handle(
              joint_state_interface_.getHandle(joint_names_[joint_id]),
              &joint_effort_cmd_[joint_id]);
      effort_joint_interface_.registerHandle(effort_handle);

    }
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);

    // initialize EtherCAT and get all devices to operational state
    initializeEthercat();
    generateAllSlaveMappings();
    startOperation();

    createEtherCATCheckThread();

    ros::TimerOptions timer_options(ros::Duration(0.002),
                                    boost::bind(&BumpybotHWInterface::EtherCATLoop, this, _1),
                                    &EtherCATUpdateQueue_, false, false);
    EtherCATTimer_ = nh_.createTimer(timer_options);
    EtherCATTimerThread_ = boost::thread(boost::bind(&BumpybotHWInterface::EtherCATTimerThread, this));
    EtherCATTimer_.start();

    ros::Duration delay(0.1);

    ROS_INFO("Finished initializing Bumpybot Hardware Interface");
    return true;
  }

  void BumpybotHWInterface::read(const ros::Time& time, const ros::Duration& period)
  {
    boost::recursive_mutex::scoped_lock lock(r_mutex_);
    
  for(int i = 0; i < ec_slavecount; i++)
    {
      //TODO use TPDO and RPDO structs to handle data transfer through SOEM
//      js_msg.name[i] = joint_names_[i].c_str();

      // Read incremental encoder tick counts
      int32_t position_count_low = *(ec_slave[i+1].inputs + 2);
      int32_t position_count_low_med = *(ec_slave[i+1].inputs + 3);
      int32_t position_count_med_high = *(ec_slave[i+1].inputs + 4);
      int32_t position_count_high = *(ec_slave[i+1].inputs + 5);
      int32_t count = (position_count_high << 24) | (position_count_med_high << 16) | (position_count_low_med << 8) | position_count_low;
      joint_position_[i] = wheel_encoder_sign_[i] * (count / wheel_encoder_counts_[i]) * 2.0 * M_PI; // convert counts -> radians
      js_msg.position[i] = joint_position_[i];

      // Read velocity
      int32_t velocity_count_low = *(ec_slave[i+1].inputs + 6);
      int32_t velocity_count_low_med = *(ec_slave[i+1].inputs + 7);
      int32_t velocity_count_med_high = *(ec_slave[i+1].inputs + 8);
      int32_t velocity_count_high = *(ec_slave[i+1].inputs + 9);
      int32_t vel = (velocity_count_high << 24) | (velocity_count_med_high << 16) | (velocity_count_low_med << 8) | velocity_count_low;
      joint_velocity_[i] = wheel_encoder_sign_[i] * (vel / wheel_encoder_counts_[i]);
      js_msg.velocity[i] = joint_velocity_[i];

      // Read torque
      int32_t torque_low = *(ec_slave[i+1].inputs + 13);
      int32_t torque_low_med = *(ec_slave[i+1].inputs + 14);
      int32_t torque_med_high = *(ec_slave[i+1].inputs + 15);
      int32_t torque_high = *(ec_slave[i+1].inputs + 16);
      int32_t torque = (torque_high << 24) | (torque_med_high << 16) | (torque_low_med << 8) | (torque_low);
      float torque_f = ((union convIntToFloat){.i32 = torque}).f32;
      joint_effort_[i] = torque_f;
      js_msg.effort[i] = joint_effort_[i];

      // Read temperature
      uint32_t temp_low = *(ec_slave[i+1].inputs + 17);
      uint32_t temp_med_low = *(ec_slave[i+1].inputs + 18);
      uint32_t temp_med_high = *(ec_slave[i+1].inputs + 19);
      uint32_t temp_high = *(ec_slave[i+1].inputs + 20);
      uint32_t temp = (temp_high << 24) | (temp_med_high << 16) | (temp_med_low << 8) | (temp_low);
      float temp_f = ((union convUIntToFloat){.u32 = temp}).f32;
      temp_msg.position[i] = temp_f;
//      temp_msg.value_in_C[i] = temp_f;
    }
    lock.unlock();

    // publish robot (measurement) state
    js_msg.header.stamp = ros::Time::now();
    temp_msg.header.stamp = ros::Time::now();
    js_pub.publish(js_msg);
    temp_pub.publish(temp_msg);
  }

  void BumpybotHWInterface::write(const ros::Time& time, const ros::Duration& period)
  {
    // E-stop

    // Enforce limits

    boost::recursive_mutex::scoped_lock lock(r_mutex_);
  for (int i = 0; i < m_num_slaves_; i++)
    {
      float effort = wheel_encoder_sign_[i] * joint_effort_cmd_[i];

      // convert float to 32 bits
      const unsigned char *pf = reinterpret_cast<const unsigned char*>(&effort);
      for(unsigned int j = 0; j < 4; j++)
      {
        *(ec_slave[i+1].outputs + 3 + j) = pf[j];
      }
    }
    lock.unlock();
  }

  void BumpybotHWInterface::cmd1Callback(const std_msgs::Float64& desired_torque)
  {
    // read desired command for motor 1
    joint_effort_cmd_[0] = desired_torque.data;
  }

  void BumpybotHWInterface::cmd2Callback(const std_msgs::Float64& desired_torque)
  {
    // read desired command for motor 2
    joint_effort_cmd_[1] = desired_torque.data;
  }

  void BumpybotHWInterface::cmd3Callback(const std_msgs::Float64& desired_torque)
  {
    // read desired command for motor 3
    joint_effort_cmd_[2] = desired_torque.data;
  }

  void BumpybotHWInterface::EtherCATTimerThread()
  {
    static const double timeout = 0.001;
    while (nh_.ok()) {
      EtherCATUpdateQueue_.callAvailable(ros::WallDuration(timeout));
    }
  }

    void BumpybotHWInterface::EtherCATLoop(const ros::TimerEvent&)
    {
      inOP = true;
      if (ec_slave[0].state == EC_STATE_OPERATIONAL )
      {
        ecx_send_processdata(&ecx_context);
        wkc = ecx_receive_processdata(&ecx_context, EC_TIMEOUTRET);
      }

    }

  void BumpybotHWInterface::loadURDF(ros::NodeHandle &nh, std::string param_name)
  {
    std::string urdf_string;
    urdf_model_ = new urdf::Model();

    // search and wait for robot_description on param server
    while (urdf_string.empty() && ros::ok())
    {
      std::string search_param_name;
      if (nh.searchParam(param_name, search_param_name))
      {
        ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                                                                                    nh.getNamespace() << search_param_name);
        nh.getParam(search_param_name, urdf_string);
      }
      else
      {
        ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                                                                                    nh.getNamespace() << param_name);
        nh.getParam(param_name, urdf_string);
      }

      usleep(100000);
    }

    if (!urdf_model_->initString(urdf_string))
      ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
    else
      ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
  }

    static int everest_write8 (uint16 slave, uint16 index, uint8 subindex, uint8 value)
    {
      int wkc;

      wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                         EC_TIMEOUTSAFE);
      return wkc;
    }

    static int everest_write16 (uint16 slave, uint16 index, uint8 subindex, uint16 value)
    {
      int wkc;

      wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                         EC_TIMEOUTSAFE);
      return wkc;
    }

    static int everest_write32 (uint16 slave, uint16 index, uint8 subindex, uint32 value)
    {
      int wkc;

      wkc = ec_SDOwrite (slave, index, subindex, FALSE, sizeof(value), &value,
                         EC_TIMEOUTSAFE);
      return wkc;
    }

static int everest_setup(uint16 slave)
{
  int wkc = 0;

  uint32_t dType_32 = 0;

  ROS_INFO("Everest XCR drive setup");

  wkc += everest_write8 (slave, 0x1C12, 0, 0);
  wkc += everest_write8 (slave, 0x1C13, 0, 0);

  /* Motor Configuration */
  // Rated Current
//   dType_float = 7.5;
//   wkc += everest_write8 (slave, 0x2100, 0, dType_float);

  if (wkc != 2)
  {
    printf ("Everest setup failed\n");
    return -1;
  }

  /* RPDO */
  // Unmap all registers from PDO by setting subindex 0x00 to zero
  dType_32 = 0x00000000;
  wkc += everest_write32 (slave, 0x1600, 0x00, dType_32);

  // Adding control word
  dType_32 = 0x60400010;
  wkc += everest_write32 (slave, 0x1600, 0x01, dType_32);

  // Adding operation mode
  dType_32 = 0x60600008;
  wkc += everest_write32 (slave, 0x1600, 0x02, dType_32);

  // Adding current quadrature set point
  dType_32 = 0x201A0020;
  wkc += everest_write32 (slave, 0x1600, 0x03, dType_32);

  wkc += everest_write8 (slave, 0x1600, 0, 3);

  /* TPDO */
  /* Unmap all registers from PDO by setting Subindex 0x00 to zero */
  dType_32 = 0x00000000;
  wkc += everest_write32 (slave, 0x1A00, 0x00, dType_32);

  // Subindex 1
  // Status word
  dType_32 = 0x60410010;
  wkc += everest_write32 (slave, 0x1A00, 0x01, dType_32);

  // Subindex 2
  // Actual position
  dType_32 = 0x60640020;
  wkc += everest_write32 (slave, 0x1A00, 0x02, dType_32);

  // Subindex 3
  // Actual velocity
  dType_32 = 0x606C0020;
  wkc += everest_write32 (slave, 0x1A00, 0x03, dType_32);

  // Subindex 4
  // Operation mode display
  dType_32 = 0x60610008;
  wkc += everest_write32 (slave, 0x1A00, 0x04, dType_32);

  // Subindex 5
  // Torque actual value
  dType_32 = 0x60770010;
  wkc += everest_write32 (slave, 0x1A00, 0x05, dType_32);

  // Subindex 6
  // Current quadrature demand
  dType_32 = 0x20720020;
  wkc += everest_write32 (slave, 0x1A00, 0x06, dType_32);

  // Subindex 7
  // Current quadrature demand
  dType_32 = 0x20610020;
  wkc += everest_write32 (slave, 0x1A00, 0x07, dType_32);

  // Enable mapping by setting SubIndex 0x00 to the number of mapped objects. (0x1600 RPDO1 mapping parameter).
  dType_32 = 0x00000007;
  wkc += everest_write32 (slave, 0x1A00, 0x00, dType_32);

  // Assign mapping to SM2
  wkc += everest_write16 (slave, 0x1C12, 1, 0x1600);
  wkc += everest_write16 (slave, 0x1C12, 0, 1);

  // Assign mapping to SM3
  wkc += everest_write16 (slave, 0x1C13, 1, 0x1A00);
  wkc += everest_write16 (slave, 0x1C13, 0, 1);

  if (wkc != 20)
  {
    printf(" TXPDO not configured correctly, Expected wkc: 20, got: %d\n", (wkc - 2));
    return -1;
  }

  /* Explicitly set flags that are (probably) invalid in EEPROM */
//   ec_slave[slave].SM[2].SMflags = 0x10024;

  /* Explicitly disable sync managers that are activated by EEPROM */
  ec_slave[slave].SM[4].StartAddr = 0;
  ec_slave[slave].SM[5].StartAddr = 0;

  /* Set a slave name */
  strncpy (ec_slave[slave].name, "Everest", EC_MAXNAME);

  ROS_INFO("Everest XCR drive setup finished without errors");

  return 1;
}

  bool BumpybotHWInterface::initializeEthercat()
  {
    ROS_INFO("Initializing EtherCAT...");

    //initialise SOEM, bind socket to ifname
    if(!ec_init(ifname_.c_str()))
    {
      ROS_ERROR("Failed to initialize EtherCAT master. Try running `sudo setcap cap_net_raw,cap_net_admin,cap_sys_rawio=+ep /opt/ros/melodic/lib/nodelet/nodelet`");
      return false;
    }

    m_num_slaves_ = 0;
    STUCK_WHILE(ros::ok()) //loop until full connection is established
    {
      osal_usleep(50);
      //find and auto-config slaves
      STUCK_WHILE(ros::ok())
      {
        osal_usleep(200);
        if(ec_config_init(FALSE) && ec_slavecount > 0)
        {
          if((unsigned int) ec_slavecount == m_num_slaves_)
          {
            break; //don't spam the message if nothing changed
          }
          ROS_INFO_STREAM("Found " << ec_slavecount << " slaves!");
          m_num_slaves_ = ec_slavecount;

          // --------------------- testing configuration
          for(unsigned int slave_ix = 1; slave_ix <= m_num_slaves_; slave_ix++)
          {
            ROS_INFO_STREAM( "Found " << ec_slave[slave_ix].name << " in state: " << ec_slave[slave_ix].state);
            ec_slave[slave_ix].PO2SOconfig = everest_setup;
            ec_slave[slave_ix].blockLRW = 1;
          }
          // --------------------- testing configuration ENDS
          break;
        }
        else
        {
          ROS_WARN_THROTTLE(1.0, "No slaves found! Will keep trying...");
        }
      }

      ec_config_map((void*) &IOmap);
      ec_configdc();

      STUCK_WHILE(ros::ok() && EcatError)
      {
        ROS_ERROR_STREAM( ec_elist2string());
      }

      //transition to SAFE_OP
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 20); //EC_TIMEOUTSTATE);

      if(ec_slave[0].state == EC_STATE_SAFE_OP)
      {
        break;
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "Not all slaves reached safe operational state! Will keep trying...");
        ec_readstate();
        for(unsigned int slave_idx = 1; slave_idx <= m_num_slaves_; slave_idx++)
        {
          if(ec_slave[slave_idx].state != EC_STATE_SAFE_OP)
          {
            ROS_ERROR_THROTTLE(1.0, "Slave %d State=%4x StatusCode=%4x : %s", slave_idx,
                               ec_slave[slave_idx].state,
                               ec_slave[slave_idx].ALstatuscode,
                               ec_ALstatuscode2string(ec_slave[slave_idx].ALstatuscode));
          }
        }
      }
    }

    ROS_INFO("EtherCAT initialization complete!");

    return true;
  }


  /** Read PDO assign structure */
  int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset, int bitoffset)
  {
      uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
      uint8 subcnt;
      int wkc, bsize = 0, rdl;
      int32 rdat2;
      uint8 bitlen, obj_subidx;
      uint16 obj_idx;
      int abs_offset, abs_bit;

      rdl = sizeof(rdat); rdat = 0;
      /* read PDO assign subindex 0 ( = number of PDO's) */
      wkc = ec_SDOread(slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
      rdat = etohs(rdat);
      /* positive result from slave ? */
      if ((wkc > 0) && (rdat > 0))
      {
          /* number of available sub indexes */
          nidx = rdat;
          bsize = 0;
          /* read all PDO's */
          for (idxloop = 1; idxloop <= nidx; idxloop++)
          {
              rdl = sizeof(rdat); rdat = 0;
              /* read PDO assign */
              wkc = ec_SDOread(slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
              /* result is index of PDO */
              idx = etohs(rdat);
              if (idx > 0)
              {
                  rdl = sizeof(subcnt); subcnt = 0;
                  /* read number of subindexes of PDO */
                  wkc = ec_SDOread(slave,idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
                  subidx = subcnt;
                  /* for each subindex */
                  for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
                  {
                      rdl = sizeof(rdat2); rdat2 = 0;
                      /* read SDO that is mapped in PDO */
                      wkc = ec_SDOread(slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
                      rdat2 = etohl(rdat2);
                      /* extract bitlength of SDO */
                      bitlen = LO_BYTE(rdat2);
                      bsize += bitlen;
                      obj_idx = (uint16)(rdat2 >> 16);
                      obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
                      abs_offset = mapoffset + (bitoffset / 8);
                      abs_bit = bitoffset % 8;
                      ODlist.Slave = slave;
                      ODlist.Index[0] = obj_idx;
                      OElist.Entries = 0;
                      wkc = 0;
                      /* read object entry from dictionary if not a filler (0x0000:0x00) */
                      if(obj_idx || obj_subidx)
                          wkc = ec_readOEsingle(0, obj_subidx, &ODlist, &OElist);
                      printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit, obj_idx, obj_subidx, bitlen);
                      if((wkc > 0) && OElist.Entries)
                      {
                          printf(" %-12s %s\n", dtype2string(OElist.DataType[obj_subidx], bitlen), OElist.Name[obj_subidx]);
                      }
                      else
                          printf("\n");
                      bitoffset += bitlen;
                  };
              };
          };
      };
      /* return total found bitlength (PDO) */
      return bsize;
  }


  void BumpybotHWInterface::generateAllSlaveMappings()
  {
    ec_readstate();
    for(unsigned int slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      STUCK_WHILE(ros::ok())
      {
        osal_usleep(50);

        uint8_t SMt_bug_add = 0;
        int outputs_bo = 0;
        int inputs_bo = 0;

        // read SyncManager Communication Type object count
        uint8_t nSM = 0;
        int rdl = sizeof(nSM);
        int wkc = ec_SDOread(slave_idx, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM, EC_TIMEOUTRXM);
        /* positive result from slave ? */
        if((wkc > 0) && (nSM > 2))
        {
          /* make nSM equal to number of defined SM */
          nSM--;
          /* limit to maximum number of SM defined, if true the slave can't be configured */
          if(nSM > EC_MAXSM)
            nSM = EC_MAXSM;
          /* iterate for every SM type defined */

          for(uint8_t iSM = 2; iSM <= nSM; iSM++)
          {
            uint8_t tSM = 0;
            rdl = sizeof(tSM);
            /* read SyncManager Communication Type */
            wkc = ec_SDOread(slave_idx, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl, &tSM, EC_TIMEOUTRXM);
            if(wkc > 0)
            {
              if((iSM == 2) && (tSM == 2)) // SM2 has type 2 == mailbox out, this is a bug in the slave!
              {
                SMt_bug_add = 1; // try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
                printf("Activated SM type workaround, possible incorrect mapping.\n");
              }
                // --------------- add Mecademic patch ---------
              else if((iSM == 2) && (tSM == 4))
              {
                tSM = 3;
              }
              else if((iSM == 3) && (tSM == 3))
              {
                tSM = 4;
              }
              // --------------- Mecademic patch ENDS -------
              if(tSM)
                tSM += SMt_bug_add;  // only add if SMt > 0

              if(tSM == 3) // outputs
              {
                int Tsize = si_PDOassign(slave_idx, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave_idx].outputs - (uint8 *)&IOmap[0]), outputs_bo );
                outputs_bo += Tsize;
              }
              if(tSM == 4) // inputs
              {
                int Tsize = si_PDOassign(slave_idx, ECT_SDO_PDOASSIGN + iSM, (int)(ec_slave[slave_idx].inputs - (uint8 *)&IOmap[0]), inputs_bo );
                inputs_bo += Tsize;
              }
            }
          }

          if((outputs_bo == 0) && (inputs_bo == 0))
          {
            ROS_ERROR_STREAM("Slave " << (int) slave_idx << " had no i/o bits, trying again");
          }
          else
          {
            break; //all is well, move to next slave
          }
        }
        else
        {
          ROS_ERROR_STREAM("Couldn't get obj count wkc=" << wkc << ", nsm=" << nSM << ", trying again");
        }
      }
    }
  }

  void BumpybotHWInterface::startOperation()
  {
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata(); //send one valid process data to make outputs in slaves happy
    ec_receive_processdata(EC_TIMEOUTRET);

    //request OP state for all slaves
    ec_writestate(0);

    //wait for all slaves to reach OP state
    int chk = 500;
    STUCK_IN();
    do
    {
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_statecheck(0, EC_STATE_OPERATIONAL, 100);
      osal_usleep(5);
    }
    while(chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
    STUCK_OUT();

    //make sure we didn't just time out
    if(ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
      ROS_INFO("All slaves operational!");
    }
    else
    {
      ROS_ERROR("Some slaves failed to reach operational state!");
    }

    osal_usleep(5);    
    for(unsigned int i = 1; i <= ec_slavecount; i++)
    {
      // Enable Everest servo and set to current mode
      *(ec_slave[i].outputs + 0) = 0x6;		// shutdown
      *(ec_slave[i].outputs + 2) = -2;		// set operation mode to current

      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET * 2);

      *(ec_slave[i].outputs + 0) = 0xF;		// switch on and enable operation
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET * 2);
      osal_usleep(5);
    }

    ROS_INFO("Everest enabled and desired current set to zero");
  }

  bool BumpybotHWInterface::createEtherCATCheckThread()
  {
    EtherCATCheckThread_ = boost::thread(boost::bind(&BumpybotHWInterface::EtherCATCheck, this));
    return true;
  }

  void BumpybotHWInterface::EtherCATCheck()
  {
    int slave;
    ros::Rate rate(100);
    while(ros::ok())
    {
      if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
        if (needlf)
        {
          needlf = FALSE;
          printf("\n");
        }
        /* one ore more slaves are not responding */
        ec_group[currentgroup].docheckstate = FALSE;
        ecx_readstate(&ecx_context);
        for (slave = 1; slave <= ec_slavecount; slave++)
        {
          if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
          {
            ec_group[currentgroup].docheckstate = TRUE;
            if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
            {
              printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
              ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
              ecx_writestate(&ecx_context, slave);
            }
            else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
            {
              printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
              ec_slave[slave].state = EC_STATE_OPERATIONAL;
              ecx_writestate(&ecx_context, slave);
            }
            else if(ec_slave[slave].state > 0x00)//EC_STATE_NONE)
            {
              if (ecx_reconfig_slave(&ecx_context, slave, EC_TIMEOUTMON))
              {
                ec_slave[slave].islost = FALSE;
                printf("MESSAGE : slave %d reconfigured\n",slave);
              }
            }
            else if(!ec_slave[slave].islost)
            {
              /* re-check state */
              ecx_statecheck(&ecx_context, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
              if (ec_slave[slave].state == 0x00)//EC_STATE_NONE)
              {
                ec_slave[slave].islost = TRUE;
                printf("ERROR : slave %d lost\n",slave);
              }
            }
          }
          if (ec_slave[slave].islost)
          {
            if(ec_slave[slave].state == 0x00)//EC_STATE_NONE)
            {
              if (ecx_recover_slave(&ecx_context, slave, EC_TIMEOUTMON))
              {
                ec_slave[slave].islost = FALSE;
                printf("MESSAGE : slave %d recovered\n",slave);
              }
            }
            else
            {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE : slave %d found\n",slave);
            }
          }
        }
        if(!ec_group[currentgroup].docheckstate)
          printf("OK : all slaves resumed OPERATIONAL.\n");
      }
      osal_usleep(10000);
    }
  }

  bool BumpybotHWInterface::clearFaults(bumpybot_hw_interface::ClearFaults::Request &req,
					bumpybot_hw_interface::ClearFaults::Response &res) 
  {
    bool success = true;
    res.out = true;
    stopOperation();

    ROS_INFO("Attempting to clear faults");
    for (unsigned int i = 1; i <= ec_slavecount; i++)
    {
	// clear faults in each Everest Servo
	*(ec_slave[i].outputs + 0) = 0x80;

	ec_send_processdata();
	ec_receive_processdata(EC_TIMEOUTRET * 2);
    }

    // check if all servos reached OP again
    if (ec_slave[0].state != EC_STATE_OPERATIONAL)
    {
	ROS_INFO("Not all servos reached OP after clearing faults.");
	success = false;
	res.out = false;
    }

    startOperation();
    return success;
  }

}
