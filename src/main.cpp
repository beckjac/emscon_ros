// main.cpp

#include <vector>

#include <ros/ros.h>

#include <boost/asio.hpp> // Provides sockets
#include <boost/thread.hpp>

#include <emscon_ros/ES_CPP_API_Def.h> // Emscon API
const int MAX_PACKET_SIZE = 64*1024;

// Class for command interface
class EmsconCommand : public CESAPICommand
{
// Members
  boost::shared_ptr<boost::asio::ip::tcp::socket> socket_;

// Construction
public:
  EmsconCommand(boost::shared_ptr<boost::asio::ip::tcp::socket> socket) : socket_(socket) {} 
  ~EmsconCommand(){}

protected:
  // CESAPICommand's virtual function override
  virtual bool SendPacket(void* pPacketStart, long lPacketSize) 
  {
    long sentData = boost::asio::write(*socket_, boost::asio::buffer((const char*)pPacketStart, lPacketSize));
    
    if(sentData != lPacketSize)
    {
      ROS_ERROR_STREAM("Failed to send data to laser");
      return false;
    }
    
    return true;
  } 
};


// Class for receive interface
class EmsconReceive: public CESAPIReceive
{
public:
  EmsconReceive(ros::NodeHandlePtr node_handle) : node_handle_(node_handle)
  {
    server_ready_ = false;
  } 
  ~EmsconReceive() {;} 

  // make protected ReceiveData() accessible from outside (i.e. define a public overhead)
  // This is due to be accessed from the thread procedure, which is not a class member
  // (if it was, it would have to be static and thus ReceiveData would have to be static....)

  // No longer needed for V1.5 since ReceiveData now public
  //bool ReceiveData(void* packetStart, long packetSize) 
  //   {return CESAPIReceive::ReceiveData(packetStart, packetSize);}


  ////////////////////////////////////////////////////////////////////
  // override virtual functions of those answers you are interested in:
  bool server_ready_;

protected:
  // Members
  ros::NodeHandlePtr node_handle_;

  // general and unsolicited answer handlers:

  void OnCommandAnswer(const BasicCommandRT& cmd) // called for every command
  {
    printf("OnCommandAnswer: cmd=%d, status=%d, packet_type=%d, packet_size=%d\n", 
      cmd.command, cmd.status, cmd.packetHeader.type, cmd.packetHeader.lPacketSize);
    
    if(cmd.status == ES_RS_AllOK)
      server_ready_ = true;
    else
      server_ready_ = false;
  }

  void OnErrorAnswer(const ErrorResponseT& error) // called on unsolicited error, e.g. beam break
  {
    printf("OnErrorAnswer: cmd=%d, status=%d, packet_type=%d, packet_size=%d\n", 
      error.command, error.status, error.packetHeader.type, error.packetHeader.lPacketSize);
  }

  // Particular command/data handlers (as far as affected by this sample):

  /**
  // For conventional 3D trackers
  // show 3 coords only, much other result data available, see SingleMeasResultT
  void OnSingleMeasurementAnswer(const SingleMeasResultT& singleMeas) 
    {printf("OnSingleMeasurementAnswer(%lf, %lf, %lf)\n", singleMeas.dVal1, singleMeas.dVal2, singleMeas.dVal3);}
  **/

  // Appropriate 6DOF supporting hardware required.
  // Show 3 coords and 3 rotation angels only, many other result data available, see ProbeStationaryResultT.
  void OnStationaryProbeMeasurementAnswer(const ProbeStationaryResultT& statProbeMeas)
  {
    printf("Position(%lf, %lf, %lf)\n", statProbeMeas.dPosition1, statProbeMeas.dPosition2, statProbeMeas.dPosition3);
    printf("RotationAngles(%lf, %lf, %lf)\n", statProbeMeas.dRotationAngleX, statProbeMeas.dRotationAngleY, statProbeMeas.dRotationAngleZ);
    const double q0 = statProbeMeas.dQuaternion0;
    const double q1 = statProbeMeas.dQuaternion1;
    const double q2 = statProbeMeas.dQuaternion2;
    const double q3 = statProbeMeas.dQuaternion3;
    printf("Quaternion(%lf, %lf, %lf, %lf)\n", q0, q1, q2, q3);
    const double R00 = q0*q0+q1*q1-q2*q2-q3*q3;
    const double R01 = 2*(q1*q2-q0*q3);
    const double R02 = 2*(q1*q3+q0*q2);
    const double R10 = 2*(q1*q2+q0*q3);
    const double R11 = q0*q0-q1*q1+q2*q2-q3*q3;
    const double R12 = 2*(q2*q3-q0*q1);
    const double R20 = 2*(q1*q3-q0*q2);
    const double R21 = 2*(q3*q2+q0*q1);
    const double R22 = q0*q0-q1*q1-q2*q2+q3*q3;
    printf("x-Axis(%lf, %lf, %lf)\n", R00, R10, R20);
    printf("y-Axis(%lf, %lf, %lf)\n", R01, R11, R21);
    printf("z-Axis(%lf, %lf, %lf)\n", R02, R12, R22);
  }
  
  void OnContinuousProbeMeasurementAnswer(const ProbeContinuousResultT& continuousProbeMeas)
  {
    ROS_INFO_STREAM("Probe");
  }

  void OnSetUnitsAnswer() {printf("OnSetUnitsAnswer()\n");} // just a confirmation when succeeded

  void OnGoBirdBathAnswer() {printf("OnGoBirdBathAnswer()\n");} // just a confirmation when succeeded

  void OnSetMeasurementModeAnswer() {printf("OnSetMeasurementModeAnswer() \n");} // just a confirmation when succeeded

  void OnGetReflectorsAnswer(const int iTotalReflectors,
                             const int iInternalReflectorId,
                             const ES_TargetType targetType,
                             const double dSurfaceOffset,
                             const unsigned short cReflectorName[32]) 
  {
    // cReflectorName[] is UNICODE by convention. Quick and Dirty Unicode to Ansi conversion:
    const int size = 32; char buf[size];

    for (int i = 0; i < size; i++)
      buf[i] = cReflectorName[i] & 0xff;
      
    printf("OnGetReflectorsAnswer(): total=%d, id=%d, type=%d, offset=%lf, name=%s\n", 
      iTotalReflectors, iInternalReflectorId, targetType, dSurfaceOffset, buf);
  }
   
  void OnSetSearchParamsAnswer() {printf("OnSetSearchParamsAnswer() \n");} // just a confirmation when succeeded

  void OnGetSearchParamsAnswer(const SearchParamsDataT& searchParams) 
  {
    printf("OnGetSearchParamsAnswer(SearchRadius=%lf, TimeOut=%ld) \n", searchParams.dSearchRadius, searchParams.lTimeOut);
  }

  // Note: if a command returns with other then ES_AllOK status (which is shown by OnCommandAnswer()), 
  //       then the particular command handler will not be called!
};

class EmsconInterface
{
public:
  EmsconInterface(boost::shared_ptr<boost::asio::ip::tcp::socket> socket, ros::NodeHandlePtr node_handle) :
  cmd_(socket), recv_(node_handle), socket_(socket)
  {
    // Store node handle
    node_handle_ = node_handle;
    
    // Spawn reciever thread
    recv_thread_ = boost::thread(boost::bind(&EmsconInterface::receivePackets, this));
    
    // Initialize laser
    initLaser(node_handle);
  }
  
  ~EmsconInterface()
  {
    // Stop thread
    recv_thread_.join();
  }
  
protected:
  // Members
  EmsconCommand cmd_;
  EmsconReceive recv_;
  ros::NodeHandlePtr node_handle_;
  
  boost::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  boost::thread recv_thread_;
  
  // Functions
  // Worker thread to read socket
  void receivePackets()
  {
    int header_size = 4;
    std::vector<char> buffer;
    std::vector<char> packet;
    
    while(ros::ok())
    {
      // Check we have enough bytes to measure packet size
      long ready_bytes = socket_->available();
      if(ready_bytes < header_size)
        continue;
      
      // Read into buffer
      buffer.resize(header_size);
      boost::asio::read(*socket_, boost::asio::buffer(buffer));
      
      // Transfer to packet
      packet = buffer;
      
      // Check amount of data to read
      PacketHeaderT *header = (PacketHeaderT*)packet.data(); // Cast buffer to header
      long packet_size = header->lPacketSize;
      
      if((packet_size < header_size) || (packet_size > MAX_PACKET_SIZE))
      {
        ROS_WARN_STREAM("Indicated packet size " << packet_size << ", is out of bounds. Discarding.");
        continue;
      }
      
      // Read full message
      buffer.resize(packet_size - header_size);
      boost::asio::read(*socket_, boost::asio::buffer(buffer));
      
      // Transfer to packet
      packet.insert(packet.end(), buffer.begin(), buffer.end());
      if(packet.size() != packet_size)
      {
        ROS_WARN_STREAM("Didn't get expected amount of data");
        continue;
      }
    
      // Pass data to API
      bool success = recv_.ReceiveData(packet.data(), packet.size());
      if(!success)
        ROS_ERROR_STREAM("Failed to parse Emscon data");
    }
  }
  
  // Blocks until the server sends the OK status
  void waitForServer()
  {
    while(!recv_.server_ready_);
    
    recv_.server_ready_ = false;
  }
  
  // Reads parameters from node handle to initialize laser
  void initLaser(ros::NodeHandlePtr node_handle)
  {
    ROS_INFO_STREAM("Setting units");
    cmd_.SetUnits(ES_LU_Meter, ES_AU_Radian, ES_TU_Celsius, ES_PU_MmHg, ES_HU_RH);
    waitForServer();
    //cmd_.SetEnvironmentParams(); // Just use defaults
    
    ROS_INFO_STREAM("Initializing tracker");
    cmd_.Initialize();
    waitForServer();
    
    ROS_INFO_STREAM("Setting measurement mode");
    cmd_.SetMeasurementMode(ES_MM_ContinuousTime);
    waitForServer();
    cmd_.SetContinuousTimeModeParams(20, 0, false, ES_RT_Sphere);
    waitForServer();
    
    std::string reflector_name = node_handle_->param<std::string>("reflector_name", "CCR-1.5in");
    ROS_INFO_STREAM("Finding reflector '" << reflector_name << "'");
    findReflector(reflector_name);
    
    ROS_INFO_STREAM("Moving to birdbath");
    cmd_.GoBirdBath();
    waitForServer();
    
    ROS_INFO_STREAM("Setting coordinate parameters");
    cmd_.SetStationOrientationParams(0, 0, 0, 0, 0, 0);
    waitForServer();
    cmd_.SetTransformationParams(0, 0, 0, 0, 0, 0, 1);
    waitForServer();
    cmd_.SetCoordinateSystemType(ES_CS_RHR);
    waitForServer();
    
    ROS_INFO_STREAM("Applying system settings");
    SystemSettingsDataT settings;
    settings.bApplyTransformationParams = true;
    settings.bApplyStationOrientationParams = true;
    settings.bKeepLastPosition = true;
    settings.bSendUnsolicitedMessages = true;
    settings.bSendReflectorPositionData = false;
    cmd_.SetSystemSettings(settings);
    waitForServer();
    
    ROS_INFO_STREAM("Beginning measurement");
    cmd_.StartMeasurement();
    waitForServer();
    
    ROS_INFO_STREAM("Tracker ready");
  }
  
  void findReflector(std::string reflector_name)
  {
    cmd_.GetReflectors();
    waitForServer();
    
    //~ = cmd_.GetReflectors();
    //~ int id;
    //~ for string in var
    //~ {
      //~ if string == reflector_name
      //~ {
        //~ id = num;
        //~ break;
      //~ }
    //~ }
    //~ cmd_.SetReflector(0);
    
    return;
  }
};

int main(int argc, char* argv[])
{
  // Setup ROS
  ros::init(argc, argv, "emscon_ros_node");
  ros::NodeHandlePtr node_handle = boost::make_shared<ros::NodeHandle>();
  
  // Connect socket
  std::string host = "192.168.0.1";
  int port = 700;
  
  boost::asio::io_service ios;
  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(host), port);
  
  boost::shared_ptr<boost::asio::ip::tcp::socket> socket = boost::make_shared<boost::asio::ip::tcp::socket>(boost::ref(ios));
  socket->connect(endpoint);
  
  // Instantiate interface classes
  EmsconInterface interface(socket, node_handle);
  
  // Start publishing
  while(ros::ok())
  {
    ios.poll();
    ros::spinOnce();
  }
  
  return 0;
}
