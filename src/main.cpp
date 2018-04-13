
#include <ros.h>
#include <boost/asio.hpp> // Provides sockets
#include <emscon_ros/ES_CPP_API_Def.h> // Emscon API

// Class for command interface
class EmsconCommand : public CESAPICommand
{
// Members
  boost::asio::ip::tcp::socket sock;

// Construction
public:
  EmsconCommand(boost::asio::ip::tcp::socket socket)
  {
    sock = socket;
  } 
  ~EmsconCommand() {;}

protected:
  // CESAPICommand's virtual function override
  virtual bool SendPacket(void* pPacketStart, long lPacketSize) 
  {
    long sentData = sock.write(boost::asio::buffer((const char*)pPacketStart, lPacketSize));
    
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
  EmsconReceive() {;} 
  ~EmsconReceive() {;} 

  // make protected ReceiveData() accessible from outside (i.e. define a public overhead)
  // This is due to be accessed from the thread procedure, which is not a class member
  // (if it was, it would have to be static and thus ReceiveData would have to be static....)

  // No longer needed for V1.5 since ReceiveData now public
  //bool ReceiveData(void* packetStart, long packetSize) 
  //   {return CESAPIReceive::ReceiveData(packetStart, packetSize);}


  ////////////////////////////////////////////////////////////////////
  // override virtual functions of those answers you are interested in:

protected:
  // general and unsolicited answer handlers:

  void OnCommandAnswer(const BasicCommandRT& cmd) // called for every command
  {
    printf("OnCommandAnswer: cmd=%d, status=%d, packet_type=%d, packet_size=%d\n", 
      cmd.command, cmd.status, cmd.packetHeader.type, cmd.packetHeader.lPacketSize);
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

  void OnSetUnitsAnswer() {printf("OnSetUnitsAnswer()\n");} // just a confirmation when succeeded

  void OnGoBirdBathAnswer() {printf("OnGoBirdBathAnswer()\n");} // just a confirmation when succeeded

  void OnSetMeasurementModeAnswer() {printf("OnSetMeasurementModeAnswer() \n");} // just a confirmation when succeeded

  void OnGetReflectorsAnswer(const int iTotalReflectors,
                             const int iInternalReflectorId,
                             const ES_TargetType targetType,
                             const double dSurfaceOffset,
                             const unsigned short cReflectorName[32]) 
  {  // cReflectorName[] is UNICODE by convention. Quick and Dirty Unicode to Ansi conversion:
    const int size = 32; char buf[size];

    for (int i = 0; i < size; i++)
      buf[i] = LOBYTE(cReflectorName[i]);
      
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

int main(int argc, char* argv[])
{
  // Connect socket
  std::string host = "192.168.0.1";
  int port = 700;
  
  boost::asio::io_service ios;
  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(host), port);
  boost::asio::ip::tcp::socket socket(ios);
  
  socket.connect(endpoint);
  
  // Instantiate interface classes
  EmsconCommand command;
  EmsconReceive receive;
  
  // Initialize laser
  
  // Start publishing
  while(true)
  {
    ios.poll();
    //nh.spinOnce();
  }
  
  return 0;
}
