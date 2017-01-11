
#ifndef _MAKEITROLL_H_
#define _MAKEITROLL_H_

#include <rtf/TestCase.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Network.h>

#include <string>


using namespace yarp::os;
using namespace std;

class MakeItRollTest : public RTF::TestCase {

private:
    ResourceFinder rf;
    string makeItRollPortName, testerPortName;
    RpcClient testerPort, portToIcubSim;
    Network yarp;

public:
    MakeItRollTest();
    virtual ~MakeItRollTest();
    virtual bool setup(int argc, char** argv);
    virtual void tearDown();
    virtual void run();
};
#endif //_MAKEITROLL_H_
