
#include <rtf/TestAssert.h>
#include <rtf/dll/Plugin.h>
#include <yarp/os/Bottle.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>
#include <yarp/sig/all.h>
#include <yarp/os/Time.h>
#include "MakeItRollTest.h"

using namespace RTF;
using namespace yarp::os;
using namespace yarp::math;
using namespace yarp::sig;

using namespace std;

PREPARE_PLUGIN(MakeItRollTest)
MakeItRollTest::MakeItRollTest() : TestCase("MakeItRollTest") { }
MakeItRollTest::~MakeItRollTest() { }

bool MakeItRollTest::setup(int argc, char** argv) {
    RTF_TEST_REPORT("running MakeItRollTest::setup...");

    testerPortName="/MakeItRollTest/rpc";
    makeItRollPortName="/service";

    testerPort.open(testerPortName);

    //setup connection with make-it-roll by /service port
    RTF_ASSERT_ERROR_IF(yarp.connect(testerPortName,makeItRollPortName),
                       Asserter::format("Failed to open connection to %s", makeItRollPortName.c_str()));
    RTF_TEST_REPORT("connection to /service done");

    //setup connection with iCubSim by /icubSim/world port
    portToIcubSim.open("/MakeItRollTest/SimWorld");
    RTF_ASSERT_ERROR_IF(yarp.connect("/MakeItRollTest/SimWorld","/icubSim/world"),
                       Asserter::format("Failed to open connection to /icubSim/world"));
    RTF_TEST_REPORT("connection to /icubSim/world done");

    return true;
}
void MakeItRollTest::tearDown() {
    RTF_TEST_REPORT("running MakeItRollTest::teardown...");

    testerPort.close();
    RTF_TEST_REPORT("testerPort closed");

    portToIcubSim.close();
    RTF_TEST_REPORT("portToIcubSim closed");
    
    RTF_TEST_REPORT("end of MakeItRollTest");

}
void MakeItRollTest::run() {
    RTF_TEST_REPORT("running MakeItRollTest::run...");
    Bottle request, response;

    //check initial ball position
    RTF_TEST_REPORT("checking initial ball position...");

    request.fromString("world get ball");
    portToIcubSim.write(request, response);

    Vector init_ball_pos(3);
    init_ball_pos[0]=response.get(0).asDouble();
    init_ball_pos[1]=response.get(1).asDouble();
    init_ball_pos[2]=response.get(2).asDouble();

    request.clear();
    response.clear();

    //asking icubSim to look down
    RTF_TEST_REPORT("asking icubSim to look down...");
    request.fromString("look_down");
    testerPort.write(request, response);

    string resp_ld = response.get(0).asString();
    string right_ans = "Yep! I'm looking down now!";
    RTF_TEST_FAIL_IF(!resp_ld.compare(right_ans), resp_ld);

    request.clear();
    response.clear();

    //asking icubSim to make the ball roll
    Time::delay(1.0);
    RTF_TEST_REPORT("asking icubSim to make the ball roll...");
    request.fromString("make_it_roll");
    testerPort.write(request, response);

    request.clear();
    response.clear();

    //wait 5 seconds to let the ball move after the hit
    Time::delay(5.0);

    request.fromString("world get ball");
    portToIcubSim.write(request, response);

    Vector final_ball_pos(3);
    final_ball_pos[0]=response.get(0).asDouble();
    final_ball_pos[1]=response.get(1).asDouble();
    final_ball_pos[2]=response.get(2).asDouble();

    bool moved = false;
    for(int i = 0; i++; i<3){
        if(final_ball_pos[i]!=init_ball_pos[i]){
            moved = true;
        }

    }

    RTF_TEST_FAIL_IF(!moved, "the ball is still at the initial postion!");

    request.clear();
    response.clear();

    //asking icubSim to go back home
    RTF_TEST_REPORT("asking icubSim to go back home");
    request.fromString("home");
    testerPort.write(request, response);

    request.clear();
    response.clear();

    RTF_TEST_REPORT("this is the end of the test");
}
