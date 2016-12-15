#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArm, drvGaze;
    ICartesianControl *iarm;
    IGazeControl      *igaze;

    BufferedPort<ImageOf<PixelRgb> > imgLPortIn,imgRPortIn;
    Port imgLPortOut,imgRPortOut;
    RpcServer rpcPort;

    Mutex mutex;
    Vector cogL,cogR;
    bool okL,okR;

    Vector init_pose,init_rot,init_gaze;

    int startup_iarm_context_id,startup_igaze_context_id;

    /***************************************************/
    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
    {
        //yInfo("getCOG function");
        int xMean=0;
        int yMean=0;
        int ct=0;

        for (int x=0; x<img.width(); x++)
        {
            for (int y=0; y<img.height(); y++)
            {
                PixelRgb &pixel=img.pixel(x,y);
                if ((pixel.b>5.0*pixel.r) && (pixel.b>5.0*pixel.g))
                {
                    //yInfo("getCOG: blu detected");
                    xMean+=x;
                    yMean+=y;
                    ct++;
                }
            }
        }

        if (ct>0)
        {
            cog.resize(2);
            cog[0]=xMean/ct;
            cog[1]=yMean/ct;
            //yInfo("cog = (%s)",cog.toString(3,3).c_str());
            return true;
        }
        else
            return false;
    }

    /***************************************************/
    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    {
        yInfo("retrieveTarget3D function");
        int context;

        igaze->storeContext(&context);
        Vector x;
        igaze->triangulate3DPoint(cogL,cogR,x);
        igaze->restoreContext(context);

        return x;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
        yInfo("fixate function");
        int context;

        igaze->storeContext(&context);
        igaze->lookAtFixationPoint(x);                  // move the gaze to the desired fixation point
        igaze->waitMotionDone();                        // wait until the operation is done

        Vector real;
        igaze->getFixationPoint(real);                  // retrieve the current fixation point
        cout<<"final error = "<<norm(x-real)<<endl;     // return a measure of the displacement error
        igaze->restoreContext(context);
    }

    /***************************************************/
    Vector computeHandOrientation()
    {
        yInfo("computeHandOrientation function");
        // Vector ox(4), oy(4), oz(4);
        // ox[0]=1.0; ox[1]=0.0; ox[2]=0.0; ox[3]=-M_PI/2.0;
        // oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=+M_PI;
        // oz[0]=0.0; oz[1]=0.0; oz[2]=1.0; oz[3]=M_PI/2.0;
        //
        // Matrix Rx=yarp::math::axis2dcm(ox);        // from axis/angle to rotation matrix notation
        // Matrix Ry=yarp::math::axis2dcm(oy);
        // Matrix Rz=yarp::math::axis2dcm(oz);
        // Matrix R=Rz*Ry*Rx;                         // compose the two rotations keeping the order
        // Vector o=yarp::math::dcm2axis(R);          // from rotation matrix back to the axis/angle notation
        Matrix R(3,3);

        R(0,0)= -1.0;  R(0,1)= 0.0;  R(0,2)= 0.0;
	    R(1,0)=  0.0;  R(1,1)= 0.0;  R(1,2)=-1.0;
        R(2,0)=  0.0;  R(2,1)=-1.0;  R(2,2)= 0.0;

        return yarp::math::dcm2axis(R);
    }

    /***************************************************/
    void approachTargetWithHand(const Vector &x, const Vector &o)
    {
        yInfo("approachTargetWithHand function");
        int context;

        Vector approched_x(3);
        approched_x[0] = x[0]+0.05;
        approched_x[1] = x[1]-0.15;
        approched_x[2] = x[2]-0.2;

        iarm->storeContext(&context);
        iarm->goToPoseSync(approched_x,o);   // send request and wait for reply
        iarm->waitMotionDone(0.04, 5);       // wait until the motion is done and ping at each 0.04 seconds
        iarm->restoreContext(context);
        yInfo("approachTargetWithHand: ball approched");
    }

    /***************************************************/
    void makeItRoll(const Vector &x, const Vector &o)
    {
        yInfo("makeItRoll function");
        int context;

        Vector roll_x = x;
        roll_x[0] = x[0]+0.05;
        roll_x[1] = x[1]-0.15;
        roll_x[2] = x[2];

        iarm->storeContext(&context);
        iarm->setTrajTime(1);           // given in seconds
        iarm->goToPoseSync(roll_x,o);        // send request and wait for reply
        iarm->waitMotionDone(0.04, 5);  // wait until the motion is done and ping at each 0.04 seconds
        iarm->restoreContext(context);

    }

    /***************************************************/
    void look_down()
    {
        yInfo("look_down function");
        int context;
        Vector ang(3);
        igaze->getAngles(ang);

        // ang[0] azimuth-component [deg]
        // ang[1] elevation-component [deg]
        // ang[2] vergence-component [deg]

        ang[1]-=39.0;

        igaze->storeContext(&context);
        igaze->lookAtAbsAngles(ang);
        igaze->waitMotionDone();
        igaze->restoreContext(context);
    }

    /***************************************************/
    void roll(const Vector &cogL, const Vector &cogR)
    {
        yInfo("detected cogs = (%s) (%s)",
              cogL.toString(0,0).c_str(),cogR.toString(0,0).c_str());

        Vector x=retrieveTarget3D(cogL,cogR);
        yInfo("retrieved 3D point = (%s)",x.toString(3,3).c_str());

        fixate(x);
        yInfo("fixating at (%s)",x.toString(3,3).c_str());

        Vector o=computeHandOrientation();
        yInfo("computed orientation = (%s)",o.toString(3,3).c_str());

        approachTargetWithHand(x,o);
        yInfo("approached");

        makeItRoll(x,o);
        yInfo("roll!");
    }

    /***************************************************/
    void home()
    {
        yInfo("home function");

        iarm->goToPoseSync(init_pose, init_rot);
        iarm->waitMotionDone(0.04,5);
        yInfo("home: arm is at home");

        igaze->lookAtAbsAngles(init_gaze);
        igaze->waitMotionDone();
        iarm->waitMotionDone(0.04,5);

        iarm->restoreContext(startup_iarm_context_id);
        igaze->restoreContext(startup_igaze_context_id);
        yInfo("home: gaze is at home");

    }

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        Property arm_options;
        RpcClient iarm_port;
        iarm_port.open("/iarm");
        arm_options.put("device", "cartesiancontrollerclient");
        arm_options.put("local", "/iarm");                 //local port names
        arm_options.put("remote", "/icubSim/cartesianController/right_arm");         //where we connect to

        drvArm.open(arm_options);
        if (!drvArm.isValid()) {
            printf("arm Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            return 1;
        }
        drvArm.view(iarm);

        Vector curDof;
        iarm->getDOF(curDof);
        //cout<<"["<<curDof.toString()<<"]"<<endl;  // [0 0 0 1 1 1 1 1 1 1] will be printed out
        Vector newDof(3);
        newDof[0]=2;    // torso pitch: 1 => enable
        newDof[1]=1;    // torso roll:  2 => skip
        newDof[2]=2;    // torso yaw:   1 => enable
        iarm->setDOF(newDof,curDof);
        //cout<<"["<<curDof.toString()<<"]"<<endl;  // [1 0 1 1 1 1 1 1 1 1] will be printed out

        Property gaze_options;
        RpcClient igaze_port;
        igaze_port.open("/igaze");
        gaze_options.put("device","gazecontrollerclient");
        gaze_options.put("remote","/iKinGazeCtrl");
        gaze_options.put("local","/igaze");

        drvGaze.open(gaze_options);
        if (!drvGaze.isValid()) {
            printf("gaze Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            return 1;
        }
        drvGaze.view(igaze);

        iarm->storeContext(&startup_iarm_context_id);
        igaze->storeContext(&startup_igaze_context_id);

        bool success = false;
        while(!success){
	        success=iarm->getPose(init_pose,init_rot);
        }
        yInfo("home orientation = (%s)",init_rot.toString(3,3).c_str());
	    yInfo("home position = (%s)",init_pose.toString(3,3).c_str());

        igaze->getAngles(init_gaze);

        imgLPortIn.open("/imgL:i");
        imgRPortIn.open("/imgR:i");

        imgLPortOut.open("/imgL:o");
        imgRPortOut.open("/imgR:o");

        rpcPort.open("/service");
        attach(rpcPort);

        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvArm.close();
        drvGaze.close();

        imgLPortIn.close();
        imgRPortIn.close();
        imgLPortOut.close();
        imgRPortOut.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- make_it_roll");
            reply.addString("- home");
            reply.addString("- quit");
        }
        else if (cmd=="look_down")
        {
            look_down();
            reply.addString("Yep! I'm looking down now!");
        }
        else if (cmd=="make_it_roll") //roll?
        {
            mutex.lock();
            ImageOf<PixelRgb> *imgL=imgLPortIn.read();
            ImageOf<PixelRgb> *imgR=imgRPortIn.read();

            if (getCOG(*imgL, cogL)&&getCOG(*imgR, cogR))
            {
                roll(cogL,cogR);
                reply.addString("Yeah! I've made it roll like a charm!");
            }
            else
                reply.addString("I don't see any object!");
            mutex.unlock();
        }
        else if (cmd=="home")
        {
            home();
            reply.addString("I've got the hard work done! Going home.");
        }
        else if (cmd=="quit")
        {
            reply.addString("Bye!!");
            close();
        }
        else
            return RFModule::respond(command,reply);

        return true;
    }

    /***************************************************/
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images
    }

    /***************************************************/
    bool updateModule()
    {
        //yInfo("update module begin..");
        mutex.lock();
        // get fresh images
        ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // interrupt sequence detected
        if ((imgL==NULL) || (imgR==NULL)){
            return false;
            yInfo("one of the two images are null..");
        }

        // compute the center-of-mass of pixels of our color
        okL=getCOG(*imgL,cogL);
        okR=getCOG(*imgR,cogR);
        mutex.unlock();

        PixelRgb color;
        color.r=255; color.g=0; color.b=0;

        if (okL)
            draw::addCircle(*imgL,color,(int)cogL[0],(int)cogL[1],5);

        if (okR)
            draw::addCircle(*imgR,color,(int)cogR[0],(int)cogR[1],5);

        imgLPortOut.write(*imgL);
        imgRPortOut.write(*imgR);

        return true;
    }
};


/***************************************************/
int main()
{
    yInfo("ctrlModule started");
    Network yarp;
    if (!yarp.checkNetwork())
        return 1;

    CtrlModule mod;
    ResourceFinder rf;
    return mod.runModule(rf);
}
