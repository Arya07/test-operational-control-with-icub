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

    Vector home_pose,home_rot,init_gaze;

    int startup_context_id;

    /***************************************************/
    bool getCOG(ImageOf<PixelRgb> &img, Vector &cog)
    {
        yInfo("getCOG function");
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
                    yInfo("getCOG: blu detected");
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
            yInfo("cog = (%s)",cog.toString(3,3).c_str());
            return true;
        }
        else
            return false;
    }

    /***************************************************/
    Vector retrieveTarget3D(const Vector &cogL, const Vector &cogR)
    {
        Vector x;
        igaze->triangulate3DPoint(cogL,cogR,x);
        return x;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
        igaze->lookAtFixationPoint(x);                  // move the gaze to the desired fixation point
        igaze->waitMotionDone();                        // wait until the operation is done

        Vector real;
        igaze->getFixationPoint(real);                  // retrieve the current fixation point
        cout<<"final error = "<<norm(x-real)<<endl;     // return a measure of the displacement error
    }

    /***************************************************/
    Vector computeHandOrientation()
    {
        Vector ox(4), oy(4), oz(4);
        ox[0]=1.0; ox[1]=0.0; ox[2]=0.0; ox[3]=-M_PI/2.0;
        oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=+M_PI;
        oz[0]=0.0; oz[1]=0.0; oz[2]=1.0; oz[3]=-M_PI/2.0;

        Matrix Rx=yarp::math::axis2dcm(ox);        // from axis/angle to rotation matrix notation
        Matrix Ry=yarp::math::axis2dcm(oy);
        Matrix Rz=yarp::math::axis2dcm(oz);
        Matrix R=Rz*Ry*Ry;                         // compose the two rotations keeping the order
        Vector o=yarp::math::dcm2axis(R);          // from rotation matrix back to the axis/angle notation
    }

    /***************************************************/
    void approachTargetWithHand(const Vector &x, const Vector &o)
    {
        Vector approched_x = x;
        approched_x[0]-=0.10;
        iarm->goToPoseSync(approched_x,o);   // send request and wait for reply
        iarm->waitMotionDone(0.04);  // wait until the motion is done and ping at each 0.04 seconds
    }

    /***************************************************/
    void makeItRoll(const Vector &x, const Vector &o)
    {
        Vector over_x = x;
        over_x[0]+=0.10;
        iarm->goToPoseSync(over_x,o);   // send request and wait for reply
        iarm->waitMotionDone(0.04);  // wait until the motion is done and ping at each 0.04 seconds
    }

    /***************************************************/
    void look_down()
    {
        int context;
        Vector ang(3);
        igaze->getAngles(ang);

        // ang[0] azimuth-component [deg]
        // ang[1] elevation-component [deg]
        // ang[2] vergence-component [deg]

        //ang[0]+=20.0;
        ang[1]-=80.0;

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
        iarm->goToPoseSync(home_pose, home_rot);
        iarm->waitMotionDone(0.04);
        igaze->lookAtAbsAngles(init_gaze);
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

        //Store cartesian context
        iarm->storeContext(&startup_context_id);

        bool success = false;
        while(!success){
	        success=iarm->getPose(home_pose,home_rot);
        }
        yInfo("home orientation = (%s)",home_rot.toString(3,3).c_str());
	    yInfo("home position = (%s)",home_pose.toString(3,3).c_str());

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
        //iarm->restoreContext(startup_context_id);

        imgLPortIn.interrupt();
        imgRPortIn.interrupt();
        return true;
    }

    /***************************************************/
    bool close()
    {
        //iarm->restoreContext(startup_context_id);

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
            //mutex.lock();
            ImageOf<PixelRgb> *imgL=imgLPortIn.read();
            ImageOf<PixelRgb> *imgR=imgRPortIn.read();

            if (getCOG(*imgL, cogL)&&getCOG(*imgR, cogR))
            {
                roll(cogL,cogR);
                reply.addString("Yeah! I've made it roll like a charm!");
            }
            else
                reply.addString("I don't see any object!");
            //mutex.unlock();
        }
        else if (cmd=="home")
        {
            home();
            reply.addString("I've got the hard work done! Going home.");
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
        yInfo("update module begin..");
        // get fresh images
        ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // interrupt sequence detected
        if ((imgL==NULL) || (imgR==NULL)){
            return false;
            yInfo("one of the two images are null..");
        }

        // compute the center-of-mass of pixels of our color
        mutex.lock();
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
