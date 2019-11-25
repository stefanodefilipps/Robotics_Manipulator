// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.3 rev4 on December 21st 2015

#include "v_repExtKUKA.h"
#include "/home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/include/luaFunctionData.h"
#include "/home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/include/v_repLib.h"
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <fstream>
#include <Eigen/Dense>
#include "Manipulator.h"
#include "path_trajectory.h"
#include "flaccoController.h"
#include "Task.h"
#include <vector>
#include <math.h>
#include <cmath>
#include <string>
ostream& operator<<(ostream& s, const vector<Eigen::MatrixXf>& m);
ostream& operator<<(ostream& s, const vector<int>& m);

#define _USE_MATH_DEFINES

using namespace Eigen;
using namespace std;

#ifdef _WIN32
	#ifdef QT_COMPIL
		#include <direct.h>
	#else
		#include <shlwapi.h>
		#pragma comment(lib, "Shlwapi.lib")
	#endif
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
	#include <unistd.h>
#endif /* __linux || __APPLE__ */

#ifdef __APPLE__
#define _stricmp strcmp
#endif

#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)

#define PLUGIN_VERSION 2 // 2 since version 3.2.1

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

// --------------------define global variables-----------------------------------------
//ifstream position;
//ifstream torque;

simInt joint_1;
simInt joint_2;
simInt joint_3;
simInt joint_4;
simInt joint_5;
simInt joint_6;
simInt joint_7;
simInt obstacle;

ofstream joint_angles;
ofstream joint_velocities;
ofstream switching_instants;
ofstream ee_task_error;
ofstream elbow_task_error;
ofstream cps_distances;

MatrixXf DH(7,4);
float d1 = 0.4;
float d2 = 0.39;
// d3 because the end effector is on top of the KUKA and not really the same as the GAZ DH table
float d3 = 0.078;

VectorXf q(7);

float t;
float lastT;
vector<int> switchingTimes;
bool switched{false};
float T;
float L;
float s;
float R;
float multiplier;
float phi;
float rho;
float v_max;
float alpha;
float init_elbow_task;

VectorXf p_in(3);
VectorXf p_fin(3);
VectorXf p_d(3);
MatrixXf J1;
MatrixXf J2(1,7);
MatrixXf J3(1,7);
MatrixXf J4(1,7);
MatrixXf b_d(3,1);
MatrixXf b1(3,1);
MatrixXf K(3,3);
MatrixXf b2(1,1);
MatrixXf b3(1,1);
MatrixXf b4(1,1);
VectorXf C(2);
//vector<MatrixXf> Ji;
//vector<VectorXf> bi;
VectorXf q_dot(7);
Vector3f obstPos;
Vector3f p_desired_ee;
vector<int> cPoints;
int always_present = 1;
float minimum_distance_presence = 0.4;

Manipulator* man;
PathTrajectory* path;
FlaccoController* controller;

simFloat* Nul;
simFloat* Clo;
simFloat* Clo2;
simFloat position[3];

simFloat  nul[3]={0,0,0};
simFloat  color[3]={0,255,0};
simFloat  color2[3]={255,0,0};

string path_;

// --------------------------------------------------------------------------------------
// simExtSkeleton_getSensorData: an example of custom Lua command
// --------------------------------------------------------------------------------------
#define LUA_GETSENSORDATA_COMMAND "simExtSkeleton_getSensorData" // the name of the new Lua command

const int inArgs_GETSENSORDATA[]={ // Decide what kind of arguments we need
	3, // we want 3 input arguments
    sim_lua_arg_int,0, // first argument is an integer
    sim_lua_arg_float|sim_lua_arg_table,3, // second argument should be a table of at least 3 float values (use 0 instead of 3 for a table of random size)
    sim_lua_arg_int|sim_lua_arg_table,2, // third argument should be a table of at least 2 integer values (use 0 instead of 2 for a table of random size)
};

void LUA_GETSENSORDATA_CALLBACK(SLuaCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getSensorData")
	p->outputArgCount=0;
	CLuaFunctionData D;
	// If successful the command will return an interger (result), a float table of size 3 (data), and a float (distance). If the command is not successful, it will not return anything
	bool commandWasSuccessful=false;
	int returnResult;
	std::vector<float> returnData;
	float returnDistance;
	if (D.readDataFromLua(p,inArgs_GETSENSORDATA,inArgs_GETSENSORDATA[0],LUA_GETSENSORDATA_COMMAND))
	{ // above function reads in the expected arguments. If the arguments are wrong, it returns false and outputs a message to the simulation status bar
		std::vector<CLuaFunctionDataItem>* inData=D.getInDataPtr();

		int sensorIndex=inData->at(0).intData[0]; // the first argument
		std::vector<float>& floatParameters=inData->at(1).floatData; // the second argument
		std::vector<int>& intParameters=inData->at(2).intData; // the third argument

		// Now you can do something with above's arguments. For example:
		if ((sensorIndex>=0)&&(sensorIndex<10))
		{
			commandWasSuccessful=true;
			returnResult=1;
			returnData.push_back(1.0f);
			returnData.push_back(2.0f);
			returnData.push_back(3.0f);
			returnDistance=59.0f;
		}
		else
			simSetLastError(LUA_GETSENSORDATA_COMMAND,"Invalid sensor index."); // output an error message to the simulator's status bar
	}
	if (commandWasSuccessful)
	{ // prepare the return values:
		D.pushOutData(CLuaFunctionDataItem(returnResult));
		D.pushOutData(CLuaFunctionDataItem(returnData));
		D.pushOutData(CLuaFunctionDataItem(returnDistance));
	}
	D.writeDataToLua(p);
}
// --------------------------------------------------------------------------------------


// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
	// Dynamically load and bind V-REP functions:
	// ******************************************
	// 1. Figure out this plugin's directory:
	char curDirAndFile[1024];
#ifdef _WIN32
	#ifdef QT_COMPIL
		_getcwd(curDirAndFile, sizeof(curDirAndFile));
	#else
		GetModuleFileName(NULL,curDirAndFile,1023);
		PathRemoveFileSpec(curDirAndFile);
	#endif
#elif defined (__linux) || defined (__APPLE__)
	getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

	std::string currentDirAndPath(curDirAndFile);
	// 2. Append the V-REP library's name:
	std::string temp(currentDirAndPath);
#ifdef _WIN32
	temp+="\\v_rep.dll";
#elif defined (__linux)
	temp+="/libv_rep.so";
#elif defined (__APPLE__)
	temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
	// 3. Load the V-REP library:
	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib==NULL)
	{
		std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************

	// Check the version of V-REP:
	// ******************************************
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<30200) // if V-REP version is smaller than 3.02.00
	{
		std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************

	std::vector<int> inArgs;

	// Register the new Lua command "simExtSkeleton_getSensorData":
	CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_GETSENSORDATA,inArgs);
	simRegisterCustomLuaFunction(LUA_GETSENSORDATA_COMMAND,strConCat("number result,table data,number distance=",LUA_GETSENSORDATA_COMMAND,"(number sensorIndex,table_3 floatParameters,table_2 intParameters)"),&inArgs[0],LUA_GETSENSORDATA_CALLBACK);

	return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
	// Here you could handle various clean-up tasks

	unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
	// Keep following 5 lines at the beginning and unchanged:
	static bool refreshDlgFlag=true;
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
	void* retVal=NULL;

	// Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
	// For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
	// in the V-REP user manual.

	if (message==sim_message_eventcallback_refreshdialogs)
		refreshDlgFlag=true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

	if (message==sim_message_eventcallback_menuitemselected)
	{ // A custom menu bar entry was selected..
		// here you could make a plugin's main dialog visible/invisible
	}

	if (message==sim_message_eventcallback_instancepass)
	{	// This message is sent each time the scene was rendered (well, shortly after) (very often)
		// It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

		int flags=auxiliaryData[0];
		bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message 
		bool instanceSwitched=((flags&64)!=0);

		if (instanceSwitched)
		{
			// React to an instance switch here!!
		}

		if (sceneContentChanged)
		{ // we actualize plugin objects for changes in the scene

			//...

			refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
		}
	}

	if (message==sim_message_eventcallback_mainscriptabouttobecalled)
	{ // The main script is about to be run (only called while a simulation is running (and not paused!))
		if(t <= path->get_end_condition()){
			// To the controller we need to pass the stack of Jacobian and tasks velocities, The world positions of the control points,
			// the world obstacles position and the feedforward term for the ee position for the cartesian control scheme
			// For now the only control point is simply the end effector 
			vector<Vector3f> cps_positions = man->controlPoints();
            J1 = man->jacobian(q);
            //b1 = path->p_dot_d(t) + controller->eeRepulsiveVelocity(man->dKin(q),0);
            //p_desired_ee = p_desired_ee + T * b1;
            //b1 = b1 + K * (p_desired_ee - man->dKin(q));
            b1 = path->p_dot_d(t) + K * (path->p_d(t) - man->dKin(q));
            b1 = b1 + controller->eeRepulsiveVelocity(man->dKin(q),0);
            // Remeber that in the stack of Jacobians each Jacobian needs to have 7 column because we have 7 degree of freedom
            // The partial Jacobian computed up to p joint has dimension 3xp but i need to extend it to 3x7 adding to it a 3x(7-p) submatrix
            // of all 0s since the kinematics of p point doesn't depend on the 7-p successive joint.
            // Moreover, since the jacobian of the control points need to be normalized by the direction vector 1x3 i will obtain a final matrix
            // of 1x7 where all the 7-p elements will be surly 0s. Therefore It is the same as doing computation with 3xp matrix and then assing the 
            // resulting submatrix to the correct part of the 1x7 final matrix
            int number_present = 0;
            vector<MatrixXf> Ji{J1,J2};
	        vector<MatrixXf> bi{b1,b2};
            if(always_present == 1){
            	J3 << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
	            J3.block(0,0,1,cPoints[1]) = controller->projectJ(man->jacobian(q,cPoints[1]),cps_positions[1]);
	            b3 << controller->projectP(cps_positions[1]);
	            Ji.push_back(J3);
	            bi.push_back(b3);
	            J4 << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
	            J4.block(0,0,1,cPoints[2]) = controller->projectJ(man->jacobian(q,cPoints[2]),cps_positions[2]);
	            b4 << controller->projectP(cps_positions[2]);
	            Ji.push_back(J4);
	            bi.push_back(b4);
            }
            else{
            	if(controller->eeDis(cps_positions[1]) <= minimum_distance_presence){
            		J3 << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
		            J3.block(0,0,1,cPoints[1]) = controller->projectJ(man->jacobian(q,cPoints[1]),cps_positions[1]);
		            b3 << controller->projectP(cps_positions[1]);
		            Ji.push_back(J3);
		            bi.push_back(b3);
		            number_present++;
            	}
            	if(controller->eeDis(cps_positions[2]) <= minimum_distance_presence){
            		J4 << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
		            J4.block(0,0,1,cPoints[2]) = controller->projectJ(man->jacobian(q,cPoints[2]),cps_positions[2]);
		            b4 << controller->projectP(cps_positions[2]);
		            Ji.push_back(J4);
		            bi.push_back(b4);
		            number_present++;
            	}
            	cout << "NUMBER PRESENT " << number_present << "\n";
            }
            Task<MatrixXf> stack_Ji{Ji};
            Task<MatrixXf> stack_bi{bi};

            /*J3 << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
            J3.block(0,0,1,cPoints[1]) = controller->projectJ(man->jacobian(q,cPoints[1]),cps_positions[1]);
            b3 << controller->projectP(cps_positions[1]);
            J4 << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
            J4.block(0,0,1,cPoints[2]) = controller->projectJ(man->jacobian(q,cPoints[2]),cps_positions[2]);
            b4 << controller->projectP(cps_positions[2]);
            vector<MatrixXf> Ji{J1,J2,J3,J4};
            vector<MatrixXf> bi{b1,b2,b3,b4};
            Task<MatrixXf> stack_Ji{Ji};
            Task<MatrixXf> stack_bi{bi};*/
            // Reorder the Jacobian and the Velocity task. I am calling twice the taskReorder function, but same positions so i will have same final ordering
            //if(t - lastT >= 5*T || !switched) { // The outermost if needs to be deleted so we can still keep info about switching instanties
			switched = controller->taskReorder(stack_Ji, cps_positions);
			controller->taskReorder(stack_bi, cps_positions);
			lastT = t;
			if(switched){
				switchingTimes.push_back(round(lastT/T));
				switching_instants << switchingTimes.back() << ";\n";
			}
			//}
            q_dot = controller->control(stack_Ji.getStack(),stack_bi.getStack());
            for (int i = 0; i < 6; ++i)
            {
            	joint_angles << q(i);
            	joint_angles << ";";
            	joint_velocities << q_dot(i);
            	joint_velocities << ";";
            }
            joint_angles << q(6);
        	joint_angles << "\n";
        	joint_velocities << q_dot(6);
        	joint_velocities << "\n";
        	ee_task_error << sqrt((path->p_d(t) - man->dKin(q)).transpose()*(path->p_d(t) - man->dKin(q))) << "\n";
        	elbow_task_error << init_elbow_task - (q[1]+q[2]+q[3]+q[4]) << "\n";
        	cps_distances << controller->eeDis(cps_positions[0]) << ";" << controller->eeDis(cps_positions[1]) << ";" << controller->eeDis(cps_positions[2]) << "\n";
        	//switching_instants << switchingTimes.back() << ";\n";
            q = man->update_configuration(q_dot,T);
            simSetJointPosition(joint_1,q(0));
            simSetJointPosition(joint_2,q(1));
            simSetJointPosition(joint_3,q(2));
            simSetJointPosition(joint_4,q(3));
            simSetJointPosition(joint_5,q(4));
            simSetJointPosition(joint_6,q(5));
            simSetJointPosition(joint_7,q(6));
            t = t + T;
        }

	}

	if (message==sim_message_eventcallback_simulationabouttostart)
    { // Simulation is about to start


        cout<<"///////////////////////////////////\n";
        cout<<"New Static Simulation is about to start\n";

        joint_1= simGetObjectHandle("Revolute_joint1");
        joint_2= simGetObjectHandle("Revolute_joint2");
        joint_3= simGetObjectHandle("Revolute_joint3");
        joint_4= simGetObjectHandle("Revolute_joint4");
        joint_5= simGetObjectHandle("Revolute_joint5");
        joint_6= simGetObjectHandle("Revolute_joint6");
        joint_7= simGetObjectHandle("Revolute_joint7");
        obstacle = simGetObjectHandle("SphereObs");
        float z_offset;
        simGetObjectPosition(joint_1,-1, position);
        z_offset = position[2];
        cout << z_offset;
        simInt code = simGetObjectPosition(obstacle,-1, position);
        cout << code;
        obstPos << position[0], position[1], position[2];

        // aplha : to shape the sigmoid curve of the magnitude
		// rho: the half length of the control point bounding box
		// v_max: The maximum velocity in module that the repulsive vector can get

        cout << "Insert alpha\n";
        cin >> alpha;
        cout << "Insert rho\n";
        cin >> rho;
        cout << "Insert v_max\n";
        cin >> v_max;
        cout << "Insert the path type\n";
        cin >> path_;
        string response;
        cout << "All control points always present?\n";
        cin >> response;
        if(response == "yes"){
        	always_present = 1;
        }
        if(response == "no"){
        	always_present = 0;
        	cout << "Insert minimum distance of presence\n";
        	cin >> minimum_distance_presence;
        }
        t = 0.0;
        lastT = t;
		T = 0.005;
		q << 0,-M_PI_2,0,-M_PI_2,0,0,0;

        DH << M_PI_2, 0, 0, 1,
         -M_PI_2, 0, 0, 1,
         -M_PI_2, 0, d1,1,
          M_PI_2, 0, 0, 1,
          M_PI_2, 0, d2,1,
         -M_PI_2, 0, 0, 1,
               0, 0, d3, 1;
        man = new Manipulator(DH,q,z_offset);
        cPoints = {7,4,2};
		man->setCtrPtsJoints(cPoints);
        VectorXf ks(3);
        ks << 20.0,20.0,20.0;
        K = ks.asDiagonal();
        vector<Vector3f> obstacles{obstPos};
        controller = new FlaccoController(alpha,rho,v_max,ks,obstacles);
        joint_angles.open("joint_angles.txt");
        joint_velocities.open("joint_velocities.txt");
        switching_instants.open("switching_instants.txt");
        ee_task_error.open("ee_task_error.txt");
        elbow_task_error.open("elbow_task_error.txt");
        cps_distances.open("cps_distances.txt");
        init_elbow_task = q[0] + q[1] + q[2] + q[3];
        if(path_.compare("linear") == 0){

	        p_in << man->dKin(q)[0],
	                man->dKin(q)[1],
	                man->dKin(q)[2];

	        cout << p_in;

	        p_desired_ee = p_in;

	        float x;
	        float y;
	        float z;
	        cout << "Insert x of final point\n";
	        cin >> x;
	        cout << "Insert y of final point\n";
	        cin >> y;
	        cout << "Insert z of final point\n";
	        cin >> z;
	        p_fin << x,
	                 y,
	                 z;

	        vector<VectorXf> in = {p_in,p_fin};
	        path = new PathTrajectory("linear",in);

	        J2 << 1.0,1.0,1.0,1.0,0.0,0.0,0.0;

	        b2 << 0;
        }
        if(path_.compare("circular") == 0){

        	float v;

        	cout << "Insert Radius";
        	cin >> R;
        	cout << "Insert Multiplier";
        	cin >> multiplier;
        	cout << "Insert phi intial offset";
        	cin >> phi;
        	cout << "Insert x Center";
        	cin >> v;
        	C(0) = v;
        	cout << "Insert y Center";
        	cin >> v;
        	C(1) = v;
        	vector<VectorXf> in = {C};
        	path = new PathTrajectory("circular",in, R, multiplier, phi);
        	p_desired_ee =  path->p_d(0);
        	J2 << 1.0,1.0,1.0,1.0,0.0,0.0,0.0;

	        b2 << 0;
        }


        ///for plot///////////
        Clo=color;
        Nul=nul;

        // task
        //draw=simAddDrawingObject(sim_drawing_lines,2.0,0.0,-1,1000000,Clo2,Nul,Clo2,Clo2);

        // end effector
        //Clo2=color2;
        //draw_real=simAddDrawingObject(sim_drawing_lines,2.0,0.0,-1,1000000,Clo2,Nul,Clo2,Clo2);

        // task1
        /*
        float cx=0,rx=-0.4,cy=0,ry=0.3,h=0.7680;//0.578+0.1;

        T02[0]=cx+rx*cos(0);
        T02[1]=cy+ry*sin(0);
        T02[2]=h;
        */

        // TASK2
        /*
        float L=0.4;
        T02[0]= 0.3376;
        T02[1]= 0.3376;
        T02[2]=0.7105;
*/

       // for(double j=0.005;j<=1.00;j=j+0.005)
       // {

            // task1
            /*
            T12[0]=cx+rx*cos(2*pi*j);
            T12[1]=cy+ry*sin(2*pi*j);
            T12[2]=h;
            */
/*
            //task 2
            T12[0]= 0.3376-(L*j);
            T12[1]= 0.3376;
            T12[2]= 0.7105;

            T2[0]=T02[0];
            T2[1]=T02[1];
            T2[2]=T02[2];
            T2[3]=T12[0];
            T2[4]=T12[1];
            T2[5]=T12[2];
            simAddDrawingObjectItem(draw_real,T2);
            T02[0]=T12[0];
            T02[1]=T12[1];
            T02[2]=T12[2];
        }
*/



        /*
        for(int j=0;j<6;++j)
            position>>q0(j);

        simSetJointPosition(joint_1,q0(0));
        simSetJointPosition(joint_2,q0(1));
        simSetJointPosition(joint_3,q0(2));
        simSetJointPosition(joint_4,-q0(3));
        simSetJointPosition(joint_5,q0(4));
        simSetJointPosition(joint_6,q0(5));
        simSetJointPosition(joint_7,0);

        num_row++;
        */


        /* with high velocities can not acheive dynamic simulations
        cout<<"///////////////////////////////////\n";
        cout<<"Dynamic Simulation is about to start\n";
        torque.open("/home/khaled/Desktop/ABRO_PhD/MasterCourses/FirstYear/Optimal_control-2015-11-06/TheProject/ToKhaled_Nov2015/ToKhaled_Nov2015/project/TOPP-1.4/tests/NewKuka_t.txt");

        joint_1= simGetObjectHandle("LBR4p_joint1");
        joint_2= simGetObjectHandle("LBR4p_joint2");
        joint_3= simGetObjectHandle("LBR4p_joint3");
        joint_4= simGetObjectHandle("LBR4p_joint4");
        joint_5= simGetObjectHandle("LBR4p_joint5");
        joint_6= simGetObjectHandle("LBR4p_joint6");
        joint_7= simGetObjectHandle("LBR4p_joint7");

        q0<<0.7,0.8,0.4,0.5,0.6,0.5,0.5;//starting configuration

        simSetJointPosition(joint_1,q0(0));
        simSetJointPosition(joint_2,q0(1));
        simSetJointPosition(joint_3,q0(2));
        simSetJointPosition(joint_4,q0(3));
        simSetJointPosition(joint_5,q0(4));
        simSetJointPosition(joint_6,q0(5));
        simSetJointPosition(joint_7,q0(6));

        dq<<9999,9999,9999,9999,9999,9999,9999;
        */


	}

	if (message==sim_message_eventcallback_simulationended)
	{ // Simulation just ended

        //for static simulation
        cout<<"New Simulation just ended\n";
        joint_angles.close();
        joint_velocities.close(); 
        switching_instants.close();
        ee_task_error.close();
        elbow_task_error.close();
        cps_distances.close();
        /*
        torque.close();
        cout<<"Simulation just ended\n";
        num_row=0;
        */


	}

	if (message==sim_message_eventcallback_moduleopen)
	{ // A script called simOpenModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(strcasecmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
			// we arrive here only at the beginning of a simulation
        }
	}

	if (message==sim_message_eventcallback_modulehandle)
	{ // A script called simHandleModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(strcasecmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running
        }
	}

	if (message==sim_message_eventcallback_moduleclose)
	{ // A script called simCloseModule (by default the main script). Is only called during simulation.
        if ( (customData==NULL)||(strcasecmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
        {
			// we arrive here only at the end of a simulation
        }
	}

	if (message==sim_message_eventcallback_instanceswitch)
	{ // We switched to a different scene. Such a switch can only happen while simulation is not running

	}

	if (message==sim_message_eventcallback_broadcast)
	{ // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

	}

	if (message==sim_message_eventcallback_scenesave)
	{ // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

	}

	// You can add many more messages to handle here

	if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
	{ // handle refresh of the plugin's dialogs
		// ...
		refreshDlgFlag=false;
	}

	// Keep following unchanged:
	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	return(retVal);
}

