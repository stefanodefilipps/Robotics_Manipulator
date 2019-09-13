#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <boost/lexical_cast.hpp>

#define _USE_MATH_DEFINES

using namespace std;
using boost::lexical_cast;

extern "C" {
    #include "/home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/remoteApi/extApi.h"
    #include "/home/stefano/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04/programming/remoteApi/extApiPlatform.h"
}

int main(int argc, char const *argv[])
{
	/* code */
	int clientID=simxStart((simxChar*)"127.0.0.1",19999,true,true,2000,5);
	if (clientID==-1){
		printf("Could not connect to V-REP remote API server\n");
		simxFinish(clientID);
	}else{
		printf("Connected to remote API server\n");
		cout << "5R DOF EXAMPLE:\n\n";
		vector<simxInt*> jointHandles = {new simxInt,new simxInt,new simxInt,new simxInt,new simxInt,new simxInt,new simxInt};
		for (int i = 1; i < 6; ++i){
			/* code */
				simxGetObjectHandle(clientID,(simxChar*)("Revolute_joint" + to_string(i)).c_str(),jointHandles[i-1],simx_opmode_blocking);
			}
		ifstream file("/home/stefano/Robotic2_project/plot_result/results_plot/5R_planar_circular/data_joints_file.txt");
		if (file.is_open()) {
		    string line;
		    string delimiter = ";";
		    while (getline(file, line)) {
		        size_t pos = 0;
				string token;
				int count = 0;
				vector<float> angles;
				while ((pos = line.find(delimiter)) != std::string::npos) {
				    token = line.substr(0, pos);
				    if (count < 5)
				    {
				    	line.erase(0, pos + delimiter.length());
				    	angles.push_back(lexical_cast<float>(token));
				    }
				    count = count + 1;
				}
				for (int i = 0; i < 5; ++i)
				{
					simxSetJointPosition(clientID,*jointHandles[i],angles[i],simx_opmode_oneshot);
				}
				extApi_sleepMs(10);
		    }
		}
		/*while (extApi_getTimeDiffInMs(startTime) < 30000){
			for (int i = 0; i < 7; ++i){
				cout << *jointPositions[i] << endl;
			}
			for (int i = 0; i < 7; ++i)
			{
				cout << targetPos1[i] << endl;
				simxSetJointTargetPosition(clientID,*jointHandles[i],targetPos1[i],simx_opmode_streaming);
			}
			cout << "pos1" << endl;
			extApi_sleepMs(2000);
			for (int i = 0; i < 7; ++i)
			{
				cout << targetPos1[i] << endl;
				simxSetJointTargetPosition(clientID,*jointHandles[i],targetPos2[i],simx_opmode_streaming);
			}
			cout << "pos2" << endl;
			extApi_sleepMs(2000);
			for (int i = 0; i < 7; ++i)
			{
				cout << targetPos1[i] << endl;
				simxSetJointTargetPosition(clientID,*jointHandles[i],targetPos3[i],simx_opmode_streaming);
			}
			cout << "pos3" << endl;
			extApi_sleepMs(2000);
		}*/
		simxFinish(clientID);
	}
	return 0;
}
	

		

	