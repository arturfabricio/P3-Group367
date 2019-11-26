// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
// Modified by group 367 ROB 3 of Aalborg University

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <string>
#include <stdlib.h>
#include <algorithm>
#include <myo/myo.hpp>
#include "SerialPort.h"	

using namespace std;
char output[MAX_DATA_LENGTH];
char incomingData[MAX_DATA_LENGTH];
char port[] = "\\\\.\\COM6";
std::string data;
std::string poseString;
vector<string> myArray;
int i = 0;

class DataCollector : public myo::DeviceListener {     // Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener provides several virtual functions for handling different kinds of events.
public:
	DataCollector()
		: onArm(false), isUnlocked(false), pitch_w(0), currentPose()
	{
	}

	// onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
	// as a unit quaternion.
	void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
	{
		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		pitch_w = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);

	}

	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose) {                                         // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example, making a fist, or not making a fist anymore.

		currentPose = pose;
		if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {                                         // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the Myo becoming locked.
			myo->unlock(myo::Myo::unlockHold);
			myo->notifyUserAction();                                                                         // Notify the Myo that the pose has resulted in an action, in this case changing the text on the screen. The Myo will vibrate.
		}
		else {
			myo->unlock(myo::Myo::unlockTimed);                                                              // Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses are being performed, but lock after inactivity.
		}
	}
	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,       // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their arm.
		myo::WarmupState warmupState) {
		onArm = true;
		whichArm = arm;
	}

	void onArmUnsync(myo::Myo* myo, uint64_t timestamp)                                                              // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after it recognized the arm.
	{
		onArm = false;
	}

	void onUnlock(myo::Myo* myo, uint64_t timestamp)                                                                 // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
	{
		isUnlocked = true;
	}

	void onLock(myo::Myo* myo, uint64_t timestamp)                                                                   // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
	{
		isUnlocked = false;
	}

	void print()															   // We define this function to print the current values that were updated by the on...() functions above.
	{
		if (onArm) {													  // Print out the lock state, the currently recognized pose, and which arm Myo is being worn on.
			std::cout << (pitch_w) << "  ";
			std::string poseString = currentPose.toString();

			if (pitch_w > 15) {
				std::cout << "NewMode" << std::endl;
				myArray.push_back("NewMode");
			}

			else {
				std::cout << poseString << std::endl;
				myArray.push_back(poseString);
			}
		}
		else { std::cout << "[?]" << std::endl; }							 // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.	
		std::cout << std::flush;
	}
	bool onArm;																			                       // These values are set by onArmSync() and onArmUnsync() above.
	myo::Arm whichArm;
	bool isUnlocked;                                                                                           // This is set by onUnlocked() and onLocked() above.
	int pitch_w;
	myo::Pose currentPose;
};

int main(int argc, char** argv)
{
	SerialPort arduino(port);
	if (arduino.isConnected()) {
		std::cout << "Connection established with the Arduino Board. Ready to go!" << std::endl << std::endl;
	}
	else {
		std::cout << "Error! Connection not established. Please verify port name." << std::endl << std::endl;
	}

	myo::Hub hub("com.example.hello-myo");                                                 // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when publishing your application. The Hub provides access to one or more Myos.
	std::cout << "Attempting to find a Myo..." << std::endl;                               // Now we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo immediately.

	myo::Myo* myo = hub.waitForMyo(10000);                                                 //we wait to find a Myo for 10 seconds. If nothing happens it'll exit with an error message.
	if (!myo) { throw std::runtime_error("Unable to find a Myo armband. Please try again."); }
	std::cout << "Connected established witha  Myo armband!" << std::endl << std::endl;    //Indicator that the connection with the Myo was established.

	DataCollector collector;                                                               // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
	hub.addListener(&collector);                                                           // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause Hub::run() to send events to all registered device listeners.


	while (1) {
		hub.run(2500 / 2);
		collector.print();

		std::string data = myArray[i];                                                     //We assign the string data with the current row of our vector that stores the poses. 

		char* charArray = new char[data.size() + 1];                                       //Here the string is converted into a char so it can be sent out.
		copy(data.begin(), data.end(), charArray);
		charArray[data.size()] = '\n';

		arduino.writeSerialPort(charArray, MAX_DATA_LENGTH);                               //It is written on the Serial Port.
		delete[] charArray;
		i++;
	}
}

