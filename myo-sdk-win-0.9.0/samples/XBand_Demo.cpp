// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.

#include <myo/myo.hpp>
#include <fstream>
#include <vector>

// Classes that inherit from myo::DeviceListener can be used to receive events from Myo devices. DeviceListener
// provides several virtual functions for handling different kinds of events. If you do not override an event, the
// default behavior is to do nothing.
class DataCollector : public myo::DeviceListener {
public:

	int limitIterator = 0;
	double rl = 0;
	double rh = 0;
	double pl = 0;
	double ph = 0;
	double yl = 0;
	double yh = 0;
	double gxl = 0;
	double gxh = 0;
	double gyl = 0;
	double gyh = 0;
	double gzl = 0;
	double gzh = 0;
	double gml = 0;
	double gmh = 0;
	double r, p, y;
	int i = 0;

	std::vector<std::vector<double> > out;

	DataCollector()
		: onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
	{
	}

	// onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
	void onUnpair(myo::Myo* myo, uint64_t timestamp)
	{
		// We've lost a Myo.
		// Let's clean up some leftover state.
		roll_w = 0;
		pitch_w = 0;
		yaw_w = 0;
		onArm = false;
		isUnlocked = false;
	}

	std::vector< std::vector <double> > getData() {

		return out;

	}

	std::vector<double> matrix() {
		std::vector<double> output;
		output.push_back(p);
		output.push_back(r);
		output.push_back(y);
		output.push_back(gyx);
		output.push_back(gyy);
		output.push_back(gyz);
		output.push_back(gym);
		return output;
	}

	void findRange() {


		if (r > rh) {
			rh = r;
		}
		if (r < rl) {
			rl = r;
		}
		if (p > ph) {
			ph = p;
		}
		if (p < pl) {
			pl = p;
		}
		if (y > yh) {
			yh = y;
		}
		if (y < yl) {
			yl = y;
		}
		if (gyx > gxh) {
			gxh = gyx;
		}
		if (gyx < gxl) {
			gxl = gyx;
		}
		if (gyy > gyh) {
			gyh = gyy;
		}
		if (gyy < gyl) {
			gyl = gyy;
		}
		if (gyz > gzh) {
			gzh = gyz;
		}
		if (gyz < gzl) {
			gzl = gyz;
		}
		if (gym > gmh) {
			gmh = gym;
		}
		if (gym < gml) {
			gml = gym;
		}
		std::vector<double> output;
		output.push_back(p);
		output.push_back(r);
		output.push_back(y);
		output.push_back(gyx);
		output.push_back(gyy);
		output.push_back(gyz);
		output.push_back(gym);
		out.push_back(output);

	}

	void publish(std::string fname) {
		std::ofstream myfile;
		std::string name;
		myfile.open(fname);
		std::cout << "What is this exercise called?\n";
		myfile << name << std::endl;
		myfile << rh << " " << rl << " " << ph << " " << pl << " " << yh << " " << yl << " " << std::endl;
		myfile.close();

	}

	void pull(std::string fname, std::string name) {
		std::ifstream myfile;
		myfile.open(fname);
		std::string num;
		std::string line;
		std::string::size_type sz;
		int found;
		while (myfile.is_open() && getline(myfile, line)) {
			if (line == name) {
				getline(myfile, line);

				found = line.find(' ');
				num = line.substr(0, found);
				rh = std::stod(num, &sz);

				num = line.substr(found + 1, line.find(' ', found + 1));
				found = line.find(' ', found + 1);
				rl = std::stod(num, &sz);

				num = line.substr(found + 1, line.find(' ', found + 1));
				found = line.find(' ', found + 1);
				ph = std::stod(num, &sz);

				num = line.substr(found + 1, line.find(' ', found + 1));
				found = line.find(' ', found + 1);
				pl = std::stod(num, &sz);

				num = line.substr(found + 1, line.find(' ', found + 1));
				found = line.find(' ', found + 1);
				yh = std::stod(num, &sz);

				num = line.substr(found + 1, line.find(' ', found + 1));
				found = line.find(' ', found + 1);
				yl = std::stod(num, &sz);
				myfile.close();
			}
			i++;
		}
		if (myfile.is_open()) {
			myfile.close();
		}
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
		double roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
			1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
		double pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
		double yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
			1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
		r = roll;
		p = pitch;
		y = yaw;
		// Convert the floating point angles in radians to a scale from 0 to 18.
		roll_w = static_cast<int>((roll + (float)M_PI) / (M_PI * 2.0f) * 18);
		pitch_w = static_cast<int>((pitch + (float)M_PI / 2.0f) / M_PI * 18);
		yaw_w = static_cast<int>((yaw + (float)M_PI) / (M_PI * 2.0f) * 18);
	}
	void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& vec) {
		gyx = vec.x();
		gyy = vec.y();
		gyz = vec.z();
		gym = vec.magnitude();
	}

	myo::Pose getPose() {
		return currentPose;
	}

	// onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
	// making a fist, or not making a fist anymore.
	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
	{
		currentPose = pose;

		if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
			// Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
			// Myo becoming locked.
			myo->unlock(myo::Myo::unlockHold);

			// Notify the Myo that the pose has resulted in an action, in this case changing
			// the text on the screen. The Myo will vibrate.
			myo->notifyUserAction();
		}
		else {
			// Tell the Myo to stay unlocked only for a short period. This allows the Myo to stay unlocked while poses
			// are being performed, but lock after inactivity.
			myo->unlock(myo::Myo::unlockTimed);
		}


		myo->unlock(myo::Myo::unlockHold);

	}

	// onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
	// arm. This lets Myo know which arm it's on and which way it's facing.
	void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
		myo::WarmupState warmupState)
	{
		onArm = true;
		whichArm = arm;
	}

	// onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
	// it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
	// when Myo is moved around on the arm.
	void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
	{
		onArm = false;
	}

	// onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
	void onUnlock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = true;
	}

	// onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
	void onLock(myo::Myo* myo, uint64_t timestamp)
	{
		isUnlocked = false;
	}

	// These values are set by onArmSync() and onArmUnsync() above.
	bool onArm;
	myo::Arm whichArm;

	// This is set by onUnlocked() and onLocked() above.
	bool isUnlocked;

	// These values are set by onOrientationData() and onPose() above.
	int roll_w, pitch_w, yaw_w;
	float gyx, gyy, gyz, gym;
	myo::Pose currentPose;
};

bool checkBound(std::vector <std::vector<double> > pattern, std::vector<double> check, double oerror, double gerror, int freq) {
	//p r y x y z m
	bool output = true;
	bool output2 = true;
	bool output3 = false;
	int i = 0;
	while (i < pattern.size() - 2) {
		int j = 0;
		while (j < 3) {
			double d = pattern[i][j];
			output = fabs(d - check[j]) < oerror;
			std::cout << d << " " << check[j] << std::endl;
			int ii = 1;
			while (ii < freq) {
				d = pattern[i + ii][j];
				output = output || fabs(d - check[j]) < oerror;
				++ii;
			}
			if (!output) {
				std::cout << std::endl << "wrong fukkin position";
			}
			ii = 1;
			double g = pattern[i][j + 3];
			output2 = fabs(g - check[j + 3]) < gerror;
			while (ii < freq) {
				g = pattern[i + ii][j + 3];
				output2 = output2 || fabs(g - check[j + 3]) < gerror;
			}



			if (!output2) {
				std::cout << std::endl << "wrong fukkin sped";
			}
			if (!(output && output2) || output3) {
				output3 = false;

			}
			else {
				output3 = true;
			}
			j++;
		}
		i++;
	}
	return output3;

}

int main(int argc, char** argv)
{


	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {

		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.example.hello-myo");

		std::cout << "Attempting to find a Myo..." << std::endl;

		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub.waitForMyo(10000);

		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}

		// We've found a Myo.
		std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector;

		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);

		//declaring variables for fine tuning of sensitivity and console refresh rate
		int refreshRate = 3;
		int measureRange;
		double oError, gError;
		std::vector <std::vector<double> > checks;
		bool init = false;

		oError = 15;
		gError = 200;
		std::string uinput;
		std::cout << std::endl << "Would you like to open a pre-existing exercise (y/n)?" << std::endl;
		std::cin >> uinput;
		if (uinput == "y") {
			std::cout << std::endl << "What is the exercise name?" << std::endl;
			std::cin >> uinput;
			collector.pull("Exercises.txt", uinput);
		}

		std::cout << std::endl << "How much time is needed for the exercise? (seconds)" << std::endl;
		std::cin >> measureRange;
		measureRange = measureRange * 30;

		std::cout << "Please do the exercise in...";
		std::cout << std::endl << "3";
		hub.run(1000);
		std::cout << std::endl << "2";
		hub.run(1000);
		std::cout << std::endl << "1";
		hub.run(1000);
		std::cout << std::endl << "go!";

		while (!init) {

			collector.findRange();

			if (measureRange <= 0) {
				init = true;
			}

			hub.run(1000 / 30);
			--measureRange;
		}

		checks = collector.getData();

		std::cout << std::endl << "Starting trainer!" << std::endl;

		int i = 0;
		int e = 0;

		while (1) {
			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
			//std::cout << std::endl << collector.gyx << " " << collector.gyy << " " << collector.gyz << std::endl;

			if (!checkBound(checks, collector.matrix(), oError, gError, measureRange * 30)) {
				++e;
			}

			else {
				--e;
			}

			if (e > 50) {
				myo->vibrate(myo::Myo::vibrationShort);
				e = 0;
			}


			if (collector.r > collector.rh || collector.r < collector.rl || collector.p > collector.ph || collector.p < collector.pl || collector.y > collector.yh || collector.y < collector.yl) {
				myo->vibrate(myo::Myo::vibrationShort);
				std::cout << std::endl << "wrong pos" << std::endl;
			}
			if (collector.currentPose.type() == libmyo_pose_wave_in || collector.currentPose.type() == libmyo_pose_wave_out) {
				myo->vibrate(myo::Myo::vibrationShort);
			}



			hub.run(1000 / 3);

		}

		// If a standard exception occurred, we print out its message and exit.
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}
