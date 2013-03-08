#include "robot.h"
#include "Python.h"
#include <string>
#include <iostream>
#include <stdio.h>

Robot::Robot(std::string modulePath, std::string robotName, RobotType type) :
		PythonInterface(modulePath) {
	Robot::name = robotName;
	Robot::type = type;
}

std::string Robot::getModuleName() {
	switch (Robot::type) {
	case (Robot::CareOBot):
		return "Robots.careobot";
	case (Robot::Sunflower):
		return "Robots.sunflower";
	default:
		return "Robots.robot";
	}
}

std::string Robot::getClassName() {
	switch (Robot::type) {
	case (Robot::CareOBot):
		return "CareOBot";
	case (Robot::Sunflower):
		return "Sunflower";
	default:
		return "Generic";
	}
}

PyObject* Robot::getConstructorArgs() {
	return PyString_FromString(Robot::name.c_str());
}

std::string Robot::setLight(int color[]) {
	char* m = strdup("setLight");
	char* f = strdup("([i,i,i])");
	//PyObject *pArgs = PyList_New(3);
	//PyList_SetItem(pArgs, 0, PyInt_FromLong(color[0]));
	//PyList_SetItem(pArgs, 1, PyInt_FromLong(color[1]));
	//PyList_SetItem(pArgs, 2, PyInt_FromLong(color[2]));

	PyObject *pValue = PyObject_CallMethod(getClassInstance(), m, f, color);
	//Py_DECREF(pArgs);

	if (pValue != NULL) {
		char* ret = PyString_AsString(pValue);
		Py_DECREF(pValue);
		return ret;
	} else {
		std::cout << "Error while calling method" << '\n';
		PyErr_Print();
		return NULL;
	}
}
