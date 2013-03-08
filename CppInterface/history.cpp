#include "history.h"
#include "Python.h"
#include <string>
#include <iostream>
#include <stdio.h>

ActionHistory::ActionHistory(std::string modulePath) :
		PythonInterface(modulePath) {
	}

std::string ActionHistory::getModuleName() {
	return "history";
}
std::string ActionHistory::getClassName() {
	return "ActionHistory";
}

bool ActionHistory::cancelPollingHistory(std::string ruleName) {
	PyObject* pValue = callMethod("cancelPollingHistory", ruleName);
	bool ret = false;
	if (PyObject_IsTrue(pValue)) {
		ret = true;
	}

	Py_DECREF(pValue);

	return ret;
}

char* ActionHistory::addPollingHistory(std::string ruleName, float delaySeconds) {

	char* m = strdup("addPollingHistory");
	char* f = strdup("(sf)");
	PyObject *pValue = PyObject_CallMethod(
			getClassInstance(),
			m,
			f,
			ruleName.c_str(),
			delaySeconds);

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

void ActionHistory::addHistoryAsync(std::string ruleName) {
	PyObject* pValue = callMethod("addHistoryAsync", ruleName);
	Py_DECREF(pValue);
}

bool ActionHistory::addHistory(std::string ruleName) {
	PyObject* pValue = callMethod("addHistory", ruleName);
	bool ret = false;
	if (PyObject_IsTrue(pValue)) {
		ret = true;
	}

	Py_DECREF(pValue);

	return ret;
}
