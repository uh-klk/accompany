#include "history.h"
#include "Python.h"
#include <string>
#include <iostream>
#include <exception>
#include <stdio.h>

PythonInterface::PythonInterface(std::string modulePath) {
	PythonInterface::modulePath = modulePath;
	PythonInterface::pInstance = NULL;
	if (!Py_IsInitialized()) {
		Py_Initialize();
		std::cout << "Python Initialized" << std::endl;
	}
}

PythonInterface::~PythonInterface() {
	if (pInstance != NULL) {
		Py_DECREF(pInstance);
	}

	Py_Finalize();
}

PyObject* PythonInterface::getConstructorArgs() {
	return NULL;
}

PyObject* PythonInterface::getClassInstance() {
	if (pInstance == NULL) {
		if (!modulePath.empty()) {

			PyRun_SimpleString("import sys");
			PyRun_SimpleString(
					("sys.path.append(\"" + std::string(modulePath) + "\")").c_str());
		}

		std::string modName = getModuleName();
		PyObject *pName = PyString_FromString(modName.c_str());
		PyObject *pModule = PyImport_Import(pName);
		Py_DECREF(pName);

		char* fileName = PyModule_GetFilename(pModule);
		char* argv[1] = { fileName };
		PySys_SetArgvEx(1, argv, 0);

		PyObject *pDict = PyModule_GetDict(pModule);
		Py_DECREF(pModule);

		PyObject *pClass = PyDict_GetItemString(pDict, getClassName().c_str());
		Py_DECREF(pDict);

		PyObject *pClassArgs = getConstructorArgs();

		pInstance = PyObject_CallObject(pClass, pClassArgs);
		Py_DECREF(pClass);
		if (pClassArgs != NULL) {
			Py_DECREF(pClassArgs);
		}
	}

	return pInstance;
}

PyObject* PythonInterface::callMethod(std::string methodName,
		std::string arg1) {
	char* f = strdup("(s)");
	char* m = strdup(methodName.c_str());
	PyObject *pValue = PyObject_CallMethod(getClassInstance(), m, f,
			arg1.c_str());

	if (pValue != NULL) {
		return pValue;
	} else {
		std::cout << "Error while calling method" << '\n';
		PyErr_Print();
		return NULL;
	}
}
