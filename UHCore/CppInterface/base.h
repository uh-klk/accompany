#ifndef PYTHON_INTERFACE
#define PYTHON_INTERFACE
#include "Python.h"
#include <string>
#include <vector>

class PythonInterface {
public:
	PythonInterface(std::string modulePath);
	virtual ~PythonInterface();
protected:
	virtual std::string getModuleName() = 0;
	virtual std::string getClassName() = 0;
	virtual PyObject* getConstructorArgs();
	PyObject* callMethod(std::string methodName, std::string arg1);
	PyObject* getClassInstance();

private:
	std::string modulePath;
	PyObject* pInstance;
};

#endif //PYTHON_INTERFACE
