#ifndef ACTION_HISTORY
#define ACTION_HISTORY
#include "base.h"
#include <string>

class Robot: public PythonInterface {
public:
	enum RobotType {
		CareOBot, Sunflower
	};

	Robot(std::string modulePath, std::string robotName, RobotType robotType);
	std::string setLight(int color[]);
protected:
	std::string getModuleName();
	std::string getClassName();
	PyObject* getConstructorArgs();
private:
	std::string name;
	RobotType type;
};

#endif //ACTION_HISTORY
