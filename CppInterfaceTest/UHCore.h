#include <string>

class PythonInterface {
public:
	PythonInterface(std::string modulePath);
	virtual ~PythonInterface();
};

class Robot: public PythonInterface {
public:
	enum RobotType {
		CareOBot, Sunflower
	};

	Robot(std::string modulePath, std::string robotName, RobotType robotType);
	std::string setLight(int color[]);
};

class ActionHistory: public PythonInterface {
public:
	ActionHistory(std::string modulePath);
	bool cancelPollingHistory(std::string ruleName);
	char* addPollingHistory(std::string ruleName, float delaySeconds);
	void addHistoryAsync(std::string ruleName);
	bool addHistory(std::string ruleName);
};

