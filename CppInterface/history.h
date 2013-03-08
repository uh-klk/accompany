#ifndef ACTION_HISTORY
#define ACTION_HISTORY
#include "base.h"
#include <string>

class ActionHistory: public PythonInterface {
public:
	ActionHistory(std::string modulePath);
	bool cancelPollingHistory(std::string ruleName);
	char* addPollingHistory(std::string ruleName, float delaySeconds);
	void addHistoryAsync(std::string ruleName);
	bool addHistory(std::string ruleName);
protected:
	std::string getModuleName();
	std::string getClassName();
};

#endif //ACTION_HISTORY
