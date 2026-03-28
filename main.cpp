#include "delay_app.h"

int main() {
	firmware::DelayApp app;
	if (!app.init()) {
		return 1;
	}
	app.run();
	return 0;
}
