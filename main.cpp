#include <pico/stdlib.h>

int main() {
	stdio_init_all();

	while (true) {
		tight_loop_contents();
	}

	return 0;
}
