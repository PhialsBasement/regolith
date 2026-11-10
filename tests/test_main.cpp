// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cstring>

#include "test.hpp"

namespace regolith::test {

std::vector<Case> &registry() {
	static std::vector<Case> cases;
	return cases;
}

int failures = 0;

} // namespace regolith::test

// regolith_core_tests [TEXT]: every case, or only those whose name contains TEXT.
int main(int argc, char **argv) {
	using namespace regolith::test;
	const char *only = argc > 1 ? argv[1] : "";
	size_t ran = 0;
	for (const Case &c : registry()) {
		if (std::strstr(c.name, only) == nullptr) {
			continue;
		}
		const int before = failures;
		c.fn();
		std::printf("%s %s\n", failures == before ? "PASS" : "FAIL", c.name);
		std::fflush(stdout);
		++ran;
	}
	std::printf("%zu cases, %d failed checks\n", ran, failures);
	return failures == 0 ? 0 : 1;
}
