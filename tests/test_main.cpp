// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "test.hpp"

namespace regolith::test {

std::vector<Case> &registry() {
	static std::vector<Case> cases;
	return cases;
}

int failures = 0;

} // namespace regolith::test

int main() {
	using namespace regolith::test;
	for (const Case &c : registry()) {
		const int before = failures;
		c.fn();
		std::printf("%s %s\n", failures == before ? "PASS" : "FAIL", c.name);
	}
	std::printf("%zu cases, %d failed checks\n", registry().size(), failures);
	return failures == 0 ? 0 : 1;
}
