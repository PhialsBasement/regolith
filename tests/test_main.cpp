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
