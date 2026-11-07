// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cstdio>
#include <vector>

namespace regolith::test {

struct Case {
	const char *name;
	void (*fn)();
};

std::vector<Case> &registry();
extern int failures;

struct Registrar {
	Registrar(const char *p_name, void (*p_fn)()) { registry().push_back({ p_name, p_fn }); }
};

} // namespace regolith::test

#define TEST_CASE(m_name)                                                  \
	static void m_name();                                                  \
	static regolith::test::Registrar m_name##_registrar(#m_name, m_name); \
	static void m_name()

#define CHECK(m_cond)                                                                    \
	do {                                                                                 \
		if (!(m_cond)) {                                                                 \
			++regolith::test::failures;                                                  \
			std::printf("  check failed: %s (%s:%d)\n", #m_cond, __FILE__, __LINE__); \
		}                                                                                \
	} while (0)
