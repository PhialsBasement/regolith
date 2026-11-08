# AGENTS.md

Rules for anyone, human or agent, writing code in this repository.

These rules apply to every change, whatever the existing code does.
Code already in the repo is not a precedent. Where it breaks these
rules, don't copy it. Write new code to this standard.

## The one rule

**Write the simplest code that fully achieves the real goal.**

Keep complexity down at every step, but never at the cost of the goal.
A shortcut that only partly solves the problem is not simpler; it's
unfinished. Get the goal right first, then find the simplest way to
reach it.

## Simplicity

- Choose the most direct solution that is correct. If two approaches
  work, take the one with fewer moving parts.
- Build only what the task needs now. No speculative options, hooks,
  parameters or abstractions "for later".
- No cleverness. A plain loop beats a dense one-liner. Obvious code
  beats fast-looking code unless a measurement shows the speed matters.
- Prefer deleting code to adding it. Remove dead code, unused
  parameters, stale comments and unreachable branches when you meet
  them in code you are changing.
- One way to do each thing. Before writing a helper, check whether one
  already exists (`find_body`, `rdu::bytes_of`, `regolith::` math) and
  use it. Never paste a second copy of logic. Extract it instead.

## Nesting

Keep nesting as shallow as possible.

- Use guard clauses and early returns. Handle the invalid, empty and
  error cases first and return, so the main path is not indented.
- Aim for at most two levels of nesting inside a function body. A third
  level needs a reason. Anything deeper must be split.
- No `else` after a `return`, `continue` or `break`.
- Use `continue` to skip loop items, not a wrapping `if`.
- When a loop body or a branch grows past a few lines, move it into a
  well-named function.

```cpp
// Don't
void update(Body *p_body) {
	if (p_body) {
		if (p_body->is_open()) {
			for (Tile &t : p_body->tiles()) {
				if (t.ready) {
					draw(t);
				}
			}
		}
	}
}

// Do
void update(Body *p_body) {
	if (!p_body || !p_body->is_open()) {
		return;
	}
	for (Tile &t : p_body->tiles()) {
		if (!t.ready) {
			continue;
		}
		draw(t);
	}
}
```

## Readability

- Code is read far more than it is written. Optimise for the next
  reader understanding it on the first pass.
- One function does one job, and its name says what that job is. Aim
  for functions that fit on one screen, about 40 lines. A longer
  function needs a reason, and multi-hundred-line functions are not
  acceptable in new code.
- One class has one responsibility. Don't add new duties to a class
  that is already doing several. Give them a class of their own.
- Name things for what they are, in full words. Use short names only
  for tiny scopes (`i`, `k`) or established terms (`rho`, `gm`, `soi`).
- Name every magic number as a constant, with its unit, and define it
  once. `AU`, `PI` and friends live in one place, not one per file.
- Comments explain *why*: intent, units, frames, invariants and
  non-obvious constraints. Don't narrate what the code already says.
  Keep comments true. A wrong comment is worse than none.
- Keep the existing naming and formatting conventions: `p_` for
  parameters, `r_` for out-parameters, a trailing `_` on members, and
  tabs for indentation.

## Correctness at the edges

Simple code must also be safe code.

- Validate everything that comes from outside before using it: files
  (`.rgp`, `.rge`, rasters, maps), command-line arguments, and values
  passed in from scripts. Check sizes, ranges and finiteness before
  allocating or indexing with them.
- Never let bad input crash the game. Report it once with
  `ERR_PRINT_ONCE` / `WARN_PRINT_ONCE` and degrade cleanly.
- No exceptions across the GDExtension boundary. Don't use throwing
  calls (`.at()`, unchecked `new` of file-sized buffers) on paths a
  script or a file can reach.
- Don't hold raw pointers to Godot objects you don't own past the
  current call. Store an instance id and resolve it with `ObjectDB`.
- Handle node lifecycle fully: anything built in `_ready` must survive
  leaving and re-entering the tree, and must not run in the editor
  unless meant to (`is_editor_hint()`).

## Scope of a change

- Change what the task needs, and leave the code you touched simpler
  than you found it. Don't rewrite unrelated code in the same change.
- New behaviour comes with a test in `tests/` when it lives in `core/`.
- Update the docs (`docs/`, `README.md`) when behaviour, formats or
  limits change.

## How to work

### 1. Prove it, don't assert it

- Every claim that something works or is fixed needs evidence: a
  test, a reproduction, or a before/after run. Confidence is not
  evidence.
- A change meant to keep behaviour the same is checked by running the
  old and new builds on the same scenario and comparing the results.
- Report what was verified, what was only reasoned about, and what was
  not checked at all. Never present an assumption as a result.

### 1b. Tests and checks MUST be readable

- A test or check must read like a statement of what is expected.
  Someone who has never seen it should understand it on the first pass.
- Define every variable clearly: a full name, its unit, and where its
  value comes from. No unexplained literals: name an expected value and
  say why it is the right one.
- One behaviour per test, named for that behaviour.
- State the end result plainly: what was run, what was expected, what
  happened, and what that means. When reporting, give the numbers that
  matter, not a wall of logs.

### 2. Small steps you can follow

- Make the change easy, then make the easy change. Refactors and
  behaviour changes go in separate commits.
- One logical change per commit, with a message that says why.
- For anything structural (splitting a class, changing a file format,
  a new public API), propose the plan and wait for agreement before
  writing code.

### 3. The owner stays in charge

- Never commit, push, or change anything outside the task without the
  owner's go-ahead.

### 4. Explain decisions

- For every non-trivial change, say why this approach and which simpler
  options were rejected.
- Explain any unusual command, flag or workaround you use, so the owner
  keeps their understanding of the system.

### 5. Stop when stuck

- If a fix needs a fix, or you are guessing, stop. Report what you know
  and what you don't. Never stack patch on patch.

### 6. Keep the design decomposed

- `core/` has no Godot dependency. Keep it that way: it is what keeps
  the core testable.
- New features get their own class or file. Don't grow `RegolithBody`
  or `RegolithSystem` further.
- No new coupling between the node classes beyond what the
  `RegolithSystem` already mediates.

### 7. Performance and resources count

- When touching hot paths (per-frame `_process`, the tile loader, the
  compositor passes), check frame cost, memory, threads and GPU
  resources before and after.
- Use the existing stats (`get_stats`, the `*_ms` timings) to show
  there is no regression. Keep the README's promises true, such as the
  atmosphere pass costing about 0.25 ms.

## Before you call it done

- [ ] It fully achieves the actual goal, not a reduced version of it.
- [ ] There is no simpler way that would also be correct.
- [ ] No function nests deeper than it has to, and the main path is not indented.
- [ ] No duplicated logic, magic numbers, dead code or unused parameters.
- [ ] External input is validated and can't crash the game.
- [ ] It builds without new warnings, and the tests pass.
- [ ] Every claim is backed by evidence, and the report says what was and wasn't verified.
- [ ] Tests and checks are readable, with named values and a plainly stated result.
- [ ] Hot paths were measured before and after, if touched.
- [ ] Nothing was committed or pushed without the owner's go-ahead.
