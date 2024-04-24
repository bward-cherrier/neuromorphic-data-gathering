# iniVation C++ Coding Guidelines

\[\[_TOC_\]\]

## About

The "rules" presented in this documentation are, in fact, *guidelines*. This means they can be ignored in exceptional
cases.

If you feel these guidelines should be modified, feel free to post the suggested change in a MR and notify the team. The
[Google guidelines](https://google.github.io/styleguide/cppguide.html) are usually a good place to ground the
discussion.

## Code formatting

Always format your code using standard tools:

- C/C++/Java: clang-format
- Python: yapf
- CMake: cmake-format

We use pre-commit to ensure formatting is automatic.

Style consistency within a file/project is most important. Don't mix styles when modifying a file, either spend the
needed time and refactor everything in a separate MR or just follow what is already there.

## Class Structure

- Each class must be separated into its own file for readability.
  - C++: try to separate declaration (`.hpp`) from definiton (`.cpp`), unless it's a header-only library.
  - C++: keep your headers clean - you can write declaration and definition together in the header, or you can separate
    them if you prefer. The most important is to keep your code clean and consistent within the file and project
  - C++: only keep small utility function and classes confined to the `.cpp` source file.
- A class definition should usually start with a `public:` section, followed by `protected:`, then `private:`.
- Within each section, group similar kinds of declarations together in the following order:
  - Types and type aliases (typedef, using, enum, nested structs and classes, and friend types)
  - Static constants
  - Factory functions
  - Constructors and assignment operators
  - Destructor
  - Member functions (static followed by non-static followed by friend functions)
  - Data members (static followed by non-static)
- Keep the same order inside all related files (.hpp, .cpp, unit tests, ...).
- Functions should be located above other functions that they depend on.
  - "Your code should read like an article" - Uncle Bob in
    *[Clean Code](https://www.oreilly.com/library/view/clean-code/9780136083238/)*
- C++: All classes and structs follow either the Rule of Zero or the Rule of Five.
  - If any of {destructor, copy constructor/assignment or move constructor/assignment} needs to be implemented
    specifically, implement all of them.
    - This is the Rule of Five
    - Deleting ( = delete) is considered a specific implementation and requires the remaining methods to be implemented
      as well.
  - Most classes should not implement any of them (all defaulted and not user-provided).
    - This is the Rule of Zero

## General

- Use `auto` for local variable types wherever possible in C++ (`var` in Java).
  - For examples, see: https://google.github.io/styleguide/cppguide.html#Type_deduction
  - Avoid double type declarations - use auto `int32_t value = static_cast<uint32_t>(otherValue);`
  - Eigen: auto has specific connotations for lazy vs immediate evaluation.
- Use fixed-width integers: `(u)int[8|16|32|64]_t` (instead of e.g. `int` and `long`).
- Make things `const` wherever they can be `const` (parameters, variables, methods).
  - If copy and move assignment operators need to be available, member variables cannot be `const`, prefer the
    assignment operators over `const` members, as it allows more flexible usage of your class.
- Use proper namespaces, do not pollute the global namespace.
- Use enum classes, not old C-style naked enums.
- Dead code is never allowed.
  - E.g. unreachable code, unused functions and variables.
- Avoid the use of `goto`.
  - There are only narrow use-cases, especially in older C code, where it turns out to be useful; exceptions and RAII in
    C++ provide much better lifetime and cleanup management.
- Try to fix as many warnings as possible.
  - Especially unused arguments often indicate an issue (either not used by mistake, or bad API).
  - Integer comparisons of different sizes or sign can often be easily fixed by casting or using the appropriate
    unsigned types (e.g. `size_t`).
    - This indicates you've at least thought about the issue and what values your integers can take.
- Don't use arithmetic operations to perform boolean operations.
  - E.g. setting flags via addition `+` is not permitted, use `|` instead.
    - Addition only works if all flags are exact powers of two, which is not guaranteed (flags may imply other flags).
- Compare `std::string`s via the comparison operator.
  - Don't use .compare() but `if (str == "hello") {}`.
- When passing raw pointers as arguments, always check them against nullptr.
- Check your grammar.
  - All documentation and comments should be in American English.
  - Most IDEs at least spell check comments and strings in English automatically or have plugins (QtCreator:
    https://github.com/CJCombrink/SpellChecker-Plugin/).
- Prefer modern C++ casts to C-style casts.
  - https://google.github.io/styleguide/cppguide.html#Casting
- Avoid using run-time type information (RTTI).
  - https://google.github.io/styleguide/cppguide.html#Run-Time_Type_Information\_\_RTTI\_
- Avoid defining implicit conversions, unless there's a very compelling reason to do so.
- Use the explicit keyword for conversion operators and single-argument constructors.
  - https://google.github.io/styleguide/cppguide.html#Implicit_Conversions
- To return multiple pieces of data, prefer the use of a struct for data that can be meaningfully named and described.
  Otherwise, the use of `std::pair` or `std::tuple` are also admissible.
  - https://google.github.io/styleguide/cppguide.html#Structs_vs.\_Tuples
  - In case of using output arguments, put them at the end of the argument list and add an 'Out' prefix to the name.

## Readability and Complexity

- Declare variables as locally in scope as possible.
- All control blocks have braces around them.
  - `if () { ... }`, `while () { ... }`
  - One line statements without braces are forbidden.
- Prefer early returns (for readability).
  - `if (!x) { return; }` instead of `if (x) { ... lots of code ... }`.
- Avoid heavily nested code (~5 levels of nested control blocks is generally too much).
- For long conditional statements, prefer breaking out to separate boolean variables.
  ```
  const bool conditionA = ((a - b) > c);
  const bool conditionB = (!d) && (e == f.x);
  const bool conditionC = ((func(y) == 42) || (!fonc(z)));
  if (conditionA && conditionB && conditionC) {}
  ```
- Put parentheses `()` inside complex conditional statement expressions.
  - E.g. `if (((a - b) > c) && (!d) && (e == f.x) && ((func(y) == 42) || (!fonc(z)))) {}`
    - (This example is for illustration only, it should obviously be broken up into separate variables).
- Newlines are your friend. Sprinkle them around to break up code blocks.
  - Really, they have no drawbacks!
- Use structs/classes to keep related data together.
- Keep functions concise, and try to follow the Single Responsibility Principle.
  - Nobody wants to read your 1000-liner wall of code, not even you yourself.
- Keep your files short.
  - Try to not exceed 1000 lines of code, split it up!

## Functions

- Pass objects of size > 16 bytes, as well as non-POD objects, by reference.
  - Integers, bool, enum and enum classes are small and POD, so pass by value.
- Wherever possible, use references instead of pointers.
- If pointers are required to manage resources, use smart pointers.
  - `std::unique_ptr`, `std::shared_ptr`, (`std::weak_ptr`).
- If pointers are required with no ownership semantics intended, use raw pointers.
  - https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rf-smart
- Whenever the passed object is not modified, pass it as `const`.
- For read-only string-like objects, prefer passing as `std::string_view` by value.
- For read-only array-like objects, prefer passing as `std::span` by value.
- Use `[[nodiscard]]` wherever it makes sense.
  - Functions that do not modify their arguments nor have any other side-effects.
    - E.g. only returning a value, or computing a value and then returning it but not storing it.
  - Functions where it is a logical error to discard the return value.

## Header Includes

- Include what you use, and nothing else. Do not rely on transitive includes.
- Use `#pragma once` instead of header guards.
- Only add forward declarations if truly needed (circular/recursive definitions), try to avoid them in most cases.

## Error Handling

- To report exceptional failure conditions, functions must not return `nullptr` or error codes, but throw exceptions
  with as much useful information as possible.
- Error handling code should be separated from code for normal processing (no nested `if`/`else` statements checking for
  and dealing with errors).

## Comments

- Function and class definitions should use Doxygen-compatible style (`/** ... */` or `///`).
- Code comments should use the `//` style (no Doxygen).
  - Long multi-line comments (4+ lines) can use the `/* ... */` style
- For TODOs, prefer linking to a gitlab issue describing the enhancement in detail. Otherwise, use the following syntax:
  `TODO(username): comment`.
- Prefer abstractions to comments.
  - If you find yourself writing gigantic comments to explain your code, this code should most likely be wrapped into a
    function with a useful name and appropriate documentation.
- Software architecture diagrams are encouraged in project readmes. Please use
  [mermaid](https://mermaid-js.github.io/mermaid/#/).

## Public Code Documentation

Public interfaces should be documented using Doxygen-compatible comments syntax:

- https://github.com/stan-dev/stan/wiki/Coding-Style-and-Idioms#how-to-write-doxygen-documentation-comments
- https://www.doxygen.nl/manual/index.html

Use the `@` syntax to introduce Doxygen keywords, not the `\` one. You can use `@brief` to split long documentation, use
your good judgment.

## Naming Conventions

- Abbreviations should generally be avoided, with two exceptions:
  - Well-established abbreviations are permissible if they shorten otherwise too long names (e.g.
    `DynamicVisionCameraController` -> `DvCameraController`).
  - Well-established abbreviations are permissible if the scope is local (e.g. `i` or `idx` as a loop variable).

### Functions

- Functions shall be written in camelCase.
  - Regex: `^[a-z][a-zA-Z0-9]*$`
- Function names should be pronounceable, meaningful, and generally longer than four characters but shorter than six
  words.
- If your function name turns out to be very long and contain And, Or or similar nouns, it's a good indicator that you
  might want to split it apart.

### Variables

- Class and struct member names shall start with an 'm' followed by a PascalCase name.
  - This makes autocompletion inside classes far easier.
  - Regex: `^m[A-Z][a-zA-Z0-9]*$`
- Local variables and function parameters shall be written in camelCase.
  - Regex: `^[a-z][a-zA-Z0-9]*$`
- Variable names should be pronounceable, meaningful, and generally longer than two characters (except for loop
  variables, but even there try to use full names, especially in nested cases).
- Make sure all class member variables are actually used somewhere. The compiler can't always analyze this.

### Classes, Structs, Unions and Enum Types

- All types shall follow PascalCase naming.
  - Regex: `^[A-Z][a-zA-Z0-9]*$`

### Macros, Constants, Enum Values and Template Type Names

- Avoid macros wherever possible
  - Really, don't do it.
- Macros and constants (constexpr, static) at global or class level shall be in SCREAMING_SNAKE_CASE.
  - Regex: `^[A-Z][A-Z0-9_]*$`

### Namespaces

- Namespaces shall be in snake_case.
  - Regex: `^[a-z][a-z0-9_]*$`
- Try to name namespaces so that they reflect the logical hierarchy and grouping of the code.

### Template Parameters

- Template parameters shall be in PascalCase.
  - Regex: `^[A-Z][a-zA-Z0-9]*$`

### Files

- `C++` files: `.cpp` and `.hpp`.
- `C` files: `.c` and `.h`.
- Python files: `.py`.
- Files should have useful names, reflecting their class name if applicable. Always add a proper file extension.
- All file names should consist entirely of the a-z A-Z 0-9 - and _ characters to be truly portable.
- Dots are generally also allowed, though usage of only one to separate the extension is preferred.
- Source code files should follow the snake_case format (NOTE: no dashes allowed for source code files).
  - Regex: `^[a-z0-9_]+\.(c|cpp|h|hpp|py)$`

A simple C++ regex to sanitize any file name:

```
#include <regex>
const std::regex filenameCleanupRegex{"[^a-zA-Z-_\\d]"};
auto cleanFilename = std::regex_replace(originalFilename, filenameCleanupRegex, "_");
```

### Branches

- names of branches should start with a prefix explaining the type of change
  - feat/xxx - new feature
  - bug/xxx - bug fix or replacement
  - junk/xxx - throwaway branch
  - wip/xxx - work in progress - stuff that won't be finished anytime soon

## File Handling

- C++ widely uses the RAII (Resource Acquisition Is Initialization) paradigm, so generally objects clean up after
  themselves when they go out of scope and their destructor is called.
  - Calling `file.close()` or similar is often not needed, especially for temporary objects.
- When loading data from file, or network, always check, check, check!
  - Never assume data is present or makes sense.
  - OpenCV for example silently returns empty/zero objects when using cv::FileStorage.
    - To actually know if there were values in the loaded file, you have to compare the returned object's `.type()` to
      `cv::FileNode::NONE`.

## C++ standard

We currently use the C++20 standard and support the following compilers and C++ standard libraries:

- GCC 10 and newer
- Clang 13 and newer
- GCC libstdc++ 10 and newer
- XCode 14 (Apple clang / libc++) and newer

We do not currently support Clang libc++ as its C++20 support is incomplete.

As such, the following C++20 features cannot be currently used:

- limited consteval support (https://en.cppreference.com/w/cpp/language/consteval)
- parenthesized initialization of aggregates (https://en.cppreference.com/w/cpp/language/aggregate_initialization)
- modules support (https://en.cppreference.com/w/cpp/language/modules)
- using enum (https://en.cppreference.com/w/cpp/language/enum#Using-enum-declaration)
- atomic for floats (https://en.cppreference.com/w/cpp/atomic/atomic#Specializations_for_floating-point_types)
- atomic std::shared_ptr (https://en.cppreference.com/w/cpp/memory/shared_ptr/atomic2)
- std::atomic_ref (https://en.cppreference.com/w/cpp/atomic/atomic_ref)
- synchronized buffered streams (https://en.cppreference.com/w/cpp/io/basic_osyncstream)
- calendar and timezone **(use https://github.com/HowardHinnant/date/ instead)**
- std::bit_cast (https://en.cppreference.com/w/cpp/numeric/bit_cast) **(use memcpy() workaround)**
- ranges and their algorithms (https://en.cppreference.com/w/cpp/ranges) **(use old algorithms with begin() and end())**
- heterogeneous lookup for unordered containers **(use https://github.com/greg7mdp/parallel-hashmap/ instead)**
- std::assume_aligned() (https://en.cppreference.com/w/cpp/memory/assume_aligned)
- std::execution::unseq (https://en.cppreference.com/w/cpp/algorithm/execution_policy_tag)
- std::basic_stringbuf efficient access to underlying buffer via str()/view()
- std::format **(use fmt::format instead)**
- constexpr std::string **(can be emulated with fixed-size std::array)**
- constexpr std::vector **(can be emulated with fixed-size std::array)**
- std::stop_token and std::jthread **(continue using std::atomic_bool to stop threads)**
- atomic waiting and notifying, std::counting_semaphore, std::latch and std::barrier
- std::source_location **(use \<dv-processing/external/source_location_compat.hpp> as alternative)**
- safe integral comparisons (https://en.cppreference.com/w/cpp/utility/intcmp)

## Library Recommendations

- file operations: [std::filesystem](https://en.cppreference.com/w/cpp/filesystem) (IMPORTANT: use this instead of
  boost::filesystem)
- better error handling: dv::exceptions
- printf-style printing in C++: [fmt::format](https://fmt.dev/latest/syntax.html)
- networking: [boost::asio](https://www.boost.org/doc/libs/master/doc/html/boost_asio.html)
- string splitting on tokens (eg. CSV):
  [boost::tokenizer](https://www.boost.org/doc/libs/master/libs/tokenizer/doc/tokenizer.htm)
- simple string split/join: boost::algorithm::string (possible alternative for joining: fmt::format)
- date, calendar and timezones: https://github.com/HowardHinnant/date/
- hash maps/sets: https://github.com/greg7mdp/parallel-hashmap/

## DV-SDK specific

- `dv::Frame::FrameFormat` is OpenCV-compatible, the values behind the enum correspond to OpenCV frame format values.
- `dv::cvector` (from \<dv-processing/data/cvector.hpp>) provides a nice std::vector alternative, that is low-level.
  C-compatible across multiple compilers and standard libraries, can convert to/from std::vector, and adds some useful
  features like append() and addition of vectors.
- Finding the home directory of the current user is not simple or portable, use the function
  `dv::portable_get_user_home_directory()` from \<dv-sdk/cross/portable_io.h>, it will try to find the real home, or in
  absence of one, the system temporary directory, and check that it exists and is actually a directory. On failure it
  will throw an exception. Don't use `std::filesystem::current_path()`.
- Never use ​`double​` to represent a timestamp, the preferred form is `dv::TimePoint` (or `dv::Duration` for time
  spans), an admissible alternative is `int64_t​`.
  - `double` only guarantees 15 significant decimal digit precision, while microsecond representation of a current time
    requires 16, which can lead to rounding errors in a few microseconds.

## Testing

### Unit testing

- All mathematically challenging code should be unit tested.
- Use [boost-ut](https://github.com/boost-ext/ut) framework for unit testing.
- Keep dependencies of your unit tests as minimal as possible.
  - Only the necessary headers should be included.
  - Keep cross-dependencies minimal.
  - Only use external files when it is well justified, e.g:
    - OK: large dataset: aedat4 file with 1000 events.
    - NOK: event store with 2 events - should be constructed in the test source file.
- Directory structure of tests should reflect the directory structure of source code.
- The order of tests in a test file should correspond to the order of functions in source code.
- Bug reports are encouraged to include a unit test or at least a code snippet reproducing the error

### Integration testing

Look at dv-runtime tests for reference.

### Manual testing

Sometimes manual testing is the only reasonable way to go. Do write extensive documentation how the tests should be
done, what are the input files, how to judge if the test passed or not. Do your own thorough research and expand this
section!

## Code review

- All of the code committed in iniVation repositories should be peer-reviewed.
  - The only permitted exceptions are oneoff projects and trivial changes (fixing typos).
- The developer who creates MR should assign the most appropriate person as the reviewer.
  - Ideally code owner, otherwise another developer who is familiar with the given repository.
  - In dv-processing, assign another developer for initial review and then the reviewer assigns @llongi for the final
    review.
  - In case you don't know who is the right person - ask on Teams.
- Avoid MRs with more than 500 lines of relevant changes. If not possible, explain how to efficiently review it in MR
  description.
- As a reviewer, do your best to reply within 24h to the review request - if you can't finish it within 24h, suggest a
  deadline.
- For low priority MRs - mention it to the reviewer so they don't feel pressured to review quickly.
- Always be polite and helpful - thanks to code reviews we learn. Try to make it a good experience for the developer.
