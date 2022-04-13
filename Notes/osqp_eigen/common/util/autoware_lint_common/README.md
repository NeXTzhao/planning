# autoware_lint_common

A custom version of [ament_lint_common](https://github.com/ament/ament_lint/tree/master/ament_lint_common) for [Autoware](https://www.autoware.org/).

## Usage

Add dependencies of `ament_lint_auto` and `autoware_lint_common` to your package as below.

`package.xml`:

```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>autoware_lint_common</test_depend>
```

`CMakeLists.txt`:

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

Then, the following linters will run during `colcon test`.

- [ament_cmake_copyright](https://github.com/ament/ament_lint/blob/master/ament_cmake_copyright/doc/index.rst)
- [ament_cmake_cppcheck](https://github.com/ament/ament_lint/blob/master/ament_cmake_cppcheck/doc/index.rst)
- [ament_cmake_lint_cmake](https://github.com/ament/ament_lint/blob/master/ament_cmake_lint_cmake/doc/index.rst)
- [ament_cmake_xmllint](https://github.com/ament/ament_lint/blob/master/ament_cmake_xmllint/doc/index.rst)

## Design

The original `ament_lint_common` contains other formatters/linters like `ament_cmake_uncrustify`, `ament_cmake_cpplint` and `ament_cmake_flake8`.
However, we don't include them because it's more useful to run them with `pre-commit` as [MoveIt](https://github.com/ros-planning/moveit2) does.

For example, the benefits are:

- We can use any version of tools independent of ament's version.
- We can easily integrate into IDE.
- We can easily check all the files in the repository without writing `test_depend` in each package.
- We can run formatters/linters without building, which makes error detection faster.

Ideally, we think other linters should be moved to `pre-commit` as well, so we'll try to support them in the future.
