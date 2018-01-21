// Author: Fenghui Zhang
#ifndef COMMON_UTIL_ID_GENERATOR
#define COMMON_UTIL_ID_GENERATOR

#include <map>

enum ClassType {VERTEX, EDGE, FACE, CORNER};

/*
This singleton class generates unique IDs for registered classes. It's not
thread-safe. So don't use it in multithreading environment.

The IDs returned from this generator all start from 0.
*/
class IdGenerator {
 public:
  static int getNextId(ClassType class_type);

 private:
  IdGenerator();

  std::map<int, int> id_repository_;
};

#endif
