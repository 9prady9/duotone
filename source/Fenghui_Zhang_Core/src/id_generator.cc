// Author: Fenghui Zhang
//#include "Fenghui_Zhang_core/common/util/id_generator.h"
#include "id_generator.h"

IdGenerator::IdGenerator() {};

/*static*/ int IdGenerator::getNextId(ClassType class_type) {
  static IdGenerator id_generator;
  return id_generator.id_repository_[static_cast<int>(class_type)]++;
}
