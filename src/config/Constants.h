#ifndef CONFIG_CONSTANTS_H_
#define CONFIG_CONSTANTS_H_

#include "utils.hpp" // Constants are included everywhere, why not utils?

/**
 * @author Eric Caldwell
 *
 * Constants is a singleton class that reads constants from a file and stores them in a set of public member
 * variables to be used later in the code.
 *
 * Names and default values for these variables are read from ConstantDeclarations.h. This file is used so
 * that values can be quickly and easily changed and tested without having to recompile code each time.
 *
 * Sample Usage:
 * Constants* constants = Constants::GetInstance();
 * printf("Test Int %d\n", constants->testinteger);
 *
 * This would create or retrieve an instance of this class with all the constants initialized.
 */
class Constants {
 public:
  // List the definitions from ConstantDeclarations.h as public members of this class.
#define DECLARE_DOUBLE(name, defaultValue) \
  double name;
#include "config/ConstantDeclarations.h"
#undef DECLARE_DOUBLE

  /**
   * Returns a singleton instance of this class to ensure that users don't create multiple classes with
   * references to variables that should have a single constant value.
   *
   * To be used in place of constructor for this class.
   */
  static Constants* GetInstance();

  /**
   *  Opens a specified filepath and replaces the default values with ones specified in the open file.
   */
  void LoadFile();
 private:
  /**
   * Enters default values for every everything entered in ConstantDeclarations.h and then replaces them
   * using LoadFile(). Private to prevent direct instantiation; use GetInstance() instead.
   */
  Constants();

  // A static reference to this class to ensure that only one instance of it is created.
  static Constants* instance_;
};

#endif  // CONFIG_CONSTANTS_H_
