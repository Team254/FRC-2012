/* @author Eric Caldwell
 *
 * This file is used to initialize and access the values of all of the constants.
 * Implements the singleton style to insure that only one instance of this class exists.
 * Uses ifstream to read values from a file into a map which is then used to replace
 * the classes member variables if ones exist that correspond to the map.
 */
#include "config/Constants.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

Constants* Constants::instance_ = NULL;
const char* kConstantsFile = "constants.txt";

/** Checks to see if an instance of Constants has been created, if not, creates it. */
Constants* Constants::GetInstance() {
  if (instance_ == NULL) {
    instance_ = new Constants();
  }
  return instance_;
}

/** 
 * Uses a macro to set default values for member variables based on the contents of ConstantDeclarations.h.
 * Then it reads from the file specified in kConstantsFile and if values are found corresponding to variables
 * in the class, it replaces the default value with the value found in the file.
 */
Constants::Constants() {
#define DECLARE_DOUBLE(name, defaultValue) \
  name = defaultValue;
#include "config/ConstantDeclarations.h"
#undef DECLARE_DOUBLE
  Constants::LoadFile();
}

/** 
 * Opens the file specified by kConstantsFile using ifstream and stores its contents into a map. Then it
 * goes through the map's keys and if they correspond with Constant's member variables it replaces the
 * existing variable value with the value from the map.
 */
void Constants::LoadFile() {
  std::ifstream constantsFile(kConstantsFile);
  std::map<std::string, std::string> constantsMap;
  if (constantsFile.is_open()) {
		while (!constantsFile.eof()) {
			std::string key;
			getline(constantsFile, key, '=');

      std::string value;
			getline(constantsFile, value);
			constantsMap[key] = value;
		}
		constantsFile.close();
	}
#define DECLARE_DOUBLE(name, defaultValue) \
  if(constantsMap.find(#name) != constantsMap.end()) { \
    name = atof(constantsMap[#name].c_str()); \
  }
#include "config/ConstantDeclarations.h"
#undef DECLARE_DOUBLE
}
