#include "config/Constants.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

Constants* Constants::instance_ = NULL;
const char* kConstantsFile = "/constants.txt";

Constants* Constants::GetInstance() {
  if (instance_ == NULL) {
    instance_ = new Constants();
  }
  return instance_;
}

Constants::Constants() {
  // Initialize the constants defined in ConstantDeclarations.h with their default values.
#define DECLARE_DOUBLE(name, defaultValue) \
  name = defaultValue;
#include "config/ConstantDeclarations.h"
#undef DECLARE_DOUBLE
  Constants::LoadFile();
}

void Constants::LoadFile() {
  // Open the constants file specified by kConstantsFile.
  std::ifstream constantsFile(kConstantsFile);
  std::map<std::string, std::string> constantsMap;
  if (constantsFile.is_open()) {
    printf("Found constants file to load.\n");
    // Store the contents of the constants file in a string-string map.
    while (!constantsFile.eof()) {
      std::string key;
      getline(constantsFile, key, '=');

      std::string value;
      getline(constantsFile, value);
      constantsMap[key] = value;
      printf("Loaded constant %s with value %s.\n", key.c_str(), value.c_str()); 
    }
  constantsFile.close();
}
  // For each constant defined in ConstantDeclarations.h, use the value from the map if it exists.
#define DECLARE_DOUBLE(name, defaultValue) \
  if (constantsMap.find(#name) != constantsMap.end()) { \
    name = atof(constantsMap[#name].c_str()); \
  }
#include "config/ConstantDeclarations.h"
#undef DECLARE_DOUBLE
}
