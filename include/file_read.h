#ifndef FILE_READ_H
#define FILE_READ_H
#include <string>
#include <vector>
#include "constants.h"

//downloading TOML file from URL and saves it locally
bool downloadTomlFile(const std::string& url, const std::string& localPath);
Header readHeader(const std::string& filename);
Scan readScan(const std::string& filename);
std::vector<double> readRanges(const std::string& filename);
std::vector<double> readIntensities(const std::string& filename);

#endif

