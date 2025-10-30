#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <curl/curl.h>

#include "file_read.h"

//callback function to write received data
size_t writeCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

//download toml files
bool downloadTomlFile(const std::string& url, const std::string& localPath) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "CURL initialization failed" << std::endl;
        return false;
    }

    std::string response;
    
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

    //bypassing the certificate warnings
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
    
    CURLcode res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK) {
        std::cerr << "Download failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    std::ofstream outFile(localPath);
    if (!outFile) {
        std::cerr << "Could not create local file" << std::endl;
        return false;
    }

    outFile << response;
    outFile.close();
    return true;
}

//trims the string
std::string trim(const std::string& s) {
    int start = 0; //strings first index
    while (start < s.size() && (s[start] == ' ' || s[start] == '\t' || s[start] == '"')) start ++; //if start index is smaller than the length of the array, we would know that this array indeed contains something. If there is unwanted charachters on the front of the array, this will trim
    int end = s.size() - 1; // arrays start with index 0, so we decremented 1 from the size to find the last value's index
    while (end >= start && (s[end] == ' ' || s[end] == '\t' || s[end] == '"')) end--; //if end index is smaller than the start index, this would create a problem. It also checks the unwanted charachters at the end of the array
    
    return s.substr(start, end - start + 1); //starts trimming from s[start], till end-start+1 which is also the length of the new string
}

//reads the first three lines of codes, that is explicitly start with "[header]"
Header readHeader(const std::string& filename) {
    std::ifstream file(filename);
    Header header = {"none", "none"};

    //checks if file is there; if its not there, return default header struct
    if (!file.is_open()){
        std::cerr << "File couldn't be found" << std::endl;
        return header; 
    }

    std::string line;
    //finds header, and gets the stamp and frame id
    while (std::getline(file, line)) {
        if (line == HEADER) {

            //using temp to pass unnecessary parts of the line
            std::string temp;
            file >> temp >> temp >> header.stamp;
            file >> temp >> temp >> header.frame_id;
            
            //removing the quote marks
            header.stamp = trim(header.stamp);
            header.frame_id = trim(header.frame_id);
            break;
        }
    }
    file.close();
    return header;
}

//using getline
Scan readScan(const std::string& filename) {
    std::ifstream file(filename);
    Scan scan = {0, 0, 0, 0, 0, 0, 0}; //default scan struct

    //checks if the file is there; if its not there, function returns default scan struct
    if (!file.is_open()) {
        std::cerr << "File couldn't be found";
        file.close();
        return scan;
    }

    std::string line; //temporary line for searching through lines
    bool isScan = false; //if [scan] is seen in the file, this will be true and it will check whats under there

    while (std::getline(file, line)) {
        if (line == "[scan]") isScan = true;

        //checks every name and when they appear on the line, the value of the name will be assigned to what function will return
        if (isScan) {
            int pos = line.find("=");
            if (pos == std::string::npos) continue;

            std::string key = trim(line.substr(0, pos));
            std::string value = trim(line.substr(pos+1));
            if (value.empty()) continue;
            
            if (key == "ranges") {
                isScan = false;
                break;
            }
            double val = stod(value);
            if (key == "angle_min") scan.angle_min = val;
            else if (key == "angle_max") scan.angle_max = val;
            else if (key == "angle_increment") scan.angle_increment = val;
            else if (key == "time_increment") scan.time_increment = val;
            else if (key == "scan_time") scan.scan_time = val;
            else if (key == "range_min") scan.range_min = val;
            else if (key == "range_max") scan.range_max = val;
            else;
        }
    }
    file.close();
    return scan;
}

//reads ranges part
std::vector<double> readRanges(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "File couldn't be opened" << std::endl;
        file.close();
        return ERROR_VECTOR;
    }

    std::string line;
    std::vector<double> ranges;
    bool in_ranges = false;

    //searching for ranges line by line
    while (getline(file, line)) {
        if (line.find("ranges") != std::string::npos) { //finds if ranges are in the file
            in_ranges = true;
            int pos = line.find("["); // finds the position of the '['; if there is none, it will return npos
            if (pos != std::string::npos) line = line.substr(pos+1); //if in the same line as ranges, there is a '[', it will pass them
        }

        if (in_ranges) {
            int end_pos = line.find("]"); //if ']' is found on the particular line, that place will be the end of our search
            if (end_pos != std::string::npos) { //checks if ']' is found, if it is still not found, end_pos will be npos
                line = line.substr(0, end_pos); //starts from the beginning, till the bracelet starts.
                in_ranges = false;
            }

            std::string numStr = ""; //getting rid of garbage value
            for (int i=0; i<line.length(); i++) { //checking each char, line by line
                char c = line[i];
                if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+') { //checks char by char, thats why so long as each of the charachters from the file is present, it will be put. Even if these are negative numbers
                    numStr += c;
                } else if (c == ' ' || c == ',' || c == '\t') { //if we see any of these chars, we will push numStr into ranges vector array
                    if (numStr != "") {
                        ranges.push_back(std::stod(numStr));
                        numStr = "";
                    }
                }
            }
            if (numStr != "") ranges.push_back(std::stod(numStr)); // end of the ranges datas, the last number does not followed up by comma, that is for that
        }
    }
    file.close();
    return ranges;
}

//reads intensities part
std::vector<double> readIntensities(const std::string& filename) {
    std::ifstream file(filename);

    //looks if the file can be opened, if it can not, it returns our error vector so we know
    if (!file.is_open()) {
        std::cerr << "File couldn't be opened" << std::endl;
        file.close();
        return ERROR_VECTOR;
    }
    
    std::string line;
    bool in_intensities = false;
    std::vector<double> intensities;

    while (std::getline(file, line)) {
        if (line.find("intensities") != std::string::npos) { //search for intensities part
            in_intensities = true;
            int pos = line.find("["); //search for this bracelet, if we find this we will start searching for numbers right after this bracelet
            if (pos != std::string::npos) line = line.substr(pos + 1);
        }

        if (in_intensities) {
            int end_pos = line.find("]"); //search for the end of the bracelet every line, if we find that, our search will be over
            if (end_pos != std::string::npos) {
                line = line.substr(0, end_pos);
                in_intensities = false;
            }

            std::string numStr = ""; //save numbers here as string, later to transform into double values
            for (int i = 0; i < line.length(); i++) {
                char c = line[i];
                if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+') {
                    numStr += c;
                } else if (c == ' ' || c == ',' || c == '\t') {
                    if (numStr != "") {
                        intensities.push_back(std::stod(numStr)); //add the number to our return vector
                        numStr = ""; //reset the number
                    }
                }
            }
            if (numStr != "") intensities.push_back(std::stod(numStr)); //check for the last value
        }
    }
    file.close();
    return intensities;
}

/* Not using getline
Scan readScan(const std::string& filename) {
    std::ifstream file(filename);
    Scan scan = {0, 0, 0, 0, 0, 0, 0}; //default scan struct

    //checks if the file is there; if its not there, function returns default scan struct
    if (!file.is_open()) {
        std::cerr << "File couldn't be found";
        return scan;
    }

    std::string line; //temporary line for searching through lines
    bool isScan = false; //if [scan] is seen in the file, this will be true and it will check whats under there
    std::string key, value, equal_sign;

    while (std::getline(file, line)) {
        if (line == "[scan]") isScan = true;

        //checks every name and when they appear on the line, the value of the name will be assigned to what function will return
        if (isScan) {
            file >> key >> equal_sign >> value;
            if (key == "ranges") break;
            double val = std::stod(value);

            std::cout << key << " " << key.size() << " : " << trim(value) << " " << val << std::endl;

            if (key == "angle_min") scan.angle_min = val;
            else if (key == "angle_max") scan.angle_max = val;
            else if (key == "angle_increment") scan.angle_increment = val;
            else if (key == "time_increment") scan.time_increment = val;
            else if (key == "scan_time") scan.scan_time = val;
            else if (key == "range_min") scan.range_min = val;
            else if (key == "range_max") scan.range_max = val;
            else;
        }
    }
    return scan;
}*/