#include <iostream> 
#include <fstream> 
#include <string> 
#include <vector>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <algorithm>

struct Point2D
{
    double x; 
    double y;
};

struct LidarScanParams
{
    double angle_min = 0.0;
    double angle_max = 0.0;
    double angle_increment = 0.0;
    double range_min = 0.0;
    double range_max = 0.0;
};

void trim(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { // Bastan sona dogru bosluklari siler
        return !std::isspace(ch);
    }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { // Sondan basa dogru bosluklari siler
        return !std::isspace(ch);
    }).base(), s.end());
}

bool parseKeyValue(const std::string& line, const std::string& key, double& value)
{
    std::stringstream ss(line);  
    std::string crrKey;
    char sep;
    
    ss >> crrKey;
    if (crrKey == key) 
    {
        ss >> sep;
        if (sep == '=') // Soyle, ss her baska degere atandiginda onundeki karakter dizisini alir ve bosluklari atlar. Yani tekrar cagirdiginda onceki string dizisini degil de onun onundeki diziyi alir. Eger = karakteri alinmissa sonraki sefer alicagi karakter dizisi istedigimiz deger olacaktir.
        {
            ss >> value;
            return true;
        }
    }
    return false; 
}

// programin baslayacagi yer
int main(int argc, char* argv[])  // Paramterleri disaridan alicak ve curl ile calisabilicek
{
    if (argc < 2) {
        std::cerr << "Hata: TOML dosya adi belirtilmedi." << "Kullanim: " << argv[0] << " <dosya_adi.toml>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    std::ifstream file(filename);

    if (!file.is_open()) // dosya acilmaz ise kullaniciya hata mesaji ver ve 1 return'le
    {
        std::cerr << "Hata: " << filename << " Dosyasi acilamadi." << std::endl;
        return 1;
    }
    LidarScanParams params;
    std::vector<double> ranges;
    std::string line;
    bool inScanSection = false;  
    bool readingRanges = false; 
    // Scan ve Ranges disindaki bolumler bizim icin onemsizdir, dogruca scan ve ranges kisimlarina ulasalim.

    while (std::getline(file, line)) 
    {
        size_t brckt; // bracket(koseli parantez) algilamak icin yazdigim degisken
        double rangeVal;
        trim(line);
        if (line.empty()) {
            continue;
        }

        if (line == "[scan]") { 
            inScanSection = true; 
            continue; 
        } else if (line[0] == '[') { 
            inScanSection = false; 
            continue; 
        }

        if (readingRanges) {
            if (line.find(']') != std::string::npos) {
                readingRanges = false; 
                line = line.substr(0, line.find(']'));
            }

            std::replace(line.begin(), line.end(), ',', ' ');
            std::stringstream ss(line);
            double rangevalue;
            while (ss >> rangevalue) {
                ranges.push_back(rangevalue);
            }
        } 
        else if (inScanSection)
        {
            if (!parseKeyValue(line, "angle_min", params.angle_min) &&
                !parseKeyValue(line, "angle_max", params.angle_max) &&
                !parseKeyValue(line, "angle_increment", params.angle_increment) &&
                !parseKeyValue(line, "range_min", params.range_min) &&
                !parseKeyValue(line, "range_max", params.range_max)) {
                
                if (line.rfind("ranges", 0) == 0) {
                    readingRanges = true;
                    brckt = line.find('[');
                    if (brckt != std::string::npos) {
                        line = line.substr(brckt + 1);
                        std::replace(line.begin(), line.end(), ',', ' ');
                        std::stringstream ss(line);
                        while (ss >> rangeVal) {
                            ranges.push_back(rangeVal);
                        }
                    }
                }
            }
        }
    }

    file.close();
    std::vector<Point2D> points;

    for (size_t i = 0; i < ranges.size(); ++i)
    {
        double curRange = ranges[i];

        if (curRange < params.range_min || curRange > params.range_max) {
            continue;
        }

        double curAngle = params.angle_min + i * params.angle_increment;
        Point2D point;
        point.x = curRange * std::cos(curAngle);
        point.y = curRange * std::sin(curAngle);
        points.push_back(point);
    }
    std::cout << "Filtrelenmis ve Donusturulmus Gecerli Noktalar (" << points.size() << " adet):" << std::endl;
    std::cout << std::fixed << std::setprecision(4); 

    for (const auto& p : points)
    {
        std::cout << "(X: " << p.x << ", Y: " << p.y << ")" << std::endl;
    }
    return 0;
}