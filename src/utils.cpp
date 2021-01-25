#include "utils.h"

constexpr double pi() { 
    return M_PI; 
}

double deg2rad(double x) { 
    return x * pi() / 180; 
}

double rad2deg(double x) { 
    return x * 180 / pi(); 
}

string hasData(string s){
    // If there is data the JSON object in string format will be returned,
    // else the empty string "" will be returned.

    auto found_null = s.find("null"); // Search for null in string
    auto b1 = s.find_first_of("["); // Search for start
    auto b2 = s.rfind("}]"); // Search for ending

    if(found_null != string::npos){
        // Found null, meaning invalid string data
        return "";
    } else if ( b1 != string::npos && b2 != string::npos) {
        // string substr (size_t pos = 0, size_t len = npos) const;
        // return string starting from b1 and ending at b2.
        return s.substr(b1, b2 - b1 + 2);
    }

    return "";

}