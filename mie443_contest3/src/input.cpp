#include <iostream>
#include <vector>
#include <string>

using namespace std;

// std::vector<std::string> getUserNames(int numNames) {
//     std::vector<std::string> names;
//     std::string name;

//     for (int i = 0; i < numNames; i++) {
//         std::cout << "Please enter name " << i+1 << ": ";
//         std::getline(std::cin, name);
//         names.push_back(name);
//     }

//     return names;
// }

int main() {
    // int numNames = 1;
    // std::vector<std::string> names = getUserNames(numNames);

    // std::cout << "The names you entered are: ";
    // for (std::vector<std::string>::iterator i = names.begin();
    //     i != names.end();
    //     i++) {
    //     std::cout << *i << " ";

    // }
    // std::cout << std::endl;
    std::string name;

    std::getline(std::cin, name);
    std::cout << name << std::endl;

    return 0;
}