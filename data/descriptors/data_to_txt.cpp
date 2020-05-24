#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>

int main(int argv, char** argc)
{
    std::ifstream points(argc[1]);
    std::ifstream massif(argc[2]);

    std::vector<unsigned long> massifOut;
    std::vector<unsigned long> pointsOut;

    std::string line;

    //read massif
    std::string massifHeapLine = "mem_heap_B";
    while( std::getline(massif, line) )
    {
        if( line.find(massifHeapLine) == std::string::npos )
            continue;
        else
            massifOut.emplace_back( std::stoul(line.substr(line.find('=') + 1)) );
    }

    //read points
    while( std::getline(points, line) )
    {
        pointsOut.emplace_back( std::stoul(line) );
    }

    //write to graph file
    std::ofstream out("SHOT.txt");
    size_t gap = std::floor(static_cast<double>(pointsOut.size()) / static_cast<double>(massifOut.size()));
    for( size_t i = 0; i < massifOut.size(); ++i) {
        if (i == 0)
            continue;

        out << pointsOut[i * gap] << ' ' << massifOut[i]*1e-6 << std::endl;
    }

    out.close();

    return 0;
}
