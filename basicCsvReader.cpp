#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

/* Approach to read csv
1.using getline(), file pointer and '\n' as the delimiter, read an an entire row and store it in a string variables,
2. using stringstream, seperate the row into words,
3. using again getline(), the stringstream pointer and ',' as the delimter, read every word in the row, and store it in a string variable and push that variable to a string vector,
4. retrieve the column data through row[index].
*/
using namespace std;
void say_hello(){
    std::cout << "Hello, from hash-real-data!\n";
}

int main(int argc, char const *argv[])
{   
    auto render = [](auto collection) {
        for(const auto &val : collection){
            cout<< val << endl;
        }
    };
    fstream fin;
    fin.open("traj.csv", ios::in);
    
    string line, word;
    vector<string> row;
    getline(fin, line);
    //for every line
    for(int i =1; i<2; i++){
        
        getline(fin, line);
        vector<string> roww;
        stringstream s(line);
        //for every word
        while(getline(s,word,',')){
            roww.push_back(word);
        }
        roww.swap(row);
    }
    render(row);
    
    return 0;
}