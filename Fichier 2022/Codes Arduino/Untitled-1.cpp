#include <iostream>
#include <stdio.h>

int main() 
{
    using namespace std;
    string s = "aubade";
    for (char const &c : s)
    {
        cout << c << endl;
    }
    return -1;
}