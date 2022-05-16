//
// Created by next on 22-5-11.
//
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class GameCharacter;

int defaultHealthCalc(const GameCharacter &gc){}

class GameCharacter {
public:
    typedef int (*HealthCalcFunc)(const GameCharacter &);

    explicit GameCharacter(HealthCalcFunc hcf = defaultHealthCalc) : healthFunc(hcf) {}


    int healthValue() const {
        return healthFunc(*this);
    }

private:
    HealthCalcFunc healthFunc;
};

class Solution {
private:
    static int cmp(int a, int b){
        return abs(a) > abs(b);
    }
public:
    int largestSumAfterKNegations(vector<int>& nums, int k) {
        sort(nums.begin(), nums.end(), [](const int& a, const int& b){
            return abs(a) > abs(b);
        });


        for(int i = 0; i < nums.size(); ++i){
            if(nums[i] < 0 && k > 0){
                nums[i] *= -1;
                --k;
            }
        }
        if(k % 2 == 1) nums[nums.size() - 1] *= -1;
        int result;
        for(const auto& item : nums) result += item;
        return result;
    }
};

int main() {
//    GameCharacter a;

Solution su;

}