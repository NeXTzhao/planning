//
// Created by next on 2022/4/7.
//

#include <cassert>

class Plus {
public:
    Plus(const int data) :
            data(data) {}

    int operator()(const int value) const {
        return value + data;
    }

private:
    const int data;
};

int main() {
    Plus plusOne(1);
    assert(plusOne(2) == 3);

    Plus plusTwo(2);
    assert(plusTwo(2) == 4);

    return 0;
}
