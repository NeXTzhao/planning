//
// Created by next on 22-4-18.
//

#include <iostream>
#include <vector>

using namespace std;

struct TreeNode {
    int val;
    TreeNode *left;
    TreeNode *right;

    TreeNode() : val(0), left(nullptr), right(nullptr) {}

    TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}

    TreeNode(int x, TreeNode *left, TreeNode *right) : val(x), left(left), right(right) {}
};

class Solution {
private:
    TreeNode *convToTree(vector<int> &nums, int left, int right) {
        if (left > right) return nullptr;
        int mid = left + (right - left) / 2;
        TreeNode *root = new TreeNode(nums[mid]);

        root->left = convToTree(nums, left, mid - 1);
        root->right = convToTree(nums, mid + 1, right);

        return root;
    }

public:
    TreeNode *sortedArrayToBST(vector<int> &nums) {
        TreeNode *root = convToTree(nums, 0, nums.size() - 1);
        return root;
    }
};

int main(){
    vector<int> arr= {-10,-3,0,5,9};
    Solution su;
    su.sortedArrayToBST(arr);

}