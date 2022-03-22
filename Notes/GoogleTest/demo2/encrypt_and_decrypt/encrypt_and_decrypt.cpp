/***
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-03-22 18:21:47
 * @LastEditors: your name
 * @Brief:
 * @FilePath: /demo2/encrypt_and_decrypt/encrypt_and_decrypt.cpp
 * @Copyright:
 */
#include "encrypt_and_decrypt.h"

#include <algorithm>

namespace string_operation {
namespace {
const std::string A_TABLE = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const std::string D_TABLE = "0123456789";
}  // namespace

void StrEncryptAndDecrypt::Encrypt(const std::string& contents,
                                   std::string& passwords) {
  if (contents.empty()) {
    passwords.clear();
    return;
  }
  passwords = contents;
  char content = '0';
  char password = '0';
  for (std::size_t i = 0; i < contents.size(); ++i) {
    content = contents[i];
    password = content;
    if (std::isalpha(content)) {
      auto pos = A_TABLE.find(std::toupper(content));
      if (std::string::npos != pos) {
        password = pos < (A_TABLE.size() - 1) ? A_TABLE[pos + 1] : A_TABLE[0];
        if (std::isupper(content)) {
          password = std::tolower(password);
        }
      }
    } else if (std::isdigit(content)) {
      auto pos = D_TABLE.find(content);
      if (std::string::npos != pos) {
        password = pos < (D_TABLE.size() - 1) ? D_TABLE[pos + 1] : D_TABLE[0];
      }
    } else {
      continue;
    }
    passwords[i] = password;
  }
}

void StrEncryptAndDecrypt::Decrypt(const std::string& passwords,
                                   std::string& contents) {
  if (passwords.empty()) {
    contents.clear();
    return;
  }
  contents = passwords;
  char content = '0';
  char password = '0';
  for (std::size_t i = 0; i < passwords.size(); ++i) {
    password = passwords[i];
    content = password;
    if (std::isalpha(password)) {
      auto pos = A_TABLE.find(std::toupper(password));
      if (std::string::npos != pos) {
        content = pos > 0 ? A_TABLE[pos - 1] : A_TABLE[A_TABLE.size() - 1];
        if (std::isupper(password)) {
          content = std::tolower(content);
        }
      }
    } else if (std::isdigit(password)) {
      auto pos = D_TABLE.find(password);
      if (std::string::npos != pos) {
        content = pos > 0 ? D_TABLE[pos - 1] : D_TABLE[D_TABLE.size() - 1];
      }
    } else {
      continue;
    }
    contents[i] = content;
  }
}
}  // namespace string_operation