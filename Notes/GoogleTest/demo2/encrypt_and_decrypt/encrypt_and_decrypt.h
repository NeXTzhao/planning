#pragma once

#include <string>

namespace string_operation {
class StrEncryptAndDecrypt {
 public:
  static void Encrypt(const std::string& contents, std::string& passwords);
  static void Decrypt(const std::string& passwords, std::string& contents);

 private:
  /* data */
};

}  // namespace string_operation
