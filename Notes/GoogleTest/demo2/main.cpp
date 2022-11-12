#include <iostream>
#include <string>

#include "encrypt_and_decrypt/encrypt_and_decrypt.h"

using string_operation::StrEncryptAndDecrypt;

int main(int, char**) {
  std::string in_contents, out_contents, in_passwords, out_passwords;
  std::cout << "Please input the contents to be encrypted: " << std::endl;
  if (std::getline(std::cin, in_contents)) {
    StrEncryptAndDecrypt::Encrypt(in_contents, out_passwords);
    std::cout << "The encrypted contents are: " << std::endl;
    std::cout << out_passwords << std::endl;
  }
  
  std::cout << "Please input the contents to be decrypted: " << std::endl;
  if (std::getline(std::cin, in_passwords)) {
    StrEncryptAndDecrypt::Decrypt(in_passwords, out_contents);
    std::cout << "The decrypted contents are: " << std::endl;
    std::cout << out_contents << std::endl;
  }

  return 0;
}