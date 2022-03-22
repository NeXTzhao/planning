/*** 
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-03-22 20:59:41
 * @LastEditors: your name
 * @Brief: 
 * @FilePath: /demo2/encrypt_and_decrypt/encrypt_and_decrypt_test.cpp
 * @Copyright:  
 */
#include "encrypt_and_decrypt.h"

#include <gtest/gtest.h>

namespace string_operation {

TEST(EncyrptTest, BasicTests) {
  std::string passwords;

  StrEncryptAndDecrypt::Encrypt("abcdefg", passwords);
  EXPECT_STREQ("BCDEFGH", passwords.c_str());

  StrEncryptAndDecrypt::Encrypt("SayHelloToMe2019", passwords);
  EXPECT_STREQ("tBZiFMMPuPnF3120", passwords.c_str());

  StrEncryptAndDecrypt::Encrypt("Are you ok?", passwords);
  EXPECT_STREQ("bSF ZPV PL?", passwords.c_str());

  StrEncryptAndDecrypt::Encrypt("To make use of the new library.", passwords);
  EXPECT_STREQ("uP NBLF VTF PG UIF OFX MJCSBSZ.", passwords.c_str());

  StrEncryptAndDecrypt::Encrypt(
      "This tutorial aims to get you up and running with GoogleTest using "
      "CMake.",
      passwords);
  EXPECT_STREQ(
      "uIJT UVUPSJBM BJNT UP HFU ZPV VQ BOE SVOOJOH XJUI hPPHMFuFTU VTJOH "
      "dnBLF.",
      passwords.c_str());
}

TEST(EncyrptTest, AdvancedTests) {
  std::string passwords;

  StrEncryptAndDecrypt::Encrypt(" ", passwords);
  EXPECT_STREQ(" ", passwords.c_str());

  StrEncryptAndDecrypt::Encrypt("", passwords);
  EXPECT_STREQ("", passwords.c_str());

  StrEncryptAndDecrypt::Encrypt(
      "jlk;fdsa7890341@!@#$^#$&*%%$#KHUrewq B789*^&^$()_,./`TH", passwords);
  EXPECT_STREQ("KML;GETB8901452@!@#$^#$&*%%$#livSFXR c890*^&^$()_,./`ui",
               passwords.c_str());

  StrEncryptAndDecrypt::Encrypt(
      "For this tutorial we will put the library into a subdirectory called "
      "MathFunctions. This directory already contains a header file, "
      "MathFunctions.h, and a source file mysqrt.cxx. The source file has one "
      "function called mysqrt that provides similar functionality to the "
      "compiler's sqrt function.",
      passwords);
  EXPECT_STREQ(
      "gPS UIJT UVUPSJBM XF XJMM QVU UIF MJCSBSZ JOUP B TVCEJSFDUPSZ DBMMFE "
      "nBUIgVODUJPOT. uIJT EJSFDUPSZ BMSFBEZ DPOUBJOT B IFBEFS GJMF, "
      "nBUIgVODUJPOT.I, BOE B TPVSDF GJMF NZTRSU.DYY. uIF TPVSDF GJMF IBT POF "
      "GVODUJPO DBMMFE NZTRSU UIBU QSPWJEFT TJNJMBS GVODUJPOBMJUZ UP UIF "
      "DPNQJMFS'T TRSU GVODUJPO.",
      passwords.c_str());
}

TEST(DecryptTest, BasicTests) {
  std::string contents;

  StrEncryptAndDecrypt::Decrypt("BCDEFGH", contents);
  EXPECT_STREQ("abcdefg", contents.c_str());

  StrEncryptAndDecrypt::Decrypt("tBZiFMMPuPnF3120", contents);
  EXPECT_STREQ("SayHelloToMe2019", contents.c_str());

  StrEncryptAndDecrypt::Decrypt("bSF ZPV PL?", contents);
  EXPECT_STREQ("Are you ok?", contents.c_str());

  StrEncryptAndDecrypt::Decrypt("uP NBLF VTF PG UIF OFX MJCSBSZ.", contents);
  EXPECT_STREQ("To make use of the new library.", contents.c_str());
}

TEST(DecryptTest, AdvancedTests) {
  std::string contents;

  StrEncryptAndDecrypt::Decrypt(" ", contents);
  EXPECT_STREQ(" ", contents.c_str());

  StrEncryptAndDecrypt::Decrypt("", contents);
  EXPECT_STREQ("", contents.c_str());

  StrEncryptAndDecrypt::Decrypt(
      "KML;GETB8901452@!@#$^#$&*%%$#livSFXR c890*^&^$()_,./`ui", contents);
  EXPECT_STREQ("jlk;fdsa7890341@!@#$^#$&*%%$#KHUrewq B789*^&^$()_,./`TH",
               contents.c_str());

  StrEncryptAndDecrypt::Decrypt(
      "gPS UIJT UVUPSJBM XF XJMM QVU UIF MJCSBSZ JOUP B TVCEJSFDUPSZ DBMMFE "
      "nBUIgVODUJPOT. uIJT EJSFDUPSZ BMSFBEZ DPOUBJOT B IFBEFS GJMF, "
      "nBUIgVODUJPOT.I, BOE B TPVSDF GJMF NZTRSU.DYY. uIF TPVSDF GJMF IBT POF "
      "GVODUJPO DBMMFE NZTRSU UIBU QSPWJEFT TJNJMBS GVODUJPOBMJUZ UP UIF "
      "DPNQJMFS'T TRSU GVODUJPO.",
      contents);
  EXPECT_STREQ(
      "For this tutorial we will put the library into a subdirectory called "
      "MathFunctions. This directory already contains a header file, "
      "MathFunctions.h, and a source file mysqrt.cxx. The source file has one "
      "function called mysqrt that provides similar functionality to the "
      "compiler's sqrt function.",
      contents.c_str());
}

TEST(DecryptTest ,AdvanceTest1){
    std::string contents;
    StrEncryptAndDecrypt::Decrypt("BCD",contents);
    EXPECT_STREQ("abc", contents.c_str())<<"result is ok";
}

}  // namespace string_operation