/*** 
 * @Autor: your name
 * @Data: Do not edit
 * @LastEditTime: 2022-03-30 11:48:31
 * @LastEditors: your name
 * @Brief: 
 * @FilePath: /Notes/tcp_test/src/fork.cpp
 * @Copyright:  
 */
#include <stdio.h>
#include <unistd.h>



int main(){
  
  pid_t pid;
  pid = fork();
  if(pid = 0){
    printf("child pid : %d\n" , getpid());
  }else{
    printf("pid:%d\n",pid);
    printf("farther pid :%d\n",getpid());
  }
}