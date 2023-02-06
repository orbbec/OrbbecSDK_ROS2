#include <fcntl.h>
#include <semaphore.h>
#include <sys/shm.h>

#include <cstring>
#include <iostream>

#include "orbbec_camera/constants.h"
#include "rclcpp/rclcpp.hpp"

using namespace orbbec_camera;

int main() {
  sem_t *sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 0);
  if (sem == SEM_FAILED) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("cleanup_shm"), "sem_open failed: " << strerror(errno));
    return 1;
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("cleanup_shm"), "sem_open succeeded");
  sem_close(sem);
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  RCLCPP_INFO_STREAM(rclcpp::get_logger("cleanup_shm"), "sem_unlink succeeded");
  int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
  if (shm_id != -1) {
    shmctl(shm_id, IPC_RMID, nullptr);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("cleanup_shm"), "shmctl `IPC_RMID` succeeded");
  }
  return 0;
}
