// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GAZEBO_PROCESS_HPP_
#define GAZEBO_PROCESS_HPP_

#include <unistd.h>
#include <stdlib.h>

#include <gtest/gtest.h>

#include <utility>
#include <vector>

namespace gazebo_ros
{

/// Helper class to run gzserver in a seperate process and later terminate that process
class GazeboProcess
{
public:
  /// Start gzserver with a list of arugments
  /// \note The path and --verbose are automaticaly added
  explicit GazeboProcess(const std::vector<const char *> & args);

  ~GazeboProcess();

  /// Start gzserver with the arguments passed to constructuor
  /// \return The result of fork(), either the pid of gazebo process or error if < 0
  int run();

  /// Terminate the child gzserve process
  /// \return -1 if run() failed or has not been called,
  int terminate();

private:
  /// Arguments to run gzserver with
  std::vector<const char *> arguments;

  // pid of gzserver
  int pid_ = -1;
};

GazeboProcess::GazeboProcess(const std::vector<const char *> & args)
{
  arguments = {"/usr/bin/gzserver", "--verbose"};
  arguments.insert(arguments.end(), args.begin(), args.end());
  arguments.push_back(nullptr);
}

GazeboProcess::~GazeboProcess()
{
  terminate();
}

int GazeboProcess::run()
{
  // Fork process so gazebo can be run as child
  pid_ = fork();

  // Child process
  if (0 == pid_) {
    // Run gazebo with arguments
    if (execvp("gzserver", const_cast<char **>(arguments.data()))) {
      // Exec failed, cannot return (in seperate process), so just print errno
      printf("gzserver failed with errno=%d", errno);
      exit(1);
    }
  }

  if (pid_ < 0) {
    return errno;
  }

  // Parent process, return pid of child (or error produced by fork())
  return pid_;
}

int GazeboProcess::terminate()
{
  // Return -1
  if (pid_ < 0) {
    return ECHILD;
  }

  // Kill gazebo (simulating ^C command)
  if (kill(pid_, SIGINT)) {
    return errno;
  }

  // Wait for gazebo to terminate
  if (waitpid(pid_, nullptr, 0) < 0) {
    return errno;
  }

  return 0;
}

}  // namespace gazebo_ros

#endif  // GAZEBO_PROCESS_HPP_
