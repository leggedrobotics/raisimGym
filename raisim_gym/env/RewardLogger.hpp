//
// Created by jemin on 5/18/19.
// MIT License
//
// Copyright (c) 2019-2019 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef _RAISIM_GYM_REWARDLOGGER_HPP
#define _RAISIM_GYM_REWARDLOGGER_HPP

#include <unordered_map>

namespace raisim {

class RewardLogger {

  class RewardTerm {
   public:
    RewardTerm() = default;

    void clean() {
      sum = 0.;
      count = 0;
      values.clear();
    }

    void log(double value) {
      values.push_back(value);
      sum += value;
      count++;
    }

    double sum = 0.;
    int count = 0;
    std::vector<double> values;
  };

 public:
  RewardLogger() = default;

  void init(std::initializer_list<std::string> rewardTermNames) {
    for (auto &item : rewardTermNames)
      rewardTerms_[item] = RewardTerm();
  }

  void log(const std::string &termName, double value) {
    rewardTerms_[termName].log(value);
  }

  const std::unordered_map<std::string, RewardTerm>& getRewardTerms() const {
    return rewardTerms_;
  }

  void clean() {
    for (auto& item : rewardTerms_)
      item.second.clean();
  }

 private:
  std::unordered_map<std::string, RewardTerm> rewardTerms_;
};

};
#endif //_RAISIM_GYM_REWARDLOGGER_HPP
