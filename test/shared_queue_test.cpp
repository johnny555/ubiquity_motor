/**
Copyright (c) 2016, Ubiquity Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include <gtest/gtest.h>

#include <ubiquity_motor/shared_queue.hpp>
#include <boost/thread.hpp>
#include <vector>

TEST(SharedQueueTest, emptyOnConstuct) {
  shared_queue<int> sq;

  ASSERT_TRUE(sq.fast_empty());
  ASSERT_TRUE(sq.empty());
}

TEST(SharedQueueTest, notEmptyAfterPush) {
  shared_queue<int> sq;
  sq.push(25);

  ASSERT_FALSE(sq.fast_empty());
  ASSERT_FALSE(sq.empty());
}

TEST(SharedQueueTest, notEmptyAfterPushMultiple) {
  shared_queue<int> sq;
  std::vector<int> v;

  for (int i = 0; i < 5; ++i) {
    v.push_back(i);
  }

  sq.push(v);

  ASSERT_FALSE(sq.fast_empty());
  ASSERT_FALSE(sq.empty());
}

TEST(SharedQueueTest, emptyAfterPushAndPop) {
  shared_queue<int> sq;
  sq.push(25);

  ASSERT_FALSE(sq.fast_empty());
  ASSERT_FALSE(sq.empty());

  sq.pop();

  ASSERT_TRUE(sq.fast_empty());
  ASSERT_TRUE(sq.empty());
}

TEST(SharedQueueTest, pushFrontPop) {
  shared_queue<int> sq;

  for (int i = 0; i < 5; ++i) {
    sq.push(i);
  }

  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(i, sq.front());
    sq.pop();
  }

  for (int i = 0; i < 5; ++i) {
    sq.push(i);
  }

  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(i, sq.front_pop());
  }

  ASSERT_TRUE(sq.fast_empty());
  ASSERT_TRUE(sq.empty());
  ASSERT_EQ(0, (int)sq.size());
}

TEST(SharedQueueTest, copy) {
  shared_queue<int> sq0;
  shared_queue<int> sq1;

  for (int i = 0; i < 5; ++i) {
    sq0.push(i);
  }

  sq1 = sq0;
  ASSERT_FALSE(sq1.fast_empty());
  ASSERT_FALSE(sq1.empty());
  ASSERT_EQ(5, (int)sq1.size());

  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(i, sq1.front_pop());
  }

  ASSERT_TRUE(sq1.fast_empty());
  ASSERT_TRUE(sq1.empty());
  ASSERT_EQ(0, (int)sq1.size());

  // Make sure that the original queue is untouched
  ASSERT_FALSE(sq0.fast_empty());
  ASSERT_FALSE(sq0.empty());
  ASSERT_EQ(5, (int)sq0.size());
}

TEST(SharedQueueTest, copyConstruct) {
  shared_queue<int> sq0;

  for (int i = 0; i < 5; ++i) {
    sq0.push(i);
  }

  shared_queue<int> sq1(sq0);
  ASSERT_FALSE(sq1.fast_empty());
  ASSERT_FALSE(sq1.empty());
  ASSERT_EQ(5, (int)sq1.size());

  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(i, sq1.front_pop());
  }

  ASSERT_TRUE(sq1.fast_empty());
  ASSERT_TRUE(sq1.empty());
  ASSERT_EQ(0, (int)sq1.size());

  // Make sure that the original queue is untouched
  ASSERT_FALSE(sq0.fast_empty());
  ASSERT_FALSE(sq0.empty());
  ASSERT_EQ(5, (int)sq0.size());
}

TEST(SharedQueueTest, pushConstCopyFront) {
  shared_queue<int> sq;

  for (int i = 0; i < 5; ++i) {
    sq.push(i);
  }

  const shared_queue<int> csq = sq;

  ASSERT_FALSE(sq.fast_empty());
  ASSERT_FALSE(sq.empty());
  ASSERT_EQ(5, (int)sq.size());

  const int & front = csq.front();
  ASSERT_EQ(0, front);
}

TEST(SharedQueueTest, pushPopSize) {
  shared_queue<int> sq;

  for (int i = 0; i < 5; ++i) {
    sq.push(i);
    ASSERT_EQ(i + 1, (int)sq.size());
  }

  ASSERT_EQ(
    5, (int)

    sq.size());

  for (int i = 0; i < 5; ++i) {
    ASSERT_EQ(5 - i, (int)sq.size());
    sq.pop();
  }
}

TEST(SharedQueueTest, pushMultipleSize) {
  shared_queue<int> sq;
  std::vector<int> v;

  for (int i = 0; i < 5; ++i) {
    v.push_back(i);
  }

  sq.push(v);
  ASSERT_EQ(5, (int)sq.size());
}

void pop_thread(shared_queue<int> * sq)
{
  for (int i = 0; i < 100; ++i) {
    ASSERT_FALSE(sq->fast_empty());
    ASSERT_FALSE(sq->empty());
    ASSERT_EQ(i, sq->front());
    sq->pop();
  }
}

TEST(SharedQueueTest, pushFrontPopThreaded) {
  shared_queue<int> sq;

  for (int i = 0; i < 100; ++i) {
    sq.push(i);
  }

  boost::thread popThread = boost::thread(pop_thread, &sq);
  popThread.join();

  ASSERT_TRUE(sq.fast_empty());
  ASSERT_TRUE(sq.empty());
  ASSERT_EQ(0, (int)sq.size());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
