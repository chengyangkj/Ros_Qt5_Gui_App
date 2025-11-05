#include <gtest/gtest.h>
#include "core/framework/framework.h"
#include <string>
#include <vector>
#include <map>
#include <atomic>
#include <thread>
#include <chrono>

struct TestStruct {
  int id;
  std::string name;
  double value;
  
  bool operator==(const TestStruct& other) const {
    return id == other.id && name == other.name && value == other.value;
  }
};

class MessageBusTest : public ::testing::Test {
 protected:
  void SetUp() override {
    GetMessageBusInstance()->Clear();
  }
  
  void TearDown() override {
    GetMessageBusInstance()->Clear();
  }
};

TEST_F(MessageBusTest, BasicIntPublishSubscribe) {
  std::atomic<int> received_value(0);
  
  SUBSCRIBE("test_int", [&received_value](const int& value) {
    received_value = value;
  });
  
  int test_value = 42;
  PUBLISH("test_int", test_value);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_value.load(), 42);
}

TEST_F(MessageBusTest, BasicDoublePublishSubscribe) {
  std::atomic<double> received_value(0.0);
  
  SUBSCRIBE("test_double", [&received_value](const double& value) {
    received_value = value;
  });
  
  double test_value = 3.14159;
  PUBLISH("test_double", test_value);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_DOUBLE_EQ(received_value.load(), 3.14159);
}

TEST_F(MessageBusTest, BasicStringPublishSubscribe) {
  std::string received_value;
  
  SUBSCRIBE("test_string", [&received_value](const std::string& value) {
    received_value = value;
  });
  
  std::string test_value = "Hello Framework";
  PUBLISH("test_string", test_value);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_value, "Hello Framework");
}

TEST_F(MessageBusTest, CustomStructPublishSubscribe) {
  TestStruct received_value{0, "", 0.0};
  
  SUBSCRIBE("test_struct", [&received_value](const TestStruct& value) {
    received_value = value;
  });
  
  TestStruct test_value{123, "test_name", 45.67};
  PUBLISH("test_struct", test_value);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_value.id, 123);
  EXPECT_EQ(received_value.name, "test_name");
  EXPECT_DOUBLE_EQ(received_value.value, 45.67);
}

TEST_F(MessageBusTest, VectorPublishSubscribe) {
  std::vector<int> received_value;
  
  SUBSCRIBE("test_vector", [&received_value](const std::vector<int>& value) {
    received_value = value;
  });
  
  std::vector<int> test_value{1, 2, 3, 4, 5};
  PUBLISH("test_vector", test_value);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_value.size(), 5);
  EXPECT_EQ(received_value[0], 1);
  EXPECT_EQ(received_value[4], 5);
}

TEST_F(MessageBusTest, MapPublishSubscribe) {
  std::map<std::string, int> received_value;
  
  SUBSCRIBE("test_map", [&received_value](const std::map<std::string, int>& value) {
    received_value = value;
  });
  
  std::map<std::string, int> test_value{{"one", 1}, {"two", 2}, {"three", 3}};
  PUBLISH("test_map", test_value);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_value.size(), 3);
  EXPECT_EQ(received_value["one"], 1);
  EXPECT_EQ(received_value["two"], 2);
  EXPECT_EQ(received_value["three"], 3);
}

TEST_F(MessageBusTest, MultipleSubscribers) {
  std::atomic<int> received_count1(0);
  std::atomic<int> received_count2(0);
  
  SUBSCRIBE("test_multi", [&received_count1](const int& value) {
    received_count1 = value;
  });
  
  SUBSCRIBE("test_multi", [&received_count2](const int& value) {
    received_count2 = value;
  });
  
  int test_value = 100;
  PUBLISH("test_multi", test_value);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_count1.load(), 100);
  EXPECT_EQ(received_count2.load(), 100);
}

TEST_F(MessageBusTest, TypeMismatchIgnored) {
  std::atomic<int> received_int(0);
  std::atomic<double> received_double(0.0);
  
  SUBSCRIBE("test_type_mismatch", [&received_int](const int& value) {
    received_int = value;
  });
  
  SUBSCRIBE("test_type_mismatch", [&received_double](const double& value) {
    received_double = value;
  });
  
  int int_value = 42;
  PUBLISH("test_type_mismatch", int_value);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_int.load(), 42);
  EXPECT_DOUBLE_EQ(received_double.load(), 0.0);
  
  double double_value = 3.14;
  PUBLISH("test_type_mismatch", double_value);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_int.load(), 42);
  EXPECT_DOUBLE_EQ(received_double.load(), 3.14);
}

TEST_F(MessageBusTest, Unsubscribe) {
  std::atomic<int> received_count(0);
  
  auto id = SUBSCRIBE("test_unsubscribe", [&received_count](const int& value) {
    received_count = value;
  });
  
  int test_value1 = 10;
  PUBLISH("test_unsubscribe", test_value1);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_count.load(), 10);
  
  UNSUBSCRIBE("test_unsubscribe", id);
  
  int test_value2 = 20;
  PUBLISH("test_unsubscribe", test_value2);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_count.load(), 10);
}

TEST_F(MessageBusTest, DifferentTopics) {
  std::atomic<int> topic1_value(0);
  std::atomic<int> topic2_value(0);
  
  SUBSCRIBE("topic1", [&topic1_value](const int& value) {
    topic1_value = value;
  });
  
  SUBSCRIBE("topic2", [&topic2_value](const int& value) {
    topic2_value = value;
  });
  
  PUBLISH("topic1", 100);
  PUBLISH("topic2", 200);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(topic1_value.load(), 100);
  EXPECT_EQ(topic2_value.load(), 200);
}

TEST_F(MessageBusTest, ConstReference) {
  std::string received_value;
  
  SUBSCRIBE("test_const_ref", [&received_value](const std::string& value) {
    received_value = value;
  });
  
  const std::string test_value = "const_string";
  PUBLISH("test_const_ref", test_value);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_value, "const_string");
}

TEST_F(MessageBusTest, LargeData) {
  std::vector<int> received_value;
  
  SUBSCRIBE("test_large", [&received_value](const std::vector<int>& value) {
    received_value = value;
  });
  
  std::vector<int> large_data(10000, 42);
  PUBLISH("test_large", large_data);
  
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  EXPECT_EQ(received_value.size(), 10000);
  EXPECT_EQ(received_value[0], 42);
  EXPECT_EQ(received_value[9999], 42);
}

