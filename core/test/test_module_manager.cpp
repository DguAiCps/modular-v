#include <gtest/gtest.h>
#include "core/module_manager.hpp"
#include "core/base_module.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace modular_v::core;

// Test module implementation
class TestModule : public BaseModule {
public:
    TestModule(const std::string& name, bool should_fail = false)
        : BaseModule(name, "1.0.0"), should_fail_(should_fail) {}

    int init_count_ = 0;
    int start_count_ = 0;
    int stop_count_ = 0;

protected:
    bool onInitialize() override {
        init_count_++;
        if (should_fail_) return false;
        updateWatchdog();
        return true;
    }

    bool onStart() override {
        start_count_++;
        if (should_fail_) return false;
        updateWatchdog();
        return true;
    }

    bool onStop() override {
        stop_count_++;
        return true;
    }

private:
    bool should_fail_;
};

class ModuleManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }
};

TEST_F(ModuleManagerTest, RegisterModule) {
    ModuleManager mgr;
    auto module = std::make_shared<TestModule>("test_module");

    EXPECT_TRUE(mgr.registerModule("test", module));
    EXPECT_EQ(mgr.getModuleList().size(), 1);
}

TEST_F(ModuleManagerTest, RegisterDuplicateModule) {
    ModuleManager mgr;
    auto module1 = std::make_shared<TestModule>("test_module1");
    auto module2 = std::make_shared<TestModule>("test_module2");

    EXPECT_TRUE(mgr.registerModule("test", module1));
    EXPECT_FALSE(mgr.registerModule("test", module2));  // Duplicate name
}

TEST_F(ModuleManagerTest, UnregisterModule) {
    ModuleManager mgr;
    auto module = std::make_shared<TestModule>("test_module");

    mgr.registerModule("test", module);
    EXPECT_TRUE(mgr.unregisterModule("test"));
    EXPECT_EQ(mgr.getModuleList().size(), 0);
}

TEST_F(ModuleManagerTest, ModuleLifecycle) {
    ModuleManager mgr;
    auto module = std::make_shared<TestModule>("test_module");

    mgr.registerModule("test", module);

    EXPECT_EQ(mgr.getModuleState("test"), ModuleState::UNINITIALIZED);

    EXPECT_TRUE(mgr.initializeSystem());
    EXPECT_EQ(module->init_count_, 1);
    EXPECT_EQ(mgr.getModuleState("test"), ModuleState::INITIALIZED);

    EXPECT_TRUE(mgr.startSystem());
    EXPECT_EQ(module->start_count_, 1);
    EXPECT_EQ(mgr.getModuleState("test"), ModuleState::RUNNING);

    EXPECT_TRUE(mgr.stopSystem());
    EXPECT_EQ(module->stop_count_, 1);
    EXPECT_EQ(mgr.getModuleState("test"), ModuleState::INITIALIZED);
}

TEST_F(ModuleManagerTest, DependencyOrder) {
    ModuleManager mgr;
    auto module_a = std::make_shared<TestModule>("module_a");
    auto module_b = std::make_shared<TestModule>("module_b");
    auto module_c = std::make_shared<TestModule>("module_c");

    // C depends on B, B depends on A
    mgr.registerModule("a", module_a);
    mgr.registerModule("b", module_b, {"a"});
    mgr.registerModule("c", module_c, {"b"});

    EXPECT_TRUE(mgr.initializeSystem());
    EXPECT_TRUE(mgr.startSystem());

    // A should start before B, B before C
    EXPECT_EQ(module_a->start_count_, 1);
    EXPECT_EQ(module_b->start_count_, 1);
    EXPECT_EQ(module_c->start_count_, 1);
}

TEST_F(ModuleManagerTest, CircularDependency) {
    ModuleManager mgr;
    auto module_a = std::make_shared<TestModule>("module_a");
    auto module_b = std::make_shared<TestModule>("module_b");

    // Create circular dependency: A -> B -> A
    mgr.registerModule("a", module_a, {"b"});
    mgr.registerModule("b", module_b, {"a"});

    // Should fail due to circular dependency
    EXPECT_FALSE(mgr.initializeSystem());
}

TEST_F(ModuleManagerTest, MissingDependency) {
    ModuleManager mgr;
    auto module_b = std::make_shared<TestModule>("module_b");

    // B depends on A, but A is not registered
    mgr.registerModule("b", module_b, {"a"});

    EXPECT_TRUE(mgr.initializeSystem());  // Init should succeed

    // Start should fail because dependency is not running
    EXPECT_FALSE(mgr.startModule("b"));
}

TEST_F(ModuleManagerTest, InitializationFailure) {
    ModuleManager mgr;
    auto module_fail = std::make_shared<TestModule>("fail_module", true);

    mgr.registerModule("fail", module_fail);

    // Should fail initialization
    EXPECT_FALSE(mgr.initializeSystem());
}

TEST_F(ModuleManagerTest, EmergencyStop) {
    ModuleManager mgr;
    auto module = std::make_shared<TestModule>("test_module");

    mgr.registerModule("test", module);
    mgr.initializeSystem();
    mgr.startSystem();

    EXPECT_EQ(mgr.getModuleState("test"), ModuleState::RUNNING);

    EXPECT_TRUE(mgr.emergencyStop());

    // Module should be stopped
    EXPECT_NE(mgr.getModuleState("test"), ModuleState::RUNNING);
}

TEST_F(ModuleManagerTest, RestartModule) {
    ModuleManager mgr;
    auto module = std::make_shared<TestModule>("test_module");

    mgr.registerModule("test", module);
    mgr.initializeSystem();
    mgr.startSystem();

    int initial_start_count = module->start_count_;

    EXPECT_TRUE(mgr.restartModule("test"));
    EXPECT_EQ(module->start_count_, initial_start_count + 1);
    EXPECT_EQ(mgr.getModuleState("test"), ModuleState::RUNNING);
}

TEST_F(ModuleManagerTest, HealthCheck) {
    ModuleManager mgr;
    auto module = std::make_shared<TestModule>("test_module");

    mgr.registerModule("test", module);
    mgr.initializeSystem();
    mgr.startSystem();

    EXPECT_TRUE(mgr.checkSystemHealth());

    auto health_status = mgr.getModuleHealthStatus();
    EXPECT_TRUE(health_status["test"]);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
