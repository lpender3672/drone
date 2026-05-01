#include <gtest/gtest.h>
#include "core/block.hpp"
#include "mocks/mock_block.h"

using shared::IPort;
using shared::InputPort;
using shared::OutputPort;
using mocks::MockBlock;

TEST(IPort, OutputPortReportsItsNameThroughBaseInterface) {
    OutputPort<int> p("x");
    IPort* base = &p;
    EXPECT_EQ(base->port_name(), "x");
}

TEST(IPort, InputPortReportsItsNameThroughBaseInterface) {
    InputPort<double> p("y");
    IPort* base = &p;
    EXPECT_EQ(base->port_name(), "y");
}

TEST(IPort, PortsOfDifferentTypeReportDistinctTypeIds) {
    OutputPort<int>    pi("a");
    OutputPort<double> pd("b");
    EXPECT_NE(pi.type_id(), pd.type_id());
}

TEST(IPort, PortsOfSameTypeShareTypeId) {
    InputPort<int>    pi("a");
    OutputPort<int>   po("b");
    EXPECT_EQ(pi.type_id(), po.type_id());  // same T → same type_id (pointer equality)
}

TEST(IPort, TypeIdIsStableAcrossInstances) {
    InputPort<double>  a("a");
    InputPort<double>  b("b");
    EXPECT_EQ(a.type_id(), b.type_id());  // tag is per-T, not per-instance
}

TEST(IPort, IsInputDistinguishesDirection) {
    OutputPort<int> out("o");
    InputPort<int>  in("i");
    EXPECT_FALSE(out.is_input());
    EXPECT_TRUE(in.is_input());
}

TEST(IPort, OutputConnectToWiresMatchingInput) {
    OutputPort<int> out("o");
    InputPort<int>  in("i");
    IPort& out_ref = out;
    IPort& in_ref  = in;

    out_ref.connect_to(in_ref);

    out.set(7);
    EXPECT_EQ(in.get(), 7);  // zero-copy through the source pointer
}

TEST(IPort, ConnectToRejectsTypeMismatch) {
    OutputPort<int>    out("o");
    InputPort<double>  in("i");
    EXPECT_THROW(static_cast<IPort&>(out).connect_to(in), std::invalid_argument);
}

TEST(IPort, ConnectToRejectsDirectionMismatch) {
    OutputPort<int> out("o");
    OutputPort<int> other_out("o2");
    EXPECT_THROW(static_cast<IPort&>(out).connect_to(other_out), std::invalid_argument);

    InputPort<int> in("i");
    InputPort<int> in2("i2");
    // calling connect_to on an input doesn't make sense — only outputs can drive
    EXPECT_THROW(static_cast<IPort&>(in).connect_to(in2), std::invalid_argument);
}

// Block::port(name) lookup — TypedBlock auto-registers its input_/output_.
TEST(Block, PortLookupReturnsRegisteredPorts) {
    MockBlock<> a("a");

    IPort* in_port  = a.port("in");
    IPort* out_port = a.port("out");

    ASSERT_NE(in_port, nullptr);
    ASSERT_NE(out_port, nullptr);
    EXPECT_TRUE(in_port->is_input());
    EXPECT_FALSE(out_port->is_input());
    EXPECT_EQ(in_port->port_name(),  "in");
    EXPECT_EQ(out_port->port_name(), "out");
}

TEST(Block, PortLookupReturnsNullForUnknownName) {
    MockBlock<> a("a");
    EXPECT_EQ(a.port("missing"), nullptr);
}
