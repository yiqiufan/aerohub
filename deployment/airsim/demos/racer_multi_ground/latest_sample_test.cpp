#include <exploration_manager/latest_sample.h>
#include <thread>
#include <stdexcept>
#include <iostream>
int main() {
 fast_planner::LatestSample<int> inbox;
 auto held=std::make_shared<const int>(0); inbox.receive(held);
 auto first=inbox.take();
 std::thread receiver([&]{ for(int i=1;i<=10000;++i) inbox.receive(std::make_shared<const int>(i)); });
 receiver.join(); // Simulates the consumer being occupied by a synchronous solve.
 auto latest=inbox.take();
 if(!first || *first!=0 || !latest || *latest!=10000 || inbox.take()) throw std::runtime_error("latest-only isolation failed");
 inbox.receive(std::make_shared<const int>(10001));
 if(*inbox.take()!=10001 || *first!=0) throw std::runtime_error("snapshot mutation or lost future sample");
 std::cout << "latest-only burst, immutable held snapshot, drain and subsequent sample: passed\n";
}
